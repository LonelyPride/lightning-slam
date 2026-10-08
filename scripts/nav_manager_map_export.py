#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 lidar_slam 的建图产物目录导出成 navigation_manager(MapStore) 可直接识别的地图包。

设计前提（**只读** navigation_manager 源码得出，本工具不改它任何代码）：
  map_store.cpp 的 MapStore::list()/get() 是**直接扫描** <maps_root>/<map_id>/ 目录，
  一个地图目录要被接受必须同时满足：
    1) <map_id>.yaml 存在，且
         image      —— 词法归一后必须正好等于 "<map_id>.pgm"
         resolution —— std::stod 可解析且 > 0
         origin     —— 必须写成**同一行**的 "[x, y, yaw]"：MapStore::yaml_value() 只读冒号后
                       那一行，yaml-cpp 默认输出的三行块序列 "- x/- y/- z" 会让
                       parse_origin 失败 -> INVALID_MAP
    2) <map_id>.pgm 存在，P5/P2、maxval<=255（MapStore::read_pgm）
    3) <map_id>.posegraph 与 <map_id>.data **必须同时存在**，否则 MAP_NOT_LOCALIZABLE。
       这两个文件 MapStore **不解析**，只校验存在/常规文件/软链接/大小；真正消费它们的是
       /slam_toolbox/deserialize_map（maps.load 时把 "<maps_root>/<id>/<id>" 前缀发给它，
       由 slam_toolbox 自己追加 .posegraph/.data）。
    4) metadata.json 可选：{"name": "..."} 显示名
       points.json   可选：{"points": [{"pointId","name","x","y","yaw","tags"}]} 任务点
    5) 目录内任何文件都不能是软链接；单文件不得超过 max_map_size_mb(默认 100MB)

  MapStore::import_legacy_maps() 还能把平铺目录 <legacy_root>/<id>.{yaml,pgm,posegraph,data}
  导入管理目录，因此本工具提供 --layout store|flat 两种布局。

产物（--layout store，默认；直接落到 maps_root 即被识别，不需要任何导入服务）：
  <maps_root>/<map_id>/
      <map_id>.yaml       内联 origin；image: <map_id>.pgm
      <map_id>.pgm        由 <map-dir>/map.pgm 复制（先校验 P5/P2 + maxval + 载荷长度）
      <map_id>.posegraph  位姿图清单(JSON)：格式版本、地图元信息、位姿文件 sha256
      <map_id>.data       位姿数据本体：keyframes.txt 的**优化后(opt)**位姿，id x y z qx qy qz qw
      metadata.json       {"name": "<name>"}
      points.json         来自 index.txt 的功能点（如 start），形状与 boost::property_tree 一致
      （--with-map-data，默认开）原样复制 map.yaml/map.pgm（lidar_slam 定位端读的原名）
        + index.txt/关键帧*.pcd/global.pcd/keyframes.txt，
        使该目录既归 navigation_manager 管理，也能**直接当 lidar_slam 定位端 system.map_path**
        （TiledMap 读 index.txt + 分块 PCD；LocSystem 读 map.yaml + map.pgm 发布 /map）

用法：
    python3 scripts/nav_manager_map_export.py --map-dir data/new_map --map-id new_map \
        --maps-root /opt/data/maps --name "厂区A"          # 导出到管理目录
    python3 scripts/nav_manager_map_export.py --map-dir data/new_map --map-id new_map --check-only
    python3 scripts/nav_manager_map_export.py --map-dir data/new_map --map-id new_map \
        --maps-root /opt/data/maps --layout flat           # 供 import_legacy_maps 导入
    python3 scripts/nav_manager_map_export.py --map-dir data/new_map --map-id new_map \
        --maps-root /tmp/navmaps --dry-run                 # 先看会写哪些文件

退出码：0 = 全部判据通过；1 = 有判据不通过（--check-only 可直接用于 CI 断言）。
"""
import argparse
import hashlib
import json
import math
import os
import re
import shutil
import sys
import tempfile

# 与 MapStore::is_valid_id 完全一致：^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$
ID_RE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$')
MAX_FILE_BYTES = 100 * 1024 * 1024  # MapStore 默认 max_map_size_mb=100
POSEGRAPH_FORMAT = 'lidar_slam_posegraph'
POSEGRAPH_VERSION = 1


def _die(msg):
    print('ERROR: ' + msg, file=sys.stderr)
    sys.exit(2)


def sanitize_id(text):
    """把任意名字压成 MapStore 认可的 id；压不出来时返回 None。"""
    if text is None:
        return None
    s = re.sub(r'[^A-Za-z0-9_-]', '_', str(text).strip())
    return s if s and ID_RE.match(s) else None


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()
def read_map_yaml(map_dir):
    """读建图产物 map.yaml -> dict(image, resolution, origin_x, origin_y, origin_yaw, width, height)。

    兼容 origin 的两种写法：单行 [x, y, z] 与 yaml-cpp 默认的三行块序列（- x/- y/- z）。
    """
    path = os.path.join(map_dir, 'map.yaml')
    if not os.path.isfile(path):
        _die('找不到 %s（请先跑 run_slam_online 建图并保存地图）' % path)
    info = {'image': '', 'resolution': None, 'origin_x': 0.0, 'origin_y': 0.0,
            'origin_yaw': 0.0, 'width': None, 'height': None}
    with open(path, 'r') as f:
        lines = f.readlines()
    for i, raw in enumerate(lines):
        line = raw.strip()
        if line.startswith('image:'):
            info['image'] = line.split(':', 1)[1].strip().strip('\'"')
        elif line.startswith('resolution:'):
            try:
                info['resolution'] = float(line.split(':', 1)[1].strip())
            except ValueError:
                _die('%s 里 resolution 无法解析: %r' % (path, line))
        elif line.startswith('width:'):
            info['width'] = int(float(line.split(':', 1)[1].strip()))
        elif line.startswith('height:'):
            info['height'] = int(float(line.split(':', 1)[1].strip()))
        elif line.startswith('origin:'):
            tail = line.split(':', 1)[1].strip()
            if tail.startswith('['):
                vals = [float(v) for v in tail.strip('[]').replace(',', ' ').split()]
            else:
                vals = []
                for k in range(1, 4):
                    if i + k < len(lines):
                        vals.append(float(lines[i + k].split('-', 1)[1].strip()))
            if len(vals) < 3:
                _die('%s 里 origin 只有 %d 个分量' % (path, len(vals)))
            info['origin_x'], info['origin_y'], info['origin_yaw'] = vals[0], vals[1], vals[2]
    if info['resolution'] is None or info['resolution'] <= 0.0:
        _die('%s 里 resolution 缺失或 <= 0' % path)
    return info


def read_pgm(path):
    """解析 PGM 头，返回 (magic, width, height, maxval, payload_offset)；P5 校验载荷长度。"""
    if not os.path.isfile(path):
        _die('找不到 %s' % path)
    with open(path, 'rb') as f:
        data = f.read()
    tokens, i, payload_start = [], 0, 0
    while len(tokens) < 4:
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while i < len(data) and data[i:i + 1] != b'\n':
                i += 1
            continue
        start = i
        while i < len(data) and not data[i:i + 1].isspace():
            i += 1
        tokens.append(data[start:i])
        if len(tokens) == 4:
            payload_start = i + 1
    magic = tokens[0].decode('ascii', 'replace')
    if magic not in ('P5', 'P2'):
        _die('%s 不是 P5/P2（MapStore::read_pgm 只认这两种）: %s' % (path, magic))
    width, height, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    if width <= 0 or height <= 0 or not (0 < maxval <= 255):
        _die('%s 头非法: %dx%d maxval=%d' % (path, width, height, maxval))
    if magic == 'P5' and len(data) - payload_start < width * height:
        _die('%s 载荷不完整: 需要 %d 字节, 实有 %d' % (path, width * height, len(data) - payload_start))
    return magic, width, height, maxval, payload_start
def read_keyframes(path):
    """读 keyframes.txt 取**优化后(opt)**位姿 -> [(id, x, y, z, qx, qy, qz, qw), ...]。

    列定义（见 SlamSystem::SaveMap）：
      id lio_xyz(1-3) opt_xyz(4-6) dpos(7) dang_deg(8) lio_q(9-12) opt_q(13-16)
    """
    if not os.path.isfile(path):
        return []
    poses = []
    with open(path, 'r') as f:
        for line in f:
            if line.startswith('#') or not line.strip():
                continue
            v = line.split()
            if len(v) < 17:
                continue
            try:
                vals = [float(x) for x in v]
            except ValueError:
                continue
            poses.append((int(vals[0]), vals[4], vals[5], vals[6],
                          vals[13], vals[14], vals[15], vals[16]))
    return poses


def read_functional_points(index_txt):
    """读 index.txt 功能点段（'# functional points' 之后的 "<name> x y z qx qy qz qw"）。"""
    if not os.path.isfile(index_txt):
        return []
    points, in_section = [], False
    with open(index_txt, 'r') as f:
        for line in f:
            text = line.strip()
            if text.startswith('#'):
                in_section = 'functional' in text
                continue
            if not in_section or not text:
                continue
            v = text.split()
            if len(v) < 8:
                continue
            try:
                vals = [float(x) for x in v[1:8]]
            except ValueError:
                continue
            yaw = math.atan2(2.0 * (vals[6] * vals[3] + vals[4] * vals[5]),
                             1.0 - 2.0 * (vals[4] ** 2 + vals[5] ** 2))
            points.append({'name': v[0], 'x': vals[0], 'y': vals[1], 'yaw': yaw})
    return points


def boost_style_points(points_json):
    """按 boost::property_tree::write_json 的形状产出 points.json 文本。

    MapStore::read_map 用 get_child("points") + item.second.get<std::string>("pointId") 解析，
    且对 tags 用 get_child("tags") 逐子节点取值。boost 写空列表是 "points": ""，
    有值时是 [{"pointId":..,"name":..,"x":..,"y":..,"yaw":..,"tags":[{"":"tag"}]}]。
    这里严格复刻该形状，避免形状不一致触发 INVALID_MAP。
    """
    if not points_json:
        return '{\n    "points": ""\n}\n'
    items = []
    for p in points_json:
        tag_lines = ',\n'.join('                {"": %s}' % json.dumps(t) for t in p.get('tags', []))
        tag_block = '""' if not tag_lines else '[\n%s\n            ]' % tag_lines
        items.append(
            '        {\n'
            '            "pointId": %s,\n'
            '            "name": %s,\n'
            '            "x": %s,\n'
            '            "y": %s,\n'
            '            "yaw": %s,\n'
            '            "tags": %s\n'
            '        }' % (json.dumps(p['point_id']), json.dumps(p['name']),
                         repr(float(p['x'])), repr(float(p['y'])), repr(float(p['yaw'])), tag_block))
    return '{\n    "points": [\n%s\n    ]\n}\n' % ',\n'.join(items)
FRAME_ID = 'map'                      # MapStore 固定按 frame_id="map" 处理（map_frame 参数默认 map）
MAP_DATA_FILES = ('index.txt', 'keyframes.txt', 'global.pcd')


def collect_points(map_dir):
    """把 index.txt 的功能点转成 points.json 条目（pointId 必须是合法 id）。"""
    points = []
    for p in read_functional_points(os.path.join(map_dir, 'index.txt')):
        pid = sanitize_id(p['name'])
        if pid is None:
            print('WARN: 功能点名 %r 不是合法 pointId，已跳过'
                  '（MapStore::is_valid_id 要求 ^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$）' % p['name'],
                  file=sys.stderr)
            continue
        points.append({'point_id': pid, 'name': p['name'], 'x': p['x'], 'y': p['y'],
                       'yaw': p['yaw'], 'tags': []})
    return points


def posegraph_texts(map_id, name, resolution, origin, width, height, poses, source_note):
    """统一生成 <map_id>.posegraph / <map_id>.data 的内容。

    导出工具（离线建图产物）与在线桥接节点（live odom）共用同一 schema，
    保证两处产出的文件可互换、可被同一个 deserialize 消费。

    poses: 可迭代的 (id, x, y, z, qx, qy, qz, qw)
    返回 (posegraph_text, data_text)。
    """
    lines = ['# %s v%d map=%s frame=%s poses=%d source=%s'
             % (POSEGRAPH_FORMAT, POSEGRAPH_VERSION, map_id, FRAME_ID, len(poses), source_note),
             '# id x y z qx qy qz qw']
    for row in poses:
        lines.append('%d %.6f %.6f %.6f %.9f %.9f %.9f %.9f' % tuple(row))
    data_text = '\n'.join(lines) + '\n'
    manifest = {
        'format': POSEGRAPH_FORMAT,
        'version': POSEGRAPH_VERSION,
        'map_id': map_id,
        'name': name,
        'frame_id': FRAME_ID,
        'resolution': resolution,
        'origin': list(origin),
        'width': width,
        'height': height,
        'pose_count': len(poses),
        'pose_file': '%s.data' % map_id,
        'pose_file_sha256': hashlib.sha256(data_text.encode('utf-8')).hexdigest(),
        'source': source_note,
        'consumer': '/slam_toolbox/deserialize_map（navigation_manager maps.load 时下发前缀）',
        'writer': 'lidar_slam/nav_manager_map_export.py',
    }
    return json.dumps(manifest, ensure_ascii=False, indent=2) + '\n', data_text


def build_text_files(map_dir, map_id, name):
    """生成所有文本产物内容 -> dict(filename, text)。"""
    info = read_map_yaml(map_dir)
    _magic, width, height, _maxval, _off = read_pgm(os.path.join(map_dir, 'map.pgm'))
    poses = read_keyframes(os.path.join(map_dir, 'keyframes.txt'))
    if not poses:
        _die('%s/keyframes.txt 读不到位姿：MapStore 要求 <id>.posegraph/<id>.data 必须存在，'
             '我们的位姿数据就来自该文件' % map_dir)

    posegraph_text, data_text = posegraph_texts(
        map_id, name, info['resolution'],
        (info['origin_x'], info['origin_y'], info['origin_yaw']), width, height, poses,
        'keyframes.txt(opt=回环优化后) @ %s' % os.path.abspath(map_dir))

    yaml_text = ('image: %s.pgm\n'
                 'mode: trinary\n'
                 'resolution: %r\n'
                 'origin: [%r, %r, %r]\n'
                 'negate: 0\n'
                 'occupied_thresh: 0.65\n'
                 'free_thresh: 0.25\n'
                 'width: %d\n'
                 'height: %d\n'
                 % (map_id, info['resolution'], info['origin_x'], info['origin_y'],
                    info['origin_yaw'], width, height))
    return {
        '%s.yaml' % map_id: yaml_text,
        '%s.data' % map_id: data_text,
        '%s.posegraph' % map_id: posegraph_text,
        'metadata.json': json.dumps({'name': name}, ensure_ascii=False, indent=4) + '\n',
        'points.json': boost_style_points(collect_points(map_dir)),
    }


def write_bundle(map_dir, map_id, name, out_dir, with_map_data=True, dry_run=False):
    """写 <out_dir>（store 布局时为 <maps_root>/<map_id>）-> (写入文件数, 额外复制数)。

    同目录内已存在的同名文件会被覆盖；先写临时目录再整体替换，避免留下半成品
    （MapStore 用 check_regular_file 逐个校验，半成品会被判 INVALID_MAP 或直接跳过）。
    """
    texts = build_text_files(map_dir, map_id, name)
    pgm_src = os.path.join(map_dir, 'map.pgm')
    # (源文件名, 目标文件名)：目标名既要有 nav-manager 要求的 <id>.pgm，
    # 又要保留一份 map.pgm/map.yaml 原名 —— lidar_slam 定位端按 <dir>/map.yaml + map.pgm
    # （LocSystem::LoadAndPublishOccupancyGrid）与 <dir>/index.txt + 分块 pcd（TiledMap）读取，
    # 这样同一个目录既能被 navigation_manager 管理，也能直接当 system.map_path 用。
    # （MapStore 只扫描 maps_root 下的目录、只认 <id>.*，多余文件不影响。）
    pairs = [('map.pgm', '%s.pgm' % map_id)]
    if with_map_data:
        pairs.append(('map.pgm', 'map.pgm'))
        pairs.append(('map.yaml', 'map.yaml'))
        for extra in MAP_DATA_FILES:
            if os.path.isfile(os.path.join(map_dir, extra)):
                pairs.append((extra, extra))
        for extra in sorted(os.listdir(map_dir)):
            if extra.endswith('.pcd'):
                pairs.append((extra, extra))
    dedup, seen = [], set()
    for src, dst in pairs:
        if dst not in seen:
            seen.add(dst)
            dedup.append((src, dst))
    copies = dedup

    total_bytes = sum(os.path.getsize(os.path.join(map_dir, src)) for src, _dst in copies)
    print('目标目录: %s' % out_dir)
    print('  文本产物: %s' % ' '.join(sorted(texts)))
    print('  复制文件: %s（合计 %.1f MB）'
          % (' '.join(dst for _src, dst in copies), total_bytes / 1048576.0))
    if dry_run:
        print('--dry-run：未写任何文件')
        return 0, 0

    os.makedirs(os.path.dirname(os.path.abspath(out_dir)) or '.', exist_ok=True)
    staging = out_dir.rstrip('/') + '.tmp'
    if os.path.exists(staging):
        shutil.rmtree(staging)
    os.makedirs(staging)
    try:
        for fname, text in texts.items():
            with open(os.path.join(staging, fname), 'w') as f:
                f.write(text)
        for src, dst in copies:
            shutil.copyfile(os.path.join(map_dir, src), os.path.join(staging, dst))
        if os.path.exists(out_dir):
            shutil.rmtree(out_dir)
        os.replace(staging, out_dir)
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    return len(texts) + len(copies), len(copies)


def check_bundle(out_dir, map_id):
    """复刻 navigation_manager::MapStore 的判据做自检 -> [(ok, text), ...]。"""
    checks = []

    def add(ok, text):
        checks.append((bool(ok), text))

    add(ID_RE.match(map_id) is not None,
        'map_id %r 符合 ^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$' % map_id)
    if not os.path.isdir(out_dir):
        add(False, '地图目录存在（MapStore::list 扫描 maps_root 下每个目录）: %s' % out_dir)
        return checks
    add(True, '地图目录存在: %s' % out_dir)
    add(not os.path.islink(out_dir), '地图目录不是软链接（contains_symlink 会拒）')

    # ---- 1) <id>.yaml：image / resolution / origin(必须单行) ----
    yaml_path = os.path.join(out_dir, '%s.yaml' % map_id)
    if not os.path.isfile(yaml_path):
        add(False, '%s.yaml 存在' % map_id)
        return checks
    add(not os.path.islink(yaml_path), '%s.yaml 不是软链接' % map_id)
    image, resolution_text, origin_tail = '', None, ''
    with open(yaml_path, 'r') as f:
        for raw in f:
            t = raw.strip()
            if t.startswith('image:'):
                image = t.split(':', 1)[1].strip().strip('\'"')
            elif t.startswith('resolution:'):
                resolution_text = t.split(':', 1)[1].strip()
            elif t.startswith('origin:'):
                origin_tail = t.split(':', 1)[1].strip()
    add(bool(image) and not os.path.isabs(image) and os.path.normpath(image) == '%s.pgm' % map_id,
        'yaml image=%r（词法归一必须 == %r）' % (image, '%s.pgm' % map_id))
    resolution = None
    try:
        resolution = float(resolution_text)
    except (TypeError, ValueError):
        resolution = None
    add(resolution is not None and resolution > 0.0,
        'yaml resolution=%r（std::stod 可解析且 > 0）' % resolution_text)
    origin = None
    if origin_tail.startswith('[') and origin_tail.endswith(']'):
        parts = origin_tail.strip('[]').replace(',', ' ').split()
        if len(parts) == 3:
            try:
                origin = [float(p) for p in parts]
            except ValueError:
                origin = None
    add(origin is not None,
        'yaml origin 写在**同一行**且可解析为 [x, y, yaw]：%s（块序列 - x/- y/- z 会得 INVALID_MAP）'
        % (origin if origin is not None else repr(origin_tail)))

    # ---- 2) <id>.pgm ----
    pgm_path = os.path.join(out_dir, '%s.pgm' % map_id)
    ok_pgm = os.path.isfile(pgm_path) and not os.path.islink(pgm_path)
    if ok_pgm:
        try:
            magic, width, height, maxval, _off = read_pgm(pgm_path)
        except SystemExit:
            ok_pgm = False
            magic = width = height = maxval = None
    else:
        magic = width = height = maxval = None
    add(ok_pgm, '%s.pgm P5/P2 + maxval<=255（read_pgm）%s'
        % (map_id, '' if not ok_pgm else '-> %s %dx%d maxval=%d' % (magic, width, height, maxval)))

    # ---- 3) <id>.posegraph + <id>.data 必须同时存在（MAP_NOT_LOCALIZABLE 判据）----
    for suffix in ('.posegraph', '.data'):
        path = os.path.join(out_dir, '%s%s' % (map_id, suffix))
        exists = os.path.isfile(path)
        add(exists and not os.path.islink(path),
            '%s%s 存在且非软链接' % (map_id, suffix))
        if exists:
            size = os.path.getsize(path)
            add(size <= MAX_FILE_BYTES,
                '%s%s 大小 %.2f MB <= 100 MB' % (map_id, suffix, size / 1048576.0))

    # ---- 4) 可选文件 + JSON 形状（read_json 会因形状错误判 INVALID_MAP）----
    for name in ('metadata.json', 'points.json'):
        path = os.path.join(out_dir, name)
        if not os.path.isfile(path):
            add(True, '%s（可选）未生成' % name)
            continue
        try:
            with open(path, 'r') as f:
                data = json.load(f)
            extra = ''
            if name == 'points.json':
                pts = data.get('points')
                bad = [p.get('pointId', '') for p in (pts if isinstance(pts, list) else [])
                       if not ID_RE.match(str(p.get('pointId', '')))]
                extra = '，%d 个任务点%s' % (len(pts) if isinstance(pts, list) else 0,
                                           '' if not bad else '，非法 pointId: %s' % bad)
                add(not bad, 'points.json 可解析且 pointId 合法%s' % extra)
                continue
            add(True, '%s 可解析%s' % (name, extra))
        except Exception as exc:  # noqa: BLE001 - 只要解析失败就算不通过
            add(False, '%s 无法解析: %s' % (name, exc))
    return checks


def write_flat(map_dir, map_id, name, maps_root):
    """--layout flat：把 <id>.{yaml,pgm,posegraph,data} 平铺写进 maps_root，供 import_legacy_maps 导入。"""
    texts = build_text_files(map_dir, map_id, name)
    staging = os.path.join(maps_root, '.%s.tmp' % map_id)
    if os.path.exists(staging):
        shutil.rmtree(staging)
    os.makedirs(staging)
    try:
        for fname in ('%s.yaml' % map_id, '%s.posegraph' % map_id, '%s.data' % map_id):
            with open(os.path.join(staging, fname), 'w') as f:
                f.write(texts[fname])
        shutil.copyfile(os.path.join(map_dir, 'map.pgm'),
                        os.path.join(staging, '%s.pgm' % map_id))
        for fname in sorted(os.listdir(staging)):
            os.replace(os.path.join(staging, fname), os.path.join(maps_root, fname))
    finally:
        shutil.rmtree(staging, ignore_errors=True)
    return 4


def print_checks(checks, title):
    print('\n== %s ==' % title)
    failed = 0
    for ok, text in checks:
        if not ok:
            failed += 1
        print('  [%s] %s' % ('PASS' if ok else 'FAIL', text))
    return failed


def main():
    ap = argparse.ArgumentParser(
        description='lidar_slam 建图产物 -> navigation_manager(MapStore) 兼容地图包',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map-dir', default='data/new_map',
                    help='建图产物目录（含 map.pgm/map.yaml/keyframes.txt；默认 data/new_map）')
    ap.add_argument('--map-id', default=None,
                    help='地图 id（默认取目录名并合法化；须匹配 ^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$）')
    ap.add_argument('--maps-root', default='/opt/data/maps',
                    help='navigation_manager 的 maps_root（默认 /opt/data/maps）')
    ap.add_argument('--name', default=None, help='地图显示名 -> metadata.json（默认同 map-id）')
    ap.add_argument('--layout', choices=('store', 'flat'), default='store',
                    help='store=写 <maps_root>/<id>/（MapStore::list 直接识别，推荐）；'
                         'flat=写 <maps_root>/<id>.{yaml,pgm,posegraph,data}（供 import_legacy_maps 导入）')
    ap.add_argument('--with-map-data', dest='with_map_data', action='store_true', default=True,
                    help='一并拷贝 index.txt/keyframes.txt/global.pcd/分块 pcd（默认开；这样同一目录'
                         '也能直接作为 lidar_slam 定位端的 system.map_path）')
    ap.add_argument('--no-map-data', dest='with_map_data', action='store_false',
                    help='不拷贝上述建图数据（只留 MapStore 需要的文件）')
    ap.add_argument('--force', action='store_true',
                    help='目标已存在时覆盖（默认拒绝，等价于 MapStore::MAP_ALREADY_EXISTS）')
    ap.add_argument('--check-only', action='store_true',
                    help='只做兼容性自检（复刻 MapStore 判据，在临时目录里生成后校验，不动 maps_root）')
    ap.add_argument('--dry-run', action='store_true', help='只打印将要写入的文件')
    args = ap.parse_args()

    map_dir = args.map_dir
    if not os.path.isdir(map_dir):
        _die('建图目录不存在: %s' % map_dir)
    map_id = sanitize_id(args.map_id if args.map_id
                         else os.path.basename(os.path.abspath(map_dir)))
    if not map_id:
        _die('map-id 非法（须匹配 ^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$）: %r'
             % (args.map_id or os.path.basename(os.path.abspath(map_dir))))
    name = args.name or map_id
    target = os.path.join(args.maps_root, map_id) if args.layout == 'store' else args.maps_root

    print('源建图产物 : %s' % os.path.abspath(map_dir))
    print('地图 id    : %s（显示名 %s）' % (map_id, name))
    print('maps_root  : %s  布局=%s' % (args.maps_root, args.layout))

    if args.check_only:
        temp_root = tempfile.mkdtemp(prefix='navmap_selfcheck_')
        try:
            if args.layout == 'store':
                out = os.path.join(temp_root, map_id)
                write_bundle(map_dir, map_id, name, out, with_map_data=False)
            else:
                out = temp_root          # flat：4 个文件按 <id>.* 平铺在目录里，判据一致
                write_flat(map_dir, map_id, name, temp_root)
            failed = print_checks(check_bundle(out, map_id), '兼容性自检（复刻 MapStore 判据）')
        finally:
            shutil.rmtree(temp_root, ignore_errors=True)
        print('\n结论: %s' % ('全部判据通过，该地图可被 navigation_manager 管理' if not failed
                             else '%d 项不通过' % failed))
        return 1 if failed else 0

    if os.path.exists(target if args.layout == 'store' else
                      os.path.join(args.maps_root, '%s.yaml' % map_id)):
        if not args.force:
            _die('目标已存在（MapStore 语义 = MAP_ALREADY_EXISTS，用 --force 覆盖）: %s' % target)
        print('NOTE: --force：覆盖已存在的目标 %s' % target)

    os.makedirs(args.maps_root, exist_ok=True)
    if args.layout == 'flat':
        if args.dry_run:
            print('目标目录: %s' % args.maps_root)
            print('  文本产物: %s.data %s.posegraph %s.yaml' % (map_id, map_id, map_id))
            print('  复制文件: %s.pgm' % map_id)
            print('--dry-run：未写任何文件')
            return 0
        written = write_flat(map_dir, map_id, name, args.maps_root)
        copied = 0
        out = args.maps_root
        print('NOTE: flat 布局只写 MapStore 需要的 4 个文件；import_legacy_maps 也只导入这 4 个，'
              '若还要给 lidar_slam 定位端当 system.map_path，请把 index.txt/分块 pcd 等拷到 '
              '%s/%s/ 下（或改用 --layout store 一步到位）' % (args.maps_root, map_id))
    else:
        written, copied = write_bundle(map_dir, map_id, name, target,
                                       with_map_data=args.with_map_data, dry_run=args.dry_run)
        if args.dry_run:
            return 0
        out = target

    print('\n已写入: %s（%d 个文件%s）' % (out, written,
                                        '' if args.layout == 'flat' else '，其中复制 %d 个建图数据' % copied))
    failed = print_checks(check_bundle(out, map_id), '写后自检（复刻 MapStore 判据）')
    print('\n结论: %s' % ('OK，可被 navigation_manager 识别' if not failed else '%d 项不通过' % failed))
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
