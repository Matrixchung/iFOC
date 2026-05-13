#!/usr/bin/env python3
"""
generate_nanopb.py
==================
从 proto3 文件自动生成 C/C++ 头文件：

  【message proto】生成三个文件：
    {name}.pb.h   — nanopb C 结构体 + FIELDLIST 宏
    {name}.pb.c   — PB_BIND 描述符（被 CMake GLOB 自动编译）
    {name}.h      — C++ 包装类（兼容 EmbeddedProto API: get_xxx/set_xxx/xxx()）

  【enum-only proto】生成一个文件：
    {name}.h      — C++ enum class : uint32_t（命名空间与原 EmbeddedProto 生成文件完全相同）

用法（在 Proto/ 目录下执行）：
  python generate_nanopb.py Base/motor_state.proto
  python generate_nanopb.py Config/Motor/foc_motor_config.proto
  python generate_nanopb.py --all          # 处理所有 .proto 文件

输出目录：Proto/ 下的子目录结构镜像到 Headers/ 下。
  例：Proto/Base/motor_state.proto      → Headers/Base/motor_state.h
      Proto/Config/Motor/foo.proto      → Headers/Config/Motor/foo.pb.h  foo.pb.c  foo.h

支持的字段类型（message proto）：
  标量：float, double, uint32, int32, uint64, int64, bool,
         sint32, sint64, fixed32, sfixed32, fixed64, sfixed64
  枚举：任意非标量类型（带或不带包名前缀），存为 int32_t，生成 typed accessor
  不支持：repeated, map, nested message, bytes, string（遇到时跳过并警告）

注意：
  • 包装 .h 若已存在则跳过（保护手动修改），删掉再跑才会重新生成。
  • enum .h 同理。
  • 新增 .pb.c 后需重新执行 cmake configure。
"""

import os
import re
import sys
from pathlib import Path

# ─────────────────────────────────────────────────────────────────────────────
# proto 标量类型 → (C 类型, nanopb LTYPE, 是否进入 REFLECT)
# ─────────────────────────────────────────────────────────────────────────────
SCALAR_MAP = {
    "float":    ("float",    "FLOAT",   True),
    "double":   ("double",   "FLOAT64", True),
    "uint32":   ("uint32_t", "UINT32",  True),
    "int32":    ("int32_t",  "INT32",   True),
    "uint64":   ("uint64_t", "UINT64",  True),
    "int64":    ("int64_t",  "INT64",   True),
    "bool":     ("bool",     "BOOL",    True),
    "sint32":   ("int32_t",  "SVARINT", True),
    "sint64":   ("int64_t",  "SVARINT", True),
    "fixed32":  ("uint32_t", "FIXED32", True),
    "sfixed32": ("int32_t",  "FIXED32", True),
    "fixed64":  ("uint64_t", "FIXED64", True),
    "sfixed64": ("int64_t",  "FIXED64", True),
}

_COMMENT = re.compile(r'//.*?$|/\*.*?\*/', re.MULTILINE | re.DOTALL)

def strip_comments(src: str) -> str:
    return _COMMENT.sub('', src)

# ─────────────────────────────────────────────────────────────────────────────
# 命名辅助
# ─────────────────────────────────────────────────────────────────────────────
def to_screaming_snake(name: str) -> str:
    """CamelCase → SCREAMING_SNAKE_CASE。
    UARTBaudrate → UART_BAUDRATE, MotorControlMode → MOTOR_CONTROL_MODE
    """
    s = re.sub(r'([A-Z]+)([A-Z][a-z])', r'\1_\2', name)
    s = re.sub(r'([a-z])([A-Z])',        r'\1_\2', s)
    return s.upper()

def pkg_to_ns_parts(package: str) -> list:
    return package.split('.') if package else []

# ─────────────────────────────────────────────────────────────────────────────
# 解析 proto 文件
# ─────────────────────────────────────────────────────────────────────────────
def parse_proto(path: Path) -> dict:
    """返回 {package, imports, enums, message} 字典。

    enums    : [{name, values: [(name, int)]}]   — 文件内所有 enum 块
    message  : None  或  {name, fields: [...]}   — 最多一个 message 块
      fields 每项: {tag, name, proto_type, c_type, nb_ltype,
                    is_enum, enum_cpp_type, reflected, getter_ret,
                    is_string, str_max_size}   — 后两项仅 string 字段有效
    """
    src_raw = path.read_text(encoding='utf-8')

    # 在去注释前，提取 string 字段的 @max_size=N 注解
    max_sizes: dict = {}
    for m in re.finditer(
            r'\bstring\s+(\w+)\s*=\s*\d+\s*;[^\n]*//[^\n]*@max_size\s*=\s*(\d+)',
            src_raw):
        max_sizes[m.group(1)] = int(m.group(2))

    src = strip_comments(src_raw)

    # package / imports
    pkg_m   = re.search(r'\bpackage\s+([\w.]+)\s*;', src)
    package = pkg_m.group(1) if pkg_m else ''
    imports = re.findall(r'\bimport\s+"([^"]+)"', src)

    # ── enums ──
    enum_blocks = re.findall(r'\benum\s+(\w+)\s*\{([^}]*)\}', src, re.DOTALL)
    enums = []
    for ename, ebody in enum_blocks:
        vals = []
        for m in re.finditer(r'\b(\w+)\s*=\s*(-?\d+)\s*;', ebody):
            vals.append((m.group(1), int(m.group(2))))
        enums.append({'name': ename, 'values': vals})

    # ── message (first one only) ──
    msg_m = re.search(r'\bmessage\s+(\w+)\s*\{(.*?)\}', src, re.DOTALL)
    message = None
    if msg_m:
        mname = msg_m.group(1)
        body  = msg_m.group(2)
        field_pat = re.compile(
            r'\b(?:optional\s+|repeated\s+|required\s+)?'
            r'([\w.]+)\s+(\w+)\s*=\s*(\d+)\s*;'
        )
        fields = []
        for m in field_pat.finditer(body):
            ptype, fname, tag_s = m.group(1), m.group(2), m.group(3)
            tag = int(tag_s)
            if ptype == 'bytes':
                print(f"  [SKIP] field '{fname}': type 'bytes' not supported")
                continue
            if ptype == 'string':
                if fname not in max_sizes:
                    print(f"  [SKIP] field '{fname}': string field requires // @max_size=N comment")
                    continue
                sz = max_sizes[fname]
                fields.append(dict(tag=tag, name=fname, proto_type='string',
                                   c_type='char', nb_ltype='STRING',
                                   is_enum=False, enum_cpp_type=None,
                                   reflected=False, getter_ret='const char*',
                                   is_string=True, str_max_size=sz))
                continue
            if ptype in SCALAR_MAP:
                c_type, nb_ltype, reflected = SCALAR_MAP[ptype]
                is_enum, enum_cpp_type, getter_ret = False, None, c_type
            else:
                c_type, nb_ltype, reflected = 'int32_t', 'INT32', False
                is_enum       = True
                enum_cpp_type = ptype.replace('.', '::')
                getter_ret    = enum_cpp_type
            fields.append(dict(tag=tag, name=fname, proto_type=ptype,
                               c_type=c_type, nb_ltype=nb_ltype,
                               is_enum=is_enum, enum_cpp_type=enum_cpp_type,
                               reflected=reflected, getter_ret=getter_ret,
                               is_string=False, str_max_size=0))
        fields.sort(key=lambda f: f['tag'])
        message = {'name': mname, 'fields': fields}

    return dict(package=package, imports=imports, enums=enums, message=message)

# ─────────────────────────────────────────────────────────────────────────────
# 生成：enum-only .h
# ─────────────────────────────────────────────────────────────────────────────
def gen_enum_h(package: str, enums: list, rel: Path) -> str:
    """生成与原 EmbeddedProto 输出格式完全相同的 C++ enum class 头文件。"""
    ns_parts = pkg_to_ns_parts(package)

    # include guard：{LAST_NS}_{ENUM_SNAKE}_H
    # 若一个文件含多个 enum，用第一个 enum 的名字。
    last_ns  = ns_parts[-1].upper() if ns_parts else ''
    guard    = f'{last_ns}_{to_screaming_snake(enums[0]["name"])}_H'

    lines = [
        f'/* auto-generated by generate_nanopb.py — source: {rel.as_posix()} */',
        f'#ifndef {guard}',
        f'#define {guard}',
        '',
        '#include <cstdint>',
        '// Include external proto definitions',
        '',
    ]

    # 开命名空间（旧式嵌套风格，与 EmbeddedProto 生成文件一致）
    for p in ns_parts:
        lines.append(f'namespace {p} {{')
    lines.append('')

    for enum in enums:
        lines.append(f'enum class {enum["name"]} : uint32_t')
        lines.append('{')
        vals = enum['values']
        for i, (vname, vval) in enumerate(vals):
            comma = ',' if i < len(vals) - 1 else ''
            lines.append(f'  {vname} = {vval}{comma}')
        lines.append('};')
        lines.append('')

    # 关命名空间
    for p in reversed(ns_parts):
        lines.append(f'}} // End of namespace {p}')
    lines.append(f'#endif // {guard}')
    lines.append('')

    return '\n'.join(lines)

# ─────────────────────────────────────────────────────────────────────────────
# 生成：message .pb.h
# ─────────────────────────────────────────────────────────────────────────────
def gen_pb_h(msg: str, fields: list) -> str:
    struct_name = msg + '_data'
    need_bool   = any(f['c_type'] == 'bool' for f in fields)

    lines = [
        f'/* nanopb descriptor for {msg} — auto-generated by generate_nanopb.py */',
        f'#ifndef {to_screaming_snake(msg)}_PB_H',
        f'#define {to_screaming_snake(msg)}_PB_H',
        '',
        '#include <pb.h>',
        '#include <stdint.h>',
    ]
    if need_bool:
        lines.append('#include <stdbool.h>')
    lines += ['', '#ifdef __cplusplus', 'extern "C" {', '#endif', '']

    # C struct
    max_ct = max((len(f['c_type']) for f in fields), default=8)
    lines.append('typedef struct {')
    for f in fields:
        pad = ' ' * (max_ct - len(f['c_type']) + 1)
        if f['is_string']:
            # char name[max_size+1]
            comment = f'  /* max {f["str_max_size"]} chars + null, STRING field */'
            lines.append(f'    {f["c_type"]}{pad}{f["name"]}[{f["str_max_size"] + 1}];{comment}')
        else:
            comment = f'  /* enum {f["proto_type"]} */' if f['is_enum'] else ''
            lines.append(f'    {f["c_type"]}{pad}{f["name"]};{comment}')
    lines.append(f'}} {struct_name};')
    lines.append('')

    # FIELDLIST macro
    max_fn = max(len(f['name']) for f in fields) if fields else 8
    lines.append(f'#define {msg}_FIELDLIST(X, a) \\')
    for i, f in enumerate(fields):
        trail  = '' if i == len(fields) - 1 else ' \\'
        fname  = f['name'] + ',' + ' ' * (max_fn - len(f['name']))
        lines.append(f'X(a, STATIC, SINGULAR, {f["nb_ltype"]}, {fname}{f["tag"]:>4}){trail}')
    lines.append('')

    lines += [
        f'#define {msg}_CALLBACK NULL',
        f'#define {msg}_DEFAULT  NULL',
        '',
        f'extern const pb_msgdesc_t {struct_name}_msg;',
        f'#define {struct_name}_fields (&{struct_name}_msg)',
        '',
        '#ifdef __cplusplus',
        '}',
        '#endif',
        '',
        f'#endif /* {to_screaming_snake(msg)}_PB_H */',
        '',
    ]
    return '\n'.join(lines)

# ─────────────────────────────────────────────────────────────────────────────
# 生成：message .pb.c
# ─────────────────────────────────────────────────────────────────────────────
def gen_pb_c(pb_h_name: str, msg: str) -> str:
    return (
        f'/* auto-generated by generate_nanopb.py */\n'
        f'#include "{pb_h_name}"\n'
        f'PB_BIND({msg}, {msg}_data, 2)\n'
    )

# ─────────────────────────────────────────────────────────────────────────────
# 生成：message C++ 包装 .h
# ─────────────────────────────────────────────────────────────────────────────
def _refl_path(out_dir: Path, proto_root: Path) -> str:
    datatype_dir = proto_root.parent
    rel = os.path.relpath(datatype_dir, out_dir)
    return rel.replace('\\', '/') + '/reflection.h'

def gen_wrapper_h(package: str, imports: list, msg: str, fields: list,
                  stem: str, out_dir: Path, proto_root: Path) -> str:
    struct_name = msg + '_data'
    ns_parts    = pkg_to_ns_parts(package)
    ns_open     = '\n'.join(f'namespace {p} {{' for p in ns_parts)
    ns_close    = '\n'.join(f'}} // namespace {p}' for p in reversed(ns_parts))

    lines = [
        '#pragma once',
        '',
        '#ifdef __GNUC__',
        '#pragma GCC diagnostic push',
        '#pragma GCC diagnostic ignored "-Winvalid-offsetof"',
        '#endif',
        '',
        f'#include "{stem}.pb.h"',
        f'#include "{_refl_path(out_dir, proto_root)}"',
    ]
    for imp in imports:
        lines.append(f'#include "{Path(imp).stem}.h"')
    lines += ['']

    if ns_open:
        lines += [ns_open, '']

    reflect_fields = [f for f in fields if f['reflected']]
    max_fn = max((len(f['name']) for f in fields), default=16)

    lines += [
        f'class {msg}',
        '{',
        'public:',
        f'    {struct_name} _d{{}};   /* MUST be first data member for reflection */',
        '',
        f'    {msg}() = default;',
    ]

    if reflect_fields:
        lines += ['', '    REFLECT(']
        for i, f in enumerate(reflect_fields):
            comma = '' if i == len(reflect_fields) - 1 else ','
            lines.append(f'        MEMBER_SIZE_OFFSET({struct_name}, {f["name"]}){comma}')
        lines.append('    )')

    lines += [
        '',
        f'    static const pb_msgdesc_t* pb_fields() {{ return {struct_name}_fields; }}',
        f'    const void* pb_data() const             {{ return &_d; }}',
        f'    void*       pb_data()                   {{ return &_d; }}',
        f'    void        clear()                     {{ memset(&_d, 0, sizeof(_d)); }}',
        '',
        '    /* ── Inline getters / setters (zero code-size overhead) ── */',
    ]

    for f in fields:
        n, ret, ct = f['name'], f['getter_ret'], f['c_type']
        pad = ' ' * max(1, max_fn - len(n) + 1)
        if f['is_string']:
            sz = f['str_max_size']
            lines += [
                f'    /* {n}: STRING, max {sz} chars — not in REFLECT */',
                f'    const char* {n}(){pad}       const {{ return _d.{n}; }}',
                f'    const char* get_{n}(){pad}    const {{ return _d.{n}; }}',
                f'    void set_{n}(const char* v){pad}     {{ strncpy(_d.{n}, v, {sz}); _d.{n}[{sz}] = \'\\0\'; }}',
            ]
        elif f['is_enum']:
            lines += [
                f'    {ret} {n}(){pad}        const {{ return static_cast<{ret}>(_d.{n}); }}',
                f'    {ret} get_{n}(){pad}     const {{ return static_cast<{ret}>(_d.{n}); }}',
                f'    void set_{n}({ret} v){pad}       {{ _d.{n} = static_cast<{ct}>(v); }}',
            ]
        else:
            lines += [
                f'    {ret} {n}(){pad}        const {{ return _d.{n}; }}',
                f'    {ret} get_{n}(){pad}     const {{ return _d.{n}; }}',
                f'    void set_{n}({ret} v){pad}       {{ _d.{n} = v; }}',
            ]

    lines += ['};', '']
    if ns_close:
        lines += [ns_close, '']
    lines += ['#ifdef __GNUC__', '#pragma GCC diagnostic pop', '#endif', '']
    return '\n'.join(lines)

# ─────────────────────────────────────────────────────────────────────────────
# 主流程：处理单个 .proto
# ─────────────────────────────────────────────────────────────────────────────
def process_proto(proto_path: Path, proto_root: Path):
    rel     = proto_path.relative_to(proto_root)
    out_dir = proto_root.parent / 'Headers' / rel.parent
    out_dir.mkdir(parents=True, exist_ok=True)

    stem = proto_path.stem
    print(f'\n[{rel}]')

    info = parse_proto(proto_path)
    pkg, imports, enums, message = (
        info['package'], info['imports'], info['enums'], info['message'])

    # ── enum-only proto ─────────────────────────────────────────────────────
    if message is None:
        if not enums:
            print('  [SKIP] no message or enum found')
            return
        h_path = out_dir / (stem + '.h')
        if h_path.exists():
            print(f'  [SKIP] {stem}.h already exists — delete to regenerate')
        else:
            h_path.write_text(gen_enum_h(pkg, enums, rel), encoding='utf-8')
            print(f'  → {h_path.relative_to(proto_root.parent)}')
        return

    # ── message proto ────────────────────────────────────────────────────────
    msg    = message['name']
    fields = message['fields']

    # .pb.h — always regenerate (pure descriptor, safe to overwrite)
    pb_h_path = out_dir / (stem + '.pb.h')
    pb_h_path.write_text(gen_pb_h(msg, fields), encoding='utf-8')
    print(f'  → {pb_h_path.relative_to(proto_root.parent)}')

    # .pb.c — always regenerate
    pb_c_path = out_dir / (stem + '.pb.c')
    pb_c_path.write_text(gen_pb_c(stem + '.pb.h', msg), encoding='utf-8')
    print(f'  → {pb_c_path.relative_to(proto_root.parent)}')

    # .h wrapper — skip if exists (may contain manual edits)
    w_path = out_dir / (stem + '.h')
    if w_path.exists():
        print(f'  [SKIP] {stem}.h already exists — delete to regenerate')
    else:
        w_path.write_text(
            gen_wrapper_h(pkg, imports, msg, fields, stem, out_dir, proto_root),
            encoding='utf-8')
        print(f'  → {w_path.relative_to(proto_root.parent)}')

# ─────────────────────────────────────────────────────────────────────────────
# 入口
# ─────────────────────────────────────────────────────────────────────────────
def main():
    script_dir = Path(__file__).parent.resolve()   # Proto/ 目录

    if len(sys.argv) < 2 or sys.argv[1] in ('-h', '--help'):
        print(__doc__)
        sys.exit(0)

    if sys.argv[1] == '--all':
        protos = sorted(script_dir.rglob('*.proto'))
        if not protos:
            print('No .proto files found.')
            sys.exit(0)
        for p in protos:
            process_proto(p, script_dir)
    else:
        proto_path = Path(sys.argv[1])
        if not proto_path.is_absolute():
            proto_path = script_dir / proto_path
        process_proto(proto_path.resolve(), script_dir)

    print('\nDone. Remember to re-run cmake if new .pb.c files were created.')

if __name__ == '__main__':
    main()
