import os
import re

REFLECTION_HEADER = '"reflection.h"'

def process_header_file(file_path):
    with open(file_path, 'r') as f:
        content = f.read()
    
    # 检查是否已经包含reflection.h
    if f'#include {REFLECTION_HEADER}' in content:
        print(f"Skipping {file_path} - already modified")
        return
    
    # 查找类定义
    class_pattern = re.compile(
        r'class (\w+) final: public \:\:EmbeddedProto\:\:MessageInterface\n\s*{([^}]*)}',
        re.DOTALL
    )
    
    class_match = class_pattern.search(content)
    if not class_match:
        return
    
    class_name = class_match.group(1)
    
    # 查找private部分
    private_section_pattern = re.compile(
        r'private:\s*((?:[^;{}]*;)+)',
        re.DOTALL
    )
    
    private_match = private_section_pattern.search(content)
    if not private_match:
        print(f"No private section found in {file_path}, skipping")
        return
    
    # 提取private成员变量
    member_var_pattern = re.compile(
        r'\bEmbeddedProto::\w+(?:<\w+>)?\s+(\w+)_\s*(?:=\s*[^;]+)?;',
        re.DOTALL
    )
    
    members = set()
    for match in member_var_pattern.finditer(private_match.group(1)):
        var_name = match.group(1)
        print(f"  - {var_name}_")
        members.add(match.group(1))
    
    if not members:
        print(f"No member variables found in private section of {file_path}, skipping")
        return
    
    # 添加reflection.h包含
    include_pattern = re.compile(r'#include <[^>]+>\n#include <[^>]+>')
    content = include_pattern.sub(
        lambda m: m.group(0) + f'\n#include {REFLECTION_HEADER}',
        content,
        1
    )
    
    # 生成REFLECT宏
    reflect_macro = "    REFLECT(\n"
    reflect_macro += ",\n".join([f"        MEMBER_SIZE_OFFSET({class_name}, {m}_)" for m in members])
    reflect_macro += "\n    )\n"
    
    # 插入REFLECT宏到public部分
    public_section_pattern = re.compile(
        r'public:\s*',
        re.DOTALL
    )
    
    content = public_section_pattern.sub(
        lambda m: m.group(0) + reflect_macro,
        content,
        1
    )
    
    # 写回原文件
    with open(file_path, 'w') as f:
        f.write(content)
    print(f"Modified {file_path}")

def process_proto_headers(directory):
    for root, _, files in os.walk(directory):
        for file in files:
            if file.endswith('.h'):
                process_header_file(os.path.join(root, file))

if __name__ == '__main__':
    import platform
    if platform.system() == "Windows":
        src_directory = '.\\'
        output_directory = r'..\Headers'
    else:
        src_directory = "./"
        output_directory = "../Headers/"
    process_proto_headers(output_directory)