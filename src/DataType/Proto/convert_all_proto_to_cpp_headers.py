import os
import subprocess
import shutil
import platform

def find_protoc() -> str:
    protoc_path = shutil.which('protoc')
    if protoc_path:
        return protoc_path
    common_paths = {
        'Windows': [r'C:\protoc\bin\protoc.exe'],
        'Linux': ['/usr/local/bin/protoc'],
        'Darwin': ['/usr/local/bin/protoc']
    }
    for path in common_paths.get(platform.system(), []):
        if os.path.exists(path):
            return path
    return None

def remove_root_path_headers(output_dir):
    target_dir = os.path.abspath(output_dir)
    if not os.path.exists(target_dir): return
    for filename in os.listdir(target_dir):
        file_path = os.path.join(target_dir, filename)
        if os.path.isfile(file_path) and filename.lower().endswith('.h'):
            try:
                os.remove(file_path)
            except Exception as e:
                print(f"Error in deleting headers {filename}: {str(e)}")

def process_proto_files(src_dir, output_dir):
    protoc = find_protoc()
    if not protoc:
        raise Exception("Protoc compiler not found. Install it from https://github.com/protocolbuffers/protobuf/releases")
    
    plugin_str = "--plugin=protoc-gen-eams=..\\..\\ThirdParty\\EmbeddedProto\\protoc-gen-eams.bat"
    if platform.system() != "Windows":
        plugin_str = "--plugin=protoc-gen-eams=../../ThirdParty/EmbeddedProto/protoc-gen-eams"

    for root, _, files in os.walk(src_dir):
        for filename in files:
            if not filename.endswith('.proto'):
                continue
            src_path = os.path.join(root, filename)
            rel_path = os.path.relpath(root, src_dir)
            target_dir = os.path.join(output_dir, rel_path)
            os.makedirs(target_dir, exist_ok=True)
            cmd = [
                protoc,
                plugin_str,
                f"--proto_path={src_dir}", 
                f"--proto_path={os.path.abspath(root)}",
                f"--eams_out={output_dir}",
                src_path
            ]
            print(f"Compiling {src_path}...")
            result = subprocess.run(cmd, capture_output=True, text=True)
            if result.returncode != 0:
                print(f"Error compiling {src_path}:")
                print(result.stderr)
                continue
    
    remove_root_path_headers(output_dir)

    print(f"All .proto files processed. Output directory: {output_dir}")

if __name__ == "__main__":
    if platform.system() == "Windows":
        src_directory = '.\\'
        output_directory = r'..\Headers'
    else:
        src_directory = "./"
        output_directory = "../Headers/"
    process_proto_files(src_directory, output_directory)