@echo off
cd  %~dp0
python convert_all_proto_to_cpp_headers.py
python add_reflection_to_headers.py
pause
exit