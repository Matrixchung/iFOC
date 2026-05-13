@echo off
cd %~dp0
python generate_nanopb.py --all
pause
exit
