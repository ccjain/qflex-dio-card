@echo off
call "%~dp0env.bat"
make -j %*
