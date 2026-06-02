@echo off
call "%~dp0env.bat"
STM32_Programmer_CLI -c port=SWD freq=4000 -d build\dio_card.bin 0x08000000 -rst
