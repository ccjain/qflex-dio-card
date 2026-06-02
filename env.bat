@echo off
set "CUBEIDE_DIR=C:\ST\STM32CubeIDE_1.19.0\STM32CubeIDE\plugins"
set "GCC_DIR=%CUBEIDE_DIR%\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344\tools\bin"
set "MAKE_DIR=%CUBEIDE_DIR%\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.0.202409170845\tools\bin"
set "PROG_DIR=%CUBEIDE_DIR%\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.200.202503041107\tools\bin"
set "PATH=%GCC_DIR%;%MAKE_DIR%;%PROG_DIR%;%PATH%"
