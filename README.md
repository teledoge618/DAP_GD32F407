# 简介

- GD32F407VET6，160MHz。
- 本项目使用硬件SPI模拟SWD时序，实现了CMSIS-DAP的SWD基本功能。
- 使用定时器比较输出作为SPI的时钟信号，输出频率准确，分频调整范围大。
- 使用定时器搭配三态门控制时钟信号输出，利用8bit的SPI外设输出任意整数个时钟周期。

效果

- 时钟可配置为系统时钟的4到65536的整数分频，也可以使用预分频器达到更低频率。
- 在10M固定频率下使用pyocd烧录500K固件到GD32F407VE的FLASH可达到约160K/s的速度，远低于野火高速DAP的约250K/s。
- 野火DAP速度优势可能来自于HID驱动下的1024字节数据包，本项目使用的WinUSB驱动数据包只有512字节。
- 利用M4的位带操作，把SPI操作间隔从2.7us降低到1.5us，烧录速度从120K/s提高到到160K/s。

问题

- 操作定时器和SPI耗费时间较长，每字节SPI操作间隔约1.5us，严重拖累时钟速度。
- 提升频率后，烧录速度提升比较小，20M频率也才210K/s左右。

# 工具链

建议使用MSYS2编译环境

- 安装msys2
- pacman -Sy
- pacman -Su
- pacman -S mingw-w64-ucrt-x86_64-arm-none-eabi-gcc
- pacman -S mingw-w64-ucrt-x86_64-cmake
- pacman -S mingw-w64-ucrt-x86_64-ninja

# 编译

建议使用VS Code

- 安装CMake和CMakeTools
- CMake > 配置 > Release
- CMake > 生成
- 找到./build/source/app/app.hex
- 使用pyocd或openocd下载到芯片

# 库引用

- CMSIS_5 https://github.com/ARM-software/CMSIS_5
- CherryUSB https://github.com/cherry-embedded/CherryUSB
- printf https://github.com/mpaland/printf
- GD32_LIB
