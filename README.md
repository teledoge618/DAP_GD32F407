
### 工具链

建议使用MSYS2编译环境

- 安装msys2
- pacman -Sy
- pacman -Su
- pacman -S mingw-w64-ucrt-x86_64-arm-none-eabi-gcc
- pacman -S mingw-w64-ucrt-x86_64-cmake
- pacman -S mingw-w64-ucrt-x86_64-ninja

## 编译

建议使用VS Code

- 安装CMake和CMakeTools
- CMake > 配置 > Release
- CMake > 生成
- 找到./build/source/app/app.hex
- 使用pyocd或opencod烧录
