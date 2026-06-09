@echo off
chcp 65001 >nul
echo ========================================
echo AM32 Windows 编译环境配置
echo ========================================
echo.

:: 检查是否已安装 winget
where winget >nul 2>&1
if %errorlevel% neq 0 (
    echo [错误] 未找到 winget，请先安装 Windows App Installer
    echo 下载地址: https://aka.ms/getwinget
    pause
    exit /b 1
)

:: 1. 安装 ARM GCC 工具链
echo [1/3] 检查 ARM GCC 工具链...
where arm-none-eabi-gcc >nul 2>&1
if %errorlevel% neq 0 (
    echo 安装 ARM GCC 工具链...
    winget install -e --id ARM.GNUToolsArmEmbedded -h --accept-source-agreements --accept-package-agreements
) else (
    echo ARM GCC 已安装
)

:: 2. 安装 GNU Make
echo [2/3] 检查 GNU Make...
where make >nul 2>&1
if %errorlevel% neq 0 (
    echo 安装 GNU Make...
    winget install -e --id GnuWin32.Make -h --accept-source-agreements --accept-package-agreements
    :: 添加到 PATH
    setx PATH "%PATH%;C:\Program Files (x86)\GnuWin32\bin" >nul 2>&1
) else (
    echo GNU Make 已安装
)

:: 3. 配置 VS Code
echo [3/3] 配置 VS Code...
cd /d "%~dp0"

:: 复制 Windows 配置文件
if exist ".vscode\settings.json.windows" (
    copy /Y ".vscode\settings.json.windows" ".vscode\settings.json" >nul
    echo VS Code 配置已更新
) else (
    echo [警告] 未找到 settings.json.windows
)

echo.
echo ========================================
echo 配置完成！
echo ========================================
echo.
echo 请注意：
echo 1. 如果之前已安装过工具，请重启终端
echo 2. 编译命令：make f421
echo.
pause
