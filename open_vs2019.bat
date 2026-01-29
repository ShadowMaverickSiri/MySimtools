@echo off
REM ============================================================
REM SimTools v2.0 - Visual Studio 2019 快速启动脚本
REM ============================================================

echo ========================================
echo   SimTools v2.0 - VS2019 快速启动
echo ========================================
echo.

REM 设置项目路径
set PROJECT_DIR=%~dp0
set BUILD_DIR=%PROJECT_DIR%build
set SOLUTION_FILE=%BUILD_DIR%\SimTools.sln

REM 检查解决方案文件是否存在
if not exist "%SOLUTION_FILE%" (
    echo [1/3] 生成 Visual Studio 项目...
    echo.

    REM 创建 build 目录
    if not exist "%BUILD_DIR%" (
        mkdir "%BUILD_DIR%"
    )

    REM 生成 VS2019 项目
    cd /d "%BUILD_DIR%"
    cmake .. -G "Visual Studio 16 2019" -A x64

    echo.
    echo [1/3] 项目生成完成！
    echo.
) else (
    echo [1/3] 项目文件已存在，跳过生成
    echo.
)

REM 检查解决方案文件是否真的存在
if not exist "%SOLUTION_FILE%" (
    echo 错误：无法找到解决方案文件
    echo 路径：%SOLUTION_FILE%
    pause
    exit /b 1
)

echo [2/3] 打开 Visual Studio 2019...
echo.
echo 解决方案：%SOLUTION_FILE%
echo.

REM 打开解决方案
start "" "%SOLUTION_FILE%"

echo [2/3] Visual Studio 2019 已启动
echo.
echo [3/3] 在 VS2019 中，请按以下步骤操作：
echo.
echo ========================================
echo   在 Visual Studio 2019 中：
echo ========================================
echo.
echo 1. 在顶部工具栏选择：
echo    - 配置：Release 或 Debug
echo    - 平台：x64
echo.
echo 2. 设置启动项目：
echo    - 右键点击 "SimTools_test" 或 "SimTools_example"
echo    - 选择 "设为启动项目"
echo.
echo 3. 编译：
echo    - 按 Ctrl+Shift+B
echo    - 或点击 "生成 → 生成解决方案"
echo.
echo 4. 运行：
echo    - 按 F5（调试）
echo    - 或 Ctrl+F5（不调试）
echo.
echo ========================================
echo.

echo 按任意键关闭此窗口...
pause >nul
