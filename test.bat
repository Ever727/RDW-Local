@echo off
setlocal enabledelayedexpansion
chcp 65001 > nul

REM 配置全局变量
set python_path=C:\Users\qq642\AppData\Local\Programs\Python\Python311\python.exe
set times=1
set environment=2
set nums=25
set algorithms=ARC APF S2C S2O

REM 遍历 algorithms
for %%a in (%algorithms%) do (
    set algorithm=%%a
    echo 正在测试算法 !algorithm!
    set times=1
    :inner_loop
    if !times! leq %nums% (
        echo ========== 开始第 !times! 次执行 (算法: !algorithm! ) ==========

        REM 检查端口占用并清理
        for /f "tokens=5" %%i in ('netstat -ano ^| findstr :8000') do taskkill /PID %%i /F >nul 2>&1
        for /f "tokens=5" %%i in ('netstat -ano ^| findstr :8765') do taskkill /PID %%i /F >nul 2>&1

        REM 启动 client_base.py
        set log_file=logs/env_%environment%/!times!_!algorithm!.txt
        if not exist logs\env_%environment% mkdir logs\env_%environment%
        start /B /D "%~dp0" "%python_path%" client_base.py -c !algorithm! > !log_file!
        set client_pid=!ERRORLEVEL!

        REM 启动 uvicorn
        pushd ..\EasyRDW-Web
        start /B uvicorn main:app --host 0.0.0.0 --port 8000 > nul 2>&1
        set uvicorn_pid=!ERRORLEVEL!
        popd

        REM 启动 firefox.py
        start /B /D "%~dp0" "%python_path%" firefox.py test_set/env_%environment%/!times!.json
        set python_pid=!ERRORLEVEL!

        REM 检查日志文件是否包含 type=end
        echo 正在等待日志文件中 'type=end' 的出现...
        REM 读取最后一行并检查 JSON 的 'type' 字段
        :check_loop
        for /f "tokens=*" %%i in ('powershell -Command "Get-Content -Tail 2 -Path ''%log_file%''"') do (
            echo %%i | findstr /C:"'type': 'end'" >nul
            if not errorlevel 1 (
                echo 检测到 'type=end'，继续执行下一步...
                goto cleanup
            )
        )
        timeout /t 1 >nul
        goto check_loop

        :cleanup
        echo Cleaning up background processes...
        taskkill /PID !client_pid! /F >nul 2>&1
        taskkill /PID !uvicorn_pid! /F >nul 2>&1
        taskkill /PID !python_pid! /F >nul 2>&1
        echo Finish cleaning up

        REM 增加 times
        set /A times+=1

        echo ========== 第 !times! 次执行完成 (算法: !algorithm!) ==========
        goto inner_loop
    )
)
echo 所有算法和任务执行完成！
exit /B