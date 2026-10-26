@echo off
title PiSim Studio - Raspberry Pi 5 Web Dashboard GCS
cd /d "%~dp0"

echo ========================================================
echo     PiSim Studio - Web Dashboard Launcher (Windows)     
echo ========================================================

python pisim_web_dashboard.py %*
if %ERRORLEVEL% neq 0 (
    echo.
    echo [-] Python calistirma hatasi olustu.
    pause
)
