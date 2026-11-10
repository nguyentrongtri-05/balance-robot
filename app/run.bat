@echo off
title He Thong Giam Sat Robot Can Bang STM32F4
cd /d "%~dp0"

echo Dang kiem tra thu vien...
python -c "import serial, matplotlib, numpy" 2>nul
if %errorlevel% neq 0 (
    echo Cai dat thu vien can thiet...
    python -m pip install -r requirements.txt
)

echo Khoi chay giao dien...
python gui_app.py
if %errorlevel% neq 0 (
    echo.
    echo Co loi xay ra khi chay ung dung!
    pause
)

