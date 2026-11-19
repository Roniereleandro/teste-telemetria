@echo off
powershell.exe -NoProfile -STA -ExecutionPolicy RemoteSigned -File "%~dp0Bancada-Integra.ps1"
if errorlevel 1 pause
