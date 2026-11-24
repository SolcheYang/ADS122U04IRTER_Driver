@echo off
cd /d "%~dp0"
python tools\build.py %*
if errorlevel 1 goto :fail
echo.
echo Build complete. Output: dist\MagSensor.exe
goto :eof
:fail
echo Build FAILED
pause
