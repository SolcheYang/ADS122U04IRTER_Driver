@echo off
cd /d "%~dp0"
python tools\build.py %*
if errorlevel 1 goto :fail
echo.
echo Build complete. Output: dist\ADS122U04_ADC_Tool.exe
goto :eof
:fail
echo Build FAILED
pause
