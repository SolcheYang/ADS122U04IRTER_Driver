@echo off
cd /d "%~dp0"
python -m PyInstaller --noconfirm --clean --onefile --noconsole ^
  --name ADS122U04_ADC_Tool ^
  --icon icon.ico ^
  --add-data "icon.ico;." ^
  --version-file version_info.txt ^
  src\ads122u04_tool.py
if errorlevel 1 goto :fail
copy /y dist\ADS122U04_ADC_Tool.exe .\ADS122U04_ADC_Tool.exe >nul
echo.
echo Build OK: %CD%\ADS122U04_ADC_Tool.exe
goto :eof
:fail
echo Build FAILED
pause
