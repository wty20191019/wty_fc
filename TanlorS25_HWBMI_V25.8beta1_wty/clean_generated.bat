@echo off
setlocal
cd /d "%~dp0"

for %%D in (".vs" ".visualgdb" "Debug" "Release" "build" "out") do (
    if exist "%%~D" rmdir /s /q "%%~D"
)

del /f /q /s *.user *.suo *.VC.db *.VC.VC.opendb *.dep *.ihex *.hex *.bin *.elf *.map *.old *.bak *.tmp *.log 2>nul

endlocal
