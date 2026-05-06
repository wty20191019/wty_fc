@echo off
setlocal
cd /d "%~dp0"

rem Clean IDE caches and build output before packaging
for %%D in (".vs" ".visualgdb" "CodeDB" "Debug" "Release" "build" "out" "obj" "VisualGDBCache") do (
    if exist "%%~D" rmdir /s /q "%%~D"
)

del /f /q /s *.user *.suo *.VC.db *.VC.VC.opendb *.dep *.ihex *.hex *.bin *.elf *.map *.old *.bak *.tmp *.log Tanlor_Vx.sdf 2>nul

endlocal
