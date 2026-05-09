@echo off
setlocal
cd /d "%~dp0"

echo [clean] remove generated directories...
for %%D in (".vs" ".visualgdb" "Debug" "Release" "build" "out" "x64" "x86") do (
    if exist "%%~D" (
        rmdir /s /q "%%~D"
        echo   - removed %%~D
    )
)

echo [clean] remove generated files...
del /f /q /s *.user *.suo *.VC.db *.VC.VC.opendb *.opendb *.dep *.o *.obj *.ihex *.hex *.bin *.elf *.map *.lst *.d *.ilk *.pdb *.idb *.tmp *.log *.tlog *.lastbuildstate *.lck 2>nul

echo [clean] done.

endlocal
