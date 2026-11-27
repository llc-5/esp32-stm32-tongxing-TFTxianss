@echo off
rem ============================================================
rem  一键编译 + 烧录 + 校验。真正的逻辑都在 flash.ps1 里。
rem
rem  用法:
rem      flash.bat            烧到默认 COM10
rem      flash.bat COM12      烧到 COM12
rem
rem  需要整片擦除 / 跳过编译等高级选项，直接用 PowerShell:
rem      .\flash.ps1 -Full
rem      .\flash.ps1 -NObuild -Port COM12
rem ============================================================
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0flash.ps1" %*
if errorlevel 1 (
    echo.
    echo 烧录流程失败，看上面的报错。
    pause
)
endlocal
