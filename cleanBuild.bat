@echo off
chcp 65001 >nul
set "target=AuroX\build"

if exist "%target%" (
    rd /s /q "%target%"
    if errorlevel 1 (
        echo Failed to delete. Check permissions or whether the folder is in use.
    ) else (
        echo Successfully removed %target%.
    )
) else (
    echo Folder %target% does not exist; nothing to clean.
)
pause
