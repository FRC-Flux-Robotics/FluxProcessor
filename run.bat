@echo off
setlocal enabledelayedexpansion

cd /d "%~dp0"

:: 1. Locate java.exe (PATH first, then WPILib 2027/2026/2025 bundled JDK)
set "JAVA_CMD="
where java >nul 2>nul
if %ERRORLEVEL% equ 0 (
    set "JAVA_CMD=java"
) else (
    for %%Y in (2027 2026 2025) do (
        if exist "C:\Users\Public\wpilib\%%Y\jdk\bin\java.exe" (
            set "JAVA_CMD=C:\Users\Public\wpilib\%%Y\jdk\bin\java.exe"
            goto :found_java
        )
    )
    echo [ERROR] Java 17+ was not found in PATH or C:\Users\Public\wpilib\202*\jdk\bin.
    pause
    exit /b 1
)
:found_java

:: 2. Locate flux-processor.jar (or build it automatically if missing)
set "JAR_PATH=dist\flux-processor.jar"
if exist "build\libs\flux-processor.jar" set "JAR_PATH=build\libs\flux-processor.jar"
if not exist "%JAR_PATH%" (
    echo Building flux-processor.jar first...
    call scripts\build.bat
    if %ERRORLEVEL% neq 0 (
        pause
        exit /b %ERRORLEVEL%
    )
    set "JAR_PATH=dist\flux-processor.jar"
)

:: 3. If arguments were passed (CLI or drag-and-drop folder onto run.bat), forward them
if not "%~1"=="" (
    set "INPUT_TARGET=%~1"
    :: If the user dragged a .zip archive from pull-logs.bat onto run.bat, extract it first
    if /i "%~x1"==".zip" (
        set "UNZIP_DIR=%~dpn1_extracted"
        echo Extracting "%~nx1" to "!UNZIP_DIR!"...
        powershell -NoProfile -Command "Expand-Archive -Path '%~f1' -DestinationPath '!UNZIP_DIR!' -Force"
        set "INPUT_TARGET=!UNZIP_DIR!"
    )
    "%JAVA_CMD%" -jar "%JAR_PATH%" "!INPUT_TARGET!" %2 %3 %4 %5 %6 %7 %8 %9
    if exist "!INPUT_TARGET!\runs-index.html" start "" "!INPUT_TARGET!\runs-index.html"
    exit /b %ERRORLEVEL%
)

:: 4. If double-clicked with no arguments, open a Windows Folder Picker dialog
echo Select a match-logs folder to analyze...
for /f "usebackq delims=" %%I in (`powershell -NoProfile -Command "Add-Type -AssemblyName System.Windows.Forms; $f = New-Object System.Windows.Forms.FolderBrowserDialog; $f.Description = 'Select folder containing .wpilog or .hoot match logs'; if ($f.ShowDialog() -eq 'OK') { $f.SelectedPath }"`) do (
    set "SELECTED_DIR=%%I"
)

if not defined SELECTED_DIR (
    echo Cancelled.
    exit /b 0
)

"%JAVA_CMD%" -jar "%JAR_PATH%" "%SELECTED_DIR%"
if exist "%SELECTED_DIR%\runs-index.html" (
    start "" "%SELECTED_DIR%\runs-index.html"
)
pause
