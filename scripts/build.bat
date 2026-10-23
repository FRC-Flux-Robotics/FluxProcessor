@echo off
setlocal enabledelayedexpansion

cd /d "%~dp0\.."
set "ROOT_DIR=%CD%"

:: Locate javac / jar (PATH first, then WPILib 2027 / 2026 JDKs)
set "JDK_BIN="
where javac >nul 2>nul
if %ERRORLEVEL% equ 0 (
    set "JAVAC_CMD=javac"
    set "JAVA_CMD=java"
    set "JAR_CMD=jar"
) else (
    for %%Y in (2027 2026 2025) do (
        if exist "C:\Users\Public\wpilib\%%Y\jdk\bin\javac.exe" (
            set "JDK_BIN=C:\Users\Public\wpilib\%%Y\jdk\bin"
            goto :found_jdk
        )
    )
    echo [ERROR] Could not find JDK 25+ in PATH or C:\Users\Public\wpilib\202*\jdk\bin
    exit /b 1
)

:found_jdk
if defined JDK_BIN (
    set "JAVAC_CMD=%JDK_BIN%\javac.exe"
    set "JAVA_CMD=%JDK_BIN%\java.exe"
    set "JAR_CMD=%JDK_BIN%\jar.exe"
)

if not exist "libs" mkdir "libs"
if exist "..\FluxFig\dist\fluxfig.jar" (
    copy /y "..\FluxFig\dist\fluxfig.jar" "libs\fluxfig-1.0.0.jar" >nul
)

if exist "build\classes\main" rmdir /s /q "build\classes\main"
if exist "build\classes\test" rmdir /s /q "build\classes\test"
mkdir "build\classes\main"
mkdir "build\classes\test"
if not exist "build\libs" mkdir "build\libs"
if not exist "dist" mkdir "dist"

echo [1/3] Compiling FluxProcessor (Java 25 bytecode)...
dir /s /b src\main\java\*.java > build\sources-main.txt
"%JAVAC_CMD%" --release 25 -encoding UTF-8 -cp "libs\fluxfig-1.0.0.jar" -d build\classes\main @build\sources-main.txt
if %ERRORLEVEL% neq 0 exit /b %ERRORLEVEL%

dir /s /b src\test\java\*.java > build\sources-test.txt
"%JAVAC_CMD%" --release 25 -encoding UTF-8 -cp "build\classes\main;libs\fluxfig-1.0.0.jar" -d build\classes\test @build\sources-test.txt
if %ERRORLEVEL% neq 0 exit /b %ERRORLEVEL%

echo [2/3] Running FluxProcessor unit ^& integration tests...
"%JAVA_CMD%" -ea -cp "build\classes\main;build\classes\test;libs\fluxfig-1.0.0.jar" frc.flux.processor.FluxProcessorTest
if %ERRORLEVEL% neq 0 exit /b %ERRORLEVEL%

echo [3/3] Packaging self-contained executable build\libs\flux-processor.jar ^& dist\flux-processor.jar...
pushd build\classes\main
"%JAR_CMD%" xf "%ROOT_DIR%\libs\fluxfig-1.0.0.jar" frc
popd
"%JAR_CMD%" --create --file build\libs\flux-processor.jar --main-class frc.flux.processor.FluxProcessorMain -C build\classes\main .
copy /y build\libs\flux-processor.jar dist\flux-processor.jar >nul

echo Done! Executable JAR: %ROOT_DIR%\dist\flux-processor.jar
