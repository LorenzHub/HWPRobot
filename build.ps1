# Build-Skript für HWPRobot AVR-Projekt
# Verwendet die AVR-Toolchain direkt ohne CMake

$ErrorActionPreference = "Stop"

# Pfade konfigurieren
$TOOLCHAIN_PATH = "avr8-gnu-toolchain-4.0.0.52-win32.any.x86_64\avr8-gnu-toolchain-win32_x86_64"
$AVR_GCC = "$TOOLCHAIN_PATH\bin\avr-gcc.exe"
$AVR_OBJCOPY = "$TOOLCHAIN_PATH\bin\avr-objcopy.exe"
$AVR_SIZE = "$TOOLCHAIN_PATH\bin\avr-size.exe"

# MCU und Frequenz
$MCU = "atmega1280"
$F_CPU = "8000000UL"

# Build-Verzeichnis
$BUILD_DIR = "Build"
$OBJ_DIR = "$BUILD_DIR\obj"

# Ausgabedateien
$ELF_FILE = "$BUILD_DIR\HWPRobot.elf"
$HEX_FILE = "$BUILD_DIR\HWPRobot.hex"

# Prüfe ob Toolchain vorhanden ist
if (-not (Test-Path $AVR_GCC)) {
    Write-Host "Fehler: AVR-Toolchain nicht gefunden in $TOOLCHAIN_PATH" -ForegroundColor Red
    exit 1
}

# Erstelle Build-Verzeichnisse
New-Item -ItemType Directory -Force -Path $BUILD_DIR | Out-Null
New-Item -ItemType Directory -Force -Path $OBJ_DIR | Out-Null

Write-Host "Kompiliere HWPRobot für $MCU..." -ForegroundColor Green

# Alle C-Quelldateien
$C_SOURCES = @(
    "lib\communication\communication.c",
    "lib\io\adc\adc.c",
    "lib\io\i2c\i2c.c",
    "lib\io\uart\uart.c",
    "lib\motor\motor.c",
    "lib\pathFollower\pathFollower.c",
    "lib\tools\labyrinth\labyrinth.c",
    "lib\tools\remoteDataProcessing\remoteDataProcessing.c",
    "lib\tools\timeTask\timeTask.c",
    "src\badISR.c",
    "src\main.c"
)

# Prüfe ob statemachine.c existiert und füge es hinzu
if (Test-Path "src\statemachine.c") {
    $C_SOURCES += "src\statemachine.c"
}

# Prüfe ob encoder.c existiert und füge es hinzu
if (Test-Path "src\encoder.c") {
    $C_SOURCES += "src\encoder.c"
}

# Prüfe ob bumper.c existiert und füge es hinzu
if (Test-Path "src\bumper.c") {
    $C_SOURCES += "src\bumper.c"
}

# Prüfe ob calibration.c existiert und füge es hinzu
if (Test-Path "src\calibration.c") {
    $C_SOURCES += "src\calibration.c"
}

# Prüfe ob position.c existiert und füge es hinzu
if (Test-Path "src\position.c") {
    $C_SOURCES += "src\position.c"
}

# Prüfe ob labyrinth.c existiert und füge es hinzu
# Beide Dateien werden benötigt: lib\tools\labyrinth\labyrinth.c für low-level Funktionen,
# src\labyrinth.c für high-level Exploration-Logik
if (Test-Path "src\labyrinth.c") {
    $C_SOURCES += "src\labyrinth.c"
}

# Prüfe ob calcPathCommand.c existiert und füge es hinzu
if (Test-Path "src\calcPathCommand.c") {
    $C_SOURCES += "src\calcPathCommand.c"
}

# Prüfe ob ir_sensors.c existiert und füge es hinzu
if (Test-Path "src\ir_sensors.c") {
    $C_SOURCES += "src\ir_sensors.c"
}

# Alle Assembler-Quelldateien
$ASM_SOURCES = @(
    "lib\io\adc\adc_isr.S",
    "lib\io\uart\uart_isr.S",
    "lib\tools\timeTask\timeTask_isr.S"
)

# Include-Verzeichnisse
$INCLUDE_DIRS = @(
    "lib",
    "src",
    "$TOOLCHAIN_PATH\avr\include"
)

# Compiler-Flags
$CFLAGS = @(
    "-mmcu=$MCU",
    "-DF_CPU=$F_CPU",
    "-O2",
    "-g2",
    "-g",
    "-fpack-struct",
    "-fshort-enums",
    "-funsigned-char",
    "-funsigned-bitfields",
    "-ffunction-sections",
    "-fdata-sections",
    "-Wall",
    "-Wextra",
    "-Wpedantic",
    "-std=gnu99"
)

# Include-Pfade zu Flags hinzufügen
$INCLUDE_FLAGS = $INCLUDE_DIRS | ForEach-Object { "-I$_" }

# Alle Flags kombinieren
$ALL_CFLAGS = $CFLAGS + $INCLUDE_FLAGS

# Kompiliere alle C-Dateien
$OBJECT_FILES = @()
foreach ($src in $C_SOURCES) {
    if (Test-Path $src) {
        # Spezielle Behandlung für labyrinth.c Dateien, um Namenskonflikte zu vermeiden
        if ($src -eq "src\labyrinth.c") {
            $obj = "$OBJ_DIR\src_labyrinth.o"
        } elseif ($src -eq "lib\tools\labyrinth\labyrinth.c") {
            $obj = "$OBJ_DIR\lib_labyrinth.o"
        } else {
            $obj = "$OBJ_DIR\" + [System.IO.Path]::GetFileNameWithoutExtension($src) + ".o"
        }
        Write-Host "  Kompiliere: $src" -ForegroundColor Cyan
        & $AVR_GCC $ALL_CFLAGS -c $src -o $obj
        if ($LASTEXITCODE -ne 0) {
            Write-Host "Fehler beim Kompilieren von $src" -ForegroundColor Red
            exit 1
        }
        $OBJECT_FILES += $obj
    } else {
        Write-Host "Warnung: Datei nicht gefunden: $src" -ForegroundColor Yellow
    }
}

# Kompiliere alle Assembler-Dateien
foreach ($src in $ASM_SOURCES) {
    if (Test-Path $src) {
        $obj = "$OBJ_DIR\" + [System.IO.Path]::GetFileNameWithoutExtension($src) + ".o"
        Write-Host "  Kompiliere (ASM): $src" -ForegroundColor Cyan
        & $AVR_GCC $ALL_CFLAGS -x assembler-with-cpp -c $src -o $obj
        if ($LASTEXITCODE -ne 0) {
            Write-Host "Fehler beim Kompilieren von $src" -ForegroundColor Red
            exit 1
        }
        $OBJECT_FILES += $obj
    } else {
        Write-Host "Warnung: Datei nicht gefunden: $src" -ForegroundColor Yellow
    }
}

# Linke alle Objektdateien
Write-Host "  Linke Objektdateien..." -ForegroundColor Cyan
$LDFLAGS = @(
    "-mmcu=$MCU",
    "-Wl,-Map,$BUILD_DIR\HWPRobot.map",
    "-Wl,--gc-sections",
    "-mrelax",
    "-lm"
)

& $AVR_GCC $LDFLAGS -o $ELF_FILE $OBJECT_FILES
if ($LASTEXITCODE -ne 0) {
    Write-Host "Fehler beim Linken" -ForegroundColor Red
    exit 1
}

# Erstelle HEX-Datei
Write-Host "  Erstelle HEX-Datei..." -ForegroundColor Cyan
& $AVR_OBJCOPY -j .text -j .data -O ihex $ELF_FILE $HEX_FILE
if ($LASTEXITCODE -ne 0) {
    Write-Host "Fehler beim Erstellen der HEX-Datei" -ForegroundColor Red
    exit 1
}

# Zeige Größe
Write-Host ""
Write-Host "Build erfolgreich!" -ForegroundColor Green
Write-Host ""
& $AVR_SIZE $ELF_FILE
Write-Host ""
Write-Host "Ausgabedateien:" -ForegroundColor Green
Write-Host "  ELF: $ELF_FILE"
Write-Host "  HEX: $HEX_FILE"

