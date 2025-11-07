#!/bin/bash
# Build-Skript für HWPRobot AVR-Projekt
# Verwendet die AVR-Toolchain direkt ohne CMake

set -e  # Beende bei Fehlern

# Pfade konfigurieren
# Prüfe verschiedene mögliche Pfade für die AVR-Toolchain
if [ -d "/usr/bin" ] && command -v avr-gcc &> /dev/null; then
    # System-weite Installation
    AVR_GCC="avr-gcc"
    AVR_OBJCOPY="avr-objcopy"
    AVR_SIZE="avr-size"
    AVR_INCLUDE="/usr/lib/avr/include"
elif [ -d "/usr/local/avr" ]; then
    TOOLCHAIN_PATH="/usr/local/avr"
    AVR_GCC="$TOOLCHAIN_PATH/bin/avr-gcc"
    AVR_OBJCOPY="$TOOLCHAIN_PATH/bin/avr-objcopy"
    AVR_SIZE="$TOOLCHAIN_PATH/bin/avr-size"
    AVR_INCLUDE="$TOOLCHAIN_PATH/avr/include"
elif [ -d "avr8-gnu-toolchain-linux_x86_64" ]; then
    TOOLCHAIN_PATH="avr8-gnu-toolchain-linux_x86_64/avr8-gnu-toolchain-linux_x86_64"
    AVR_GCC="$TOOLCHAIN_PATH/bin/avr-gcc"
    AVR_OBJCOPY="$TOOLCHAIN_PATH/bin/avr-objcopy"
    AVR_SIZE="$TOOLCHAIN_PATH/bin/avr-size"
    AVR_INCLUDE="$TOOLCHAIN_PATH/avr/include"
else
    echo "Fehler: AVR-Toolchain nicht gefunden!"
    echo "Bitte installieren Sie die AVR-Toolchain oder passen Sie TOOLCHAIN_PATH an."
    exit 1
fi

# MCU und Frequenz
MCU="atmega1280"
F_CPU="8000000UL"

# Build-Verzeichnis
BUILD_DIR="Build"
OBJ_DIR="$BUILD_DIR/obj"

# Ausgabedateien
ELF_FILE="$BUILD_DIR/HWPRobot.elf"
HEX_FILE="$BUILD_DIR/HWPRobot.hex"

# Prüfe ob Toolchain vorhanden ist
if ! command -v "$AVR_GCC" &> /dev/null && [ ! -f "$AVR_GCC" ]; then
    echo "Fehler: AVR-Toolchain nicht gefunden: $AVR_GCC" >&2
    exit 1
fi

# Erstelle Build-Verzeichnisse
mkdir -p "$BUILD_DIR"
mkdir -p "$OBJ_DIR"

echo "Kompiliere HWPRobot für $MCU..."

# Alle C-Quelldateien
C_SOURCES=(
    "lib/communication/communication.c"
    "lib/io/adc/adc.c"
    "lib/io/i2c/i2c.c"
    "lib/io/uart/uart.c"
    "lib/motor/motor.c"
    "lib/pathFollower/pathFollower.c"
    "lib/tools/labyrinth/labyrinth.c"
    "lib/tools/remoteDataProcessing/remoteDataProcessing.c"
    "lib/tools/timeTask/timeTask.c"
    "src/badISR.c"
    "src/main.c"
)

# Prüfe ob statemachine.c existiert und füge es hinzu
if [ -f "src/statemachine.c" ]; then
    C_SOURCES+=("src/statemachine.c")
fi

# Prüfe ob bumper.c und encoder.c existieren und füge sie hinzu
if [ -f "src/bumper.c" ]; then
    C_SOURCES+=("src/bumper.c")
fi
if [ -f "src/encoder.c" ]; then
    C_SOURCES+=("src/encoder.c")
fi

# Alle Assembler-Quelldateien
ASM_SOURCES=(
    "lib/io/adc/adc_isr.S"
    "lib/io/uart/uart_isr.S"
    "lib/tools/timeTask/timeTask_isr.S"
)

# Include-Verzeichnisse
INCLUDE_DIRS=(
    "lib"
    "src"
    "$AVR_INCLUDE"
)

# Compiler-Flags
CFLAGS=(
    "-mmcu=$MCU"
    "-DF_CPU=$F_CPU"
    "-O2"
    "-g2"
    "-g"
    "-fpack-struct"
    "-fshort-enums"
    "-funsigned-char"
    "-funsigned-bitfields"
    "-ffunction-sections"
    "-fdata-sections"
    "-Wall"
    "-Wextra"
    "-Wpedantic"
    "-std=gnu99"
)

# Include-Pfade zu Flags hinzufügen
INCLUDE_FLAGS=()
for dir in "${INCLUDE_DIRS[@]}"; do
    INCLUDE_FLAGS+=("-I$dir")
done

# Alle Flags kombinieren
ALL_CFLAGS=("${CFLAGS[@]}" "${INCLUDE_FLAGS[@]}")

# Kompiliere alle C-Dateien
OBJECT_FILES=()
for src in "${C_SOURCES[@]}"; do
    if [ -f "$src" ]; then
        obj_name=$(basename "$src" | sed 's/\.[^.]*$//')
        obj="$OBJ_DIR/${obj_name}.o"
        echo "  Kompiliere: $src"
        $AVR_GCC "${ALL_CFLAGS[@]}" -c "$src" -o "$obj"
        OBJECT_FILES+=("$obj")
    else
        echo "Warnung: Datei nicht gefunden: $src" >&2
    fi
done

# Kompiliere alle Assembler-Dateien
for src in "${ASM_SOURCES[@]}"; do
    if [ -f "$src" ]; then
        obj_name=$(basename "$src" | sed 's/\.[^.]*$//')
        obj="$OBJ_DIR/${obj_name}.o"
        echo "  Kompiliere (ASM): $src"
        $AVR_GCC "${ALL_CFLAGS[@]}" -x assembler-with-cpp -c "$src" -o "$obj"
        OBJECT_FILES+=("$obj")
    else
        echo "Warnung: Datei nicht gefunden: $src" >&2
    fi
done

# Linke alle Objektdateien
echo "  Linke Objektdateien..."
LDFLAGS=(
    "-mmcu=$MCU"
    "-Wl,-Map,$BUILD_DIR/HWPRobot.map"
    "-Wl,--gc-sections"
    "-mrelax"
    "-lm"
)

$AVR_GCC "${LDFLAGS[@]}" -o "$ELF_FILE" "${OBJECT_FILES[@]}"

# Erstelle HEX-Datei
echo "  Erstelle HEX-Datei..."
$AVR_OBJCOPY -j .text -j .data -O ihex "$ELF_FILE" "$HEX_FILE"

# Zeige Größe
echo ""
echo "Build erfolgreich!"
echo ""
$AVR_SIZE "$ELF_FILE"
echo ""
echo "Ausgabedateien:"
echo "  ELF: $ELF_FILE"
echo "  HEX: $HEX_FILE"

