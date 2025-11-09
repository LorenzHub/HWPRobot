Overview {#mainpage}
========

HWPRobot provides both a library for often used functions in the HWP and a sample main.c file enabling
quick startup in programming the microcontroller and interfacing it to the HWP Control System (HWPCS).
It also contains project files for Eclipse CDT as well as Atmel Studio and Code::Blocks with completely 
pre-configured compiler and linker settings. The relevant files for Eclipse CDT are: .cproject and .project.
Atmel Studio uses the files: HWPRobot.atsln, HWPRobot.componentinfo.xml and HWPRobot.cproj. The project 
files for Code::Blocks are: HWPRobot_Windows.cbp or HWPRobot_LinuxMac.cbp, depending on the OS.

The directory @ref lib contains the library part with the contents as follows:
- @ref communication
  + [communication.h](@ref communication.h): Functions for bi-directional communication with HWPCS
  + [packetTypes.h](@ref packetTypes.h): Definition of communication packet types
- @ref io
  + [adc.h](@ref adc.h): Analog to digital converter library
  + [i2c.h](@ref i2c.h): Sending and receiving of I2C messages
  + [led.h](@ref led.h): Access the user LED on the HWP board
  + [uart.h](@ref uart.h): UART library: controls UART 0,1,2,3 via interrupts and FIFO buffers
- @ref motor
  + [motor.h](@ref motor.h): Motor functions
- @ref pathFollower
  + [pathFollower.h](@ref pathFollower.h): Implementation of a path follower which interfaces with HWPCS
- @ref tools
  + [fifo.h](@ref fifo.h): Simple implementation of a FIFO buffer using a ring buffer (circular buffer)
  + [labyrinth.h](@ref labyrinth.h): Storage and manipulation functions for labyrinth walls
  + [remoteDataProcessing.h](@ref remoteDataProcessing.h): Data and command transfer to a remote data processor (such as MATLAB, Octave, Scilab) for executing commands through HWPCS.
  + [timeTask.h](@ref timeTask.h): Functions for timed execution of code and measurement of execution time
  + [GPIOR0Defs.h](@ref GPIOR0Defs.h): Definitions for timeTask and ADC functions and interrupts concerning the use of the register GPIOR0
  + [powerSaver.h](@ref powerSaver.h): Power saving features for the ATmega1280
  + [variablesAccess.h](@ref variablesAccess.h): Provision of bitset data types and access to memory locations as different data types

The directory @ref src contains the main file and the user configuration of the UART and ADC library:
- [main.c](@ref main.c): sample main file with rudimentary functionalities
- [badISR.c](@ref badISR.c): implementatation of a catch-all ISR for all undefined ISRs
- @ref cfg
  + [adc_cfg.h](@ref adc_cfg.h): User configuration for ADC library
  + [uart_cfg.h](@ref uart_cfg.h): User configuration for UART and communication library

@defgroup lib lib
@{
    @brief HWPRobot library functions
@}

@defgroup communication communication
@{
    @ingroup lib
    @brief Communication function library for interfacing with HWP Control System
@}

@defgroup io io
@{
    @ingroup lib
    @brief I/O function library including ADC, I2C, UART and LED functions
@}

@defgroup motor motor
@{
    @ingroup lib
    @brief Motor control functions
@}

@defgroup pathFollower pathFollower
@{
    @ingroup lib
    @brief Implementation of a path follower
@}

@defgroup tools tools
@{
    @ingroup lib
    @brief Helpful tools including timed tasks, management of labyrinth walls, FIFO, power saving functions, access to memory locations by types and remote data processing
@}

@defgroup src src
@{
    @brief Main file main.c and user configuration for UART and ADC library functions
@}

@defgroup cfg cfg
@{
    @ingroup src
    @brief User configuration for UART and ADC library functions
@}
