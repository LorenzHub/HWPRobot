#include <io/uart/uart.h>
#include <communication/communication.h>
#include <tools/timeTask/timeTask.h>
#include <tools/powerSaver.h>
#include <io/led/led.h>
#include <motor/motor.h>
#include <pathFollower/pathFollower.h>
#include <tools/remoteDataProcessing/remoteDataProcessing.h>

#include <avr/io.h>                 // AVR IO ports
#include <avr/interrupt.h>          // AVR Interrupts
#include <avr/pgmspace.h>           // AVR Program Space Utilities
#include <util/delay.h>              // AVR delay functions
#include <math.h>                   // Math functions and constants
#include <inttypes.h>               // Integer type conversions for printf, scanf and alike
#include <stdint.h>

#include "statemachine.h"
#include "calibration.h"
#include "io/adc/adc.h"

#include "bumper.h"
#include "encoder.h"
#include "position.h"

/*
 *******************************************************************************
 * PRIVATE VARIABLES
 *******************************************************************************
 */

// robot's pose, initial pose: x=200, y=0, theta=PI/2 (Northern direction)
static Pose_t pose = { 200.0f, 0.0f, M_PI_2 };

// Robot parameters for odometry calculation
static RobotParameters_t robotParams = {
    .axleWidth = 170.0f,      // mm (aus encoder.h Kommentaren)
    .distPerTick = 0.0688f,   // mm/Tick (2048 Ticks = 1 Rad-Umdrehung)
    .user1 = 0.0f,
    .user2 = 0.0f
};

// Getter-Funktion für Robot-Parameter (für position.c)
const RobotParameters_t* getRobotParams(void) {
    return &robotParams;
}


/*
 *******************************************************************************
 * PRIVATE FUNCTIONS
 *******************************************************************************
 */

// callback function for communication channel CH_IN_DEBUG (Debug View in HWPCS)
static void commDebug(__attribute__((unused)) const uint8_t* packet, __attribute__((unused)) const uint16_t size) {
    communication_log(LEVEL_FINE, "received %" PRIu16 " bytes", size);
}

// callback function for communication channel CH_IN_DRIVE (Control Input / Telemetry View in HWPCS)
static void commDriveCommand(const uint8_t* packet, __attribute__((unused)) const uint16_t size) {
}

// callback function for communication channel CH_IN_ROBOT_PARAMS (Scene View in HWPCS)
static void commRobotParameters(const uint8_t* packet, __attribute__((unused)) const uint16_t size) {
    RobotParameters_t* params = (RobotParameters_t*) packet;
    // Speichere Robot-Parameter für Odometrie-Berechnung
    robotParams.axleWidth = params->axleWidth;
    robotParams.distPerTick = params->distPerTick;
    robotParams.user1 = params->user1;
    robotParams.user2 = params->user2;
    communication_log(LEVEL_INFO, "Robot-Parameter aktualisiert: axleWidth=%.2f mm, distPerTick=%.4f mm/Tick", 
                     robotParams.axleWidth, robotParams.distPerTick);
}

// callback function for communication channel CH_IN_POSE (Scene View in HWPCS)
static void commPose(const uint8_t* packet, __attribute__((unused)) const uint16_t size) {
    // handle pose update
}

// callback function for communication channel CH_IN_USER_COMMAND (User Command View in HWPCS)
static void commUserCommand(const uint8_t* packet, __attribute__((unused)) const uint16_t size) {
    UserCommand_t* cmd = (UserCommand_t*) packet;
    switch (cmd->id) {
    case 0: // command ID 0: stop motors
        setState(IDLE); 
        break;
    case 1: // command ID 1: turn on spot
        setState(Turn_On_Spot);
        break;
    case 2: // command ID 2: drive forwards
        setState(Drive_Forward);
        break;
    case 3: // command ID 3: drive forwards for 5 seconds, then stop
        setState(Drive_Forward_5sec);
        break;
    case 4: { //command ID 4: read all infrared sensors
        uint16_t ir1 = ADC_getFilteredValue(0);  // right front
        uint16_t ir2 = ADC_getFilteredValue(1);  // right back
        uint16_t ir3 = ADC_getFilteredValue(2);  // front
        uint16_t ir4 = ADC_getFilteredValue(3);  // left front
        uint16_t ir5 = ADC_getFilteredValue(4);  // left back
        
        communication_log(LEVEL_INFO, "IR1(RF): %" PRIu16 " IR2(RB): %" PRIu16 " IR3(F): %" PRIu16 " IR4(LF): %" PRIu16 " IR5(LB): %" PRIu16,
                         ir1, ir2, ir3, ir4, ir5);
        break;
    }
    case 5: {
        int16_t encoderR = encoder_getCountR();
        int16_t encoderL = encoder_getCountL();
        communication_log(LEVEL_INFO, "Encoder R: %" PRId16 " Encoder L: %" PRId16,
                         encoderR, encoderL);
        break;
        
    }
    case 6: { // command ID 6: Kalibrierungsfahrt - fährt 1000 Encoder-Ticks
        setState(Calibrate_Distance);
        communication_log(LEVEL_INFO, "Kalibrierung gestartet: Fahre 1000 Ticks...");
        break;
    }
    case 7: { // command ID 7: Fahre 50 mm (5 cm) vorwärts
        statemachine_setTargetDistance(50);
        statemachine_setTargetPWM(4000);
        setState(Drive_Forward_Distance);
        communication_log(LEVEL_INFO, "Fahre 50 mm (5 cm) vorwärts mit PWM 4000...");
        break;
    }
    case 8: { // command ID 8: Fahre 100 mm (10 cm) vorwärts
        statemachine_setTargetDistance(100);
        statemachine_setTargetPWM(4000);
        setState(Drive_Forward_Distance);
        communication_log(LEVEL_INFO, "Fahre 100 mm (10 cm) vorwärts mit PWM 4000...");
        break;
    }
    case 9: { // command ID 9: Fahre 5000 ticks vorwärts
        statemachine_setTargetTicks(2048);
        statemachine_setTargetPWM(4000);
        setState(Drive_Forward_Ticks);
        communication_log(LEVEL_INFO, "Fahre 5000 ticks vorwärts mit PWM 4000...");
        break;
    }
    case 10: { // command ID 10: Fahre 1000mm (100 cm) vorwärts
        statemachine_setTargetDistance(1000);
        statemachine_setTargetPWM(4000);
        setState(Drive_Forward_Distance);
        communication_log(LEVEL_INFO, "Fahre 1000mm (100 cm) vorwärts mit PWM 4000...");
        break;
    }
    case 12: { // command ID 12: Drehe 90° links auf der Stelle
        statemachine_setTargetAngle(86);
        statemachine_setTargetPWM(4000);
        setState(Turn_On_Spot_Degrees);
        communication_log(LEVEL_INFO, "Drehe 90° links auf der Stelle mit PWM 4000...");
        break;
    }
    case 13: { // command ID 13: Drehe 180° auf der Stelle
        statemachine_setTargetAngle(172);
        statemachine_setTargetPWM(4000);
        setState(Turn_On_Spot_Degrees);
        communication_log(LEVEL_INFO, "Drehe 180° auf der Stelle mit PWM 4000...");
        break;
    }
    }
}

// callback function for communication channel CH_IN_ADDITIONAL_POSE (Scene View in HWPCS)
static void commAdditionalPose(const uint8_t* packet, __attribute__((unused)) const uint16_t size) {
    // handle additional pose update
}


// initialization
static void init(void) {
    powerSaver_init(); // must be the first call!
    LED_init();
    uart_init();
    communication_init();

    // register communication callback functions which are executed by
    // communication_readPackets() in main loop when a packet is received from
    // HWPCS on the corresponding communication channel
    communication_setCallback(CH_IN_DEBUG, commDebug);
    communication_setCallback(CH_IN_DRIVE, commDriveCommand);
    communication_setCallback(CH_IN_ROBOT_PARAMS, commRobotParameters);
    pathFollower_init();
    communication_setCallback(CH_IN_POSE, commPose);
    communication_setCallback(CH_IN_USER_COMMAND, commUserCommand);
    communication_setCallback(CH_IN_ADDITIONAL_POSE, commAdditionalPose);
    

    Motor_init();
    timeTask_init();
	ADC_init(true);

    bumper_init();
    encoder_init();
    
    // Lade gespeicherte PWM-Kalibrierungswerte aus EEPROM
    calibration_init();
    
    // Initialisiere Position-Modul mit Start-Pose
    position_init(&pose);

    // global interrupt enable
    sei();
}


int main(void) {
    init();
    
    communication_log_P(LEVEL_INFO, PSTR("Booted"));

    // do forever
    for (;;) {

        stateMachine();

        TIMETASK(LED_TASK, 500) { // execute block approximately every 500ms
            LED2_TOGGLE();
        }

        TIMETASK(TELEMETRY_TASK, 300) { // execute block approximately every 300ms
            // send telemetry data to HWPCS
            Telemetry_t telemetry;
            telemetry.bumpers = bumper_getBumpers();
            telemetry.contacts = bumper_getContacts();
            telemetry.encoder1 = encoder_getCountR();
            telemetry.encoder2 = encoder_getCountL();
            telemetry.infrared1 = ADC_getFilteredValue(0); //right front
            telemetry.infrared2 = ADC_getFilteredValue(1); //right back
            telemetry.infrared3 = ADC_getFilteredValue(2); //front
            telemetry.infrared4 = ADC_getFilteredValue(3); //left front
            telemetry.infrared5 = ADC_getFilteredValue(4); //left back
            telemetry.user1 = 20;
            telemetry.user2 = 42.42f;
            communication_writePacket(CH_OUT_TELEMETRY, (uint8_t*)&telemetry, sizeof(telemetry));
        }

        TIMETASK(POSE_TASK, 10) { // execute block approximately every 10ms (100 Hz)
            // Aktualisiere Odometrie
            position_updateExpectedPose();
            
            // Hole aktuelle Pose
            const Pose_t* currentPose = position_getCurrentPose();
            
            // Aktualisiere lokale pose-Variable für Kompatibilität
            pose = *currentPose;
            
            // send pose update to HWPCS
            communication_writePacket(CH_OUT_POSE, (uint8_t*)&pose, sizeof(pose));
        }

        // poll receive buffer (read and parse all available packets from UART buffer)
        // and execute registered callback functions
        communication_readPackets();
    }

    return 0;
}
