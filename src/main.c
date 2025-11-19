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
#include <stddef.h>                 // NULL definition

#include "statemachine.h"
#include "calibration.h"
#include "io/adc/adc.h"

#include "bumper.h"
#include "encoder.h"
#include "position.h"

#include "calcPathCommand.h"
#include "labyrinth.h"

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
static void commDriveCommand(__attribute__((unused)) const uint8_t* packet, __attribute__((unused)) const uint16_t size) {
}

// callback function for communication channel CH_IN_ROBOT_PARAMS (Scene View in HWPCS)
static void commRobotParameters(const uint8_t* packet, __attribute__((unused)) const uint16_t size) {
    RobotParameters_t* params = (RobotParameters_t*) packet;
    // Speichere Robot-Parameter für Odometrie-Berechnung
    robotParams.axleWidth = params->axleWidth;
    robotParams.distPerTick = params->distPerTick;
    robotParams.user1 = params->user1;
    robotParams.user2 = params->user2;
    // Konvertiere Float-Werte zu Integer für Log-Ausgabe (AVR unterstützt kein Float-Format)
    int16_t axleWidth_cm = (int16_t)(robotParams.axleWidth / 10.0f);  // in cm
    int16_t distPerTick_um = (int16_t)(robotParams.distPerTick * 1000.0f);  // in Mikrometer
    communication_log(LEVEL_INFO, "Robot-Parameter: axleWidth=%d cm, distPerTick=%d um/Tick", 
                     axleWidth_cm, distPerTick_um);
}

// callback function for communication channel CH_IN_POSE (Scene View in HWPCS)
static void commPose(const uint8_t* packet, const uint16_t size) {
    // WICHTIG: Diese Funktion wird aufgerufen, wenn HWPCS eine Pose sendet
    // Prüfe Paketgröße
    if (size != sizeof(Pose_t)) {
        communication_log(LEVEL_WARNING, "commPose: Falsche Paketgroesse: %" PRIu16 " Bytes (erwartet: %zu)", size, sizeof(Pose_t));
        return;
    }
    Pose_t* aprilTagPose = (Pose_t*)packet;
    
    // Setze Pose (wichtig: vor Log-Ausgabe, damit sie gespeichert ist)
    position_setAprilTagPose(aprilTagPose);
    
    // Konvertiere Float-Werte zu Integer für Log-Ausgabe (AVR unterstützt kein Float-Format)
    int16_t x = (int16_t)(aprilTagPose->x);
    int16_t y = (int16_t)(aprilTagPose->y);
    int16_t theta_mrad = (int16_t)(aprilTagPose->theta * 1000.0f);  // Theta in Milliradiant
    communication_log(LEVEL_INFO, "Kamera-Pose empfangen: x=%d y=%d theta=%d mrad", 
                     x, y, theta_mrad);
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
        
    }/*
    case 6: { // command ID 6: Kalibrierungsfahrt - fährt 1000 Encoder-Ticks
        setState(Calibrate_Distance);
        communication_log(LEVEL_INFO, "Kalibrierung gestartet: Fahre 1000 Ticks...");
        break;
    }*/
   case 6:{
    statemachine_setTargetDistance(10000); // Fahre 10000 mm (10 m)
    statemachine_setTargetPWM(8000);       // mit PWM 4000
    setState(Drive_Forward_Distance);
    communication_log(LEVEL_INFO, "Fahre 10000 mm (10 m) vorwärts mit PWM 3000...");
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
    case 14: { // command ID 14: Synchronisiere Odometrie mit Kamera-Pose
        const Pose_t* currentPose = position_getCurrentPose();
        const Pose_t* cameraPose = position_getAprilTagPose();
        uint32_t age_us = position_getLastCameraUpdateTime();
        uint8_t hasValid = position_hasValidCameraPose();
        uint8_t wasReceived = position_wasCameraPoseReceived();
        
        // Konvertiere Float-Werte zu Integer für Log-Ausgabe (AVR unterstützt kein Float-Format)
        int16_t odom_x = (int16_t)(currentPose->x);
        int16_t odom_y = (int16_t)(currentPose->y);
        int16_t odom_theta_mrad = (int16_t)(currentPose->theta * 1000.0f);  // Theta in Milliradiant
        
        int16_t cam_x = (int16_t)(cameraPose->x);
        int16_t cam_y = (int16_t)(cameraPose->y);
        int16_t cam_theta_mrad = (int16_t)(cameraPose->theta * 1000.0f);
        
        communication_log(LEVEL_INFO, "Sync-Status: wasReceived=%d, age=%" PRIu32 " us, hasValid=%d", 
                         wasReceived, age_us, hasValid);
        communication_log(LEVEL_INFO, "Odometrie: x=%d y=%d theta=%d mrad", 
                         odom_x, odom_y, odom_theta_mrad);
        communication_log(LEVEL_INFO, "Kamera: x=%d y=%d theta=%d mrad", 
                         cam_x, cam_y, cam_theta_mrad);
        
        if (position_syncPoses()) {
            currentPose = position_getCurrentPose();
            odom_x = (int16_t)(currentPose->x);
            odom_y = (int16_t)(currentPose->y);
            odom_theta_mrad = (int16_t)(currentPose->theta * 1000.0f);
            communication_log(LEVEL_INFO, "Posen synchronisiert: x=%d y=%d theta=%d mrad", 
                             odom_x, odom_y, odom_theta_mrad);
        } else {
            if (age_us == UINT32_MAX) {
                communication_log(LEVEL_WARNING, "Synchronisierung fehlgeschlagen: Kamera-Pose wurde noch nie empfangen");
            } else {
                communication_log(LEVEL_WARNING, "Synchronisierung fehlgeschlagen: Kamera-Pose zu alt (%" PRIu32 " us, max 10000 ms)", 
                                 age_us);
            }
        }
        break;
    }
    case 15: { // command ID 15: Sende sofortige Pose-Anfrage an HWPCS
        GetPose_t getPose;
        getPose.aprilTagType = APRIL_TAG_MAIN;
        communication_writePacket(CH_OUT_GET_POSE, (uint8_t*)&getPose, sizeof(GetPose_t));
        communication_log(LEVEL_INFO, "Pose-Anfrage an HWPCS gesendet");
        break;
    }
    case 16: {
        setState(ExploreMaze);
        communication_log(LEVEL_INFO, "Starte Labyrinth-Erkundung");
        break;
    }
    }
}

// callback function for communication channel CH_IN_ADDITIONAL_POSE (Scene View in HWPCS)
static void commAdditionalPose(__attribute__((unused)) const uint8_t* packet, __attribute__((unused)) const uint16_t size) {
    // handle additional pose update
}


// initialization
static void init(void) {
    powerSaver_init(); // must be the first call!
    LED_init();
    uart_init();
    communication_init();
    pathFollower_init();

    // register communication callback functions which are executed by
    // communication_readPackets() in main loop when a packet is received from
    // HWPCS on the corresponding communication channel
    communication_setCallback(CH_IN_DEBUG, commDebug);
    communication_setCallback(CH_IN_DRIVE, commDriveCommand);
    communication_setCallback(CH_IN_ROBOT_PARAMS, commRobotParameters);
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

        TIMETASK(POSE_TASK, 20) { // execute block approximately every 20ms (50 Hz)
            // Aktualisiere Odometrie
            position_updateExpectedPose();
            
            // Berechne Differenz zwischen Odometrie und Kamera-Pose
            position_calculatePoseDifference();
            
            // Wende automatische Korrektur an (gewichtete Sensorfusion)
            position_applyCorrection();
            
            // Hole aktuelle Pose
            const Pose_t* currentPose = position_getCurrentPose();
            
            // Aktualisiere lokale pose-Variable für Kompatibilität
            pose = *currentPose;
            
            // send pose update to HWPCS
            // Verwende sizeof(Pose_t) statt sizeof(pose) für Konsistenz
            // Pose_t sollte 12 Bytes sein (3 * 4 Bytes float)
            static uint16_t poseSize = sizeof(Pose_t);
            if (poseSize != 12) {
                // Warnung nur einmal loggen
                static uint8_t sizeWarningLogged = 0;
                if (!sizeWarningLogged) {
                    communication_log(LEVEL_WARNING, "Pose_t size mismatch: %" PRIu16 " bytes (expected 12)", poseSize);
                    sizeWarningLogged = 1;
                }
            }
            communication_writePacket(CH_OUT_POSE, (uint8_t*)&pose, poseSize);
        }

        TIMETASK(GET_POSE_TASK, 500) { // execute block approximately every 500ms (2 Hz)
            // Fordere Kamera-Pose von HWPCS an
            // HWPCS antwortet mit Pose_t auf CH_IN_POSE (wird von commPose() verarbeitet)
            GetPose_t getPose;
            getPose.aprilTagType = APRIL_TAG_MAIN;
            communication_writePacket(CH_OUT_GET_POSE, (uint8_t*)&getPose, sizeof(GetPose_t));
            
            // Logge nur alle 10 Sekunden (20 Anfragen) um Log-Spam zu vermeiden
            static uint16_t requestCount = 0;
            if (++requestCount >= 20) {
                communication_log(LEVEL_FINE, "Pose-Anfrage gesendet (automatisch, alle 500ms)");
                requestCount = 0;
            }

        }

        TIMETASK(USER_DATA_TASK, 100) { // execute block approximately every 100ms (10 Hz)
            // Sende Pose-Differenz an HWPCS für grafische Visualisierung im User Data View
            // float1 = Differenz X (mm), float2 = Differenz Y (mm), float3 = Differenz Theta (rad)
            const Pose_t* poseDiff = position_getPoseDifference();
            UserData_t userData;
            userData.uint16 = 0;
            userData.uint32 = 0;
            userData.int16 = 0;
            userData.int32 = 0;
            userData.float1 = poseDiff->x;      // X-Differenz in mm
            userData.float2 = poseDiff->y;      // Y-Differenz in mm
            userData.float3 = poseDiff->theta;  // Theta-Differenz in rad
            userData.float4 = 0.0f;             // Reserve
            communication_writePacket(CH_OUT_USER_DATA, (uint8_t*)&userData, sizeof(userData));
        }

        // poll receive buffer (read and parse all available packets from UART buffer)
        // and execute registered callback functions
        communication_readPackets();

        TIMETASK(FOLLOWER_TASK, 20) { // execute block approximately every 20ms (50 Hz)
            const Pose_t* currentPose = position_getCurrentPose();
            const PathFollowerStatus_t* pathFollower_status = pathFollower_getStatus();
            if (pathFollower_status->enabled) {
                if (pathFollower_update(currentPose)){
                    setState(FollowThePath);
                    calculateDriveCommand(currentPose, &pathFollower_status->lookahead);
                }
                else{
                Motor_stopAll();  
                pathFollower_command(FOLLOWER_CMD_RESET);
                }
            }
            sendPathFollowerStatus(pathFollower_status);
        }
    }

    return 0;
}
