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
#include <stdbool.h>                // Boolean type

#include "statemachine.h"
#include "io/adc/adc.h"

#include "bumper.h"
#include "encoder.h"

/*
 *******************************************************************************
 * PRIVATE VARIABLES
 *******************************************************************************
 */

// robot's pose, initial pose: x=200, y=0, theta=PI/2 (Northern direction)
static Pose_t pose = { 200.0f, 0.0f, M_PI_2 };

// Flag to control continuous output of infrared sensor 3
static bool infrared3_output_active = false;

// Encoder calibration state
static bool encoder_calibration_active = false;
static uint8_t calibration_sample_count = 0;
static bool calibration_rotation_phase = false; // false = forward, true = rotation
// Buffer for calibration data (store locally, send at end)
static int16_t calibration_enc1[250];
static int16_t calibration_enc2[250];
static int16_t calibration_pwm[250];
static uint8_t calibration_phase[250];


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
    // handle robot parameters
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
        Motor_stopAll();
        break;
    case 1: // command ID 1: turn on spot
        Motor_setPWM(3000, -3000);
        break;
    case 2: // command ID 2: drive forwards
        Motor_setPWM(3000, 3000);
        break;
    case 3: // command ID 3: drive forwards for 5 seconds, then stop
        setState(Drive_Forward_5sec);
        break;
    case 4: // command ID 4: plot infrared sensor values
        {
            // Clear previous script
            remoteDataProcessing_clear();
            
            // Collect samples and send to HWPCS
            for (uint8_t i = 0; i < 50; ++i) {
                uint16_t ir1 = ADC_getFilteredValue(0);
                uint16_t ir2 = ADC_getFilteredValue(1);
                uint16_t ir3 = ADC_getFilteredValue(2);
                uint16_t ir4 = ADC_getFilteredValue(3);
                uint16_t ir5 = ADC_getFilteredValue(4);
                
                remoteDataProcessing_command(false, "ir_data(%"PRIu8",:) = [%"PRIu16 ", %"PRIu16 ", %"PRIu16 ", %"PRIu16 ", %"PRIu16 "];", 
                    i+1, ir1, ir2, ir3, ir4, ir5);
                
                // Small delay between samples
                _delay_ms(50);
            }
            
            // Create plot script with 5 subplots (2 rows, 3 columns)
            remoteDataProcessing_command(false, "figure;");
            remoteDataProcessing_command(false, "subplot(2,3,1); plot(ir_data(:,1)); title('Infrared Sensor 1'); ylabel('ADC Value'); grid on;");
            remoteDataProcessing_command(false, "subplot(2,3,2); plot(ir_data(:,2)); title('Infrared Sensor 2'); ylabel('ADC Value'); grid on;");
            remoteDataProcessing_command(false, "subplot(2,3,3); plot(ir_data(:,3)); title('Infrared Sensor 3'); ylabel('ADC Value'); grid on;");
            remoteDataProcessing_command(false, "subplot(2,3,4); plot(ir_data(:,4)); title('Infrared Sensor 4'); ylabel('ADC Value'); grid on;");
            remoteDataProcessing_command(false, "subplot(2,3,5); plot(ir_data(:,5)); title('Infrared Sensor 5'); ylabel('ADC Value'); grid on;");
            
            // Execute script
            remoteDataProcessing_command(true, "disp('Infrared sensor data plotted');");
        }
        break;
    case 5: // command ID 5: start continuous output of infrared sensor 3
        infrared3_output_active = true;
        communication_log_P(LEVEL_INFO, PSTR("Infrared Sensor 3 continuous output started"));
        break;
    case 6: // command ID 6: stop continuous output of infrared sensor 3
        infrared3_output_active = false;
        communication_log_P(LEVEL_INFO, PSTR("Infrared Sensor 3 continuous output stopped"));
        break;
    case 7: // command ID 7: start encoder calibration (3 meters with increasing speed)
        {
            // Reset encoders
            encoder_reset(1);
            encoder_reset(2);
            
            // Clear previous script
            remoteDataProcessing_clear();
            
            // Initialize calibration
            encoder_calibration_active = true;
            calibration_sample_count = 0;
            calibration_rotation_phase = false;
            
            // Clear calibration buffers
            for (uint8_t i = 0; i < 250; i++) {
                calibration_enc1[i] = 0;
                calibration_enc2[i] = 0;
                calibration_pwm[i] = 0;
                calibration_phase[i] = 0;
            }
            
            // Start driving forward with initial low speed
            setState(Encoder_Calibration);
            Motor_setPWM(1000, 1000); // Start with low speed
            
            communication_log_P(LEVEL_INFO, PSTR("Encoder calibration started - driving 3m with increasing speed"));
        }
        break;
    case 8: // command ID 8: stop encoder calibration
        encoder_calibration_active = false;
        Motor_stopAll();
        setState(IDLE);
        communication_log_P(LEVEL_INFO, PSTR("Encoder calibration stopped"));
        break;
    case 9: // command ID 9: test encoder pins (debug)
        {
            uint8_t raw1 = encoder_getRawPins(1);
            uint8_t raw2 = encoder_getRawPins(2);
            int16_t val1 = encoder_getValue(1);
            int16_t val2 = encoder_getValue(2);
            communication_log(LEVEL_INFO, "Encoder Test: E1=%"PRId16" (Pins=0x%02X), E2=%"PRId16" (Pins=0x%02X)", 
                val1, raw1, val2, raw2);
        }
        break;
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
    
    // Initialize ADC with JTAG disabled to allow use of ADC4 (PF4) for infrared sensor 5
    // Note: ADC_init() calls timeTask_init() internally, so we don't need to call it separately
    ADC_init(true);

    bumper_init();
    encoder_init();

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
            telemetry.bumpers.value = 0; // initialize with zero
            telemetry.bumpers.bitset.bit1 = 1;
            telemetry.contacts = 0;
            telemetry.encoder1 = encoder_getValue(1);
            telemetry.encoder2 = encoder_getValue(2);
            // Read filtered ADC values for infrared sensors (filtered values are more stable)
            telemetry.infrared1 = ADC_getFilteredValue(0); //right front
            telemetry.infrared2 = ADC_getFilteredValue(1); //right back
            telemetry.infrared3 = ADC_getFilteredValue(2); //front
            telemetry.infrared4 = ADC_getFilteredValue(3); //left front
            telemetry.infrared5 = ADC_getFilteredValue(4); //left back
            telemetry.user1 = 20;
            telemetry.user2 = 42.42f;
            communication_writePacket(CH_OUT_TELEMETRY, (uint8_t*)&telemetry, sizeof(telemetry));
        }

        TIMETASK(POSE_TASK, 150) { // execute block approximately every 150ms
            static int t_index = 0; // time index

            // simulate the robot moving on a circular trajectory with a frequency of 0.1Hz
            // angle = omega * t = (2 * PI * 0.1Hz) * (t_index * 0.15s)
            float wt = 2.0f * M_PI * 0.1f * (float)t_index * 0.15f;
            pose.x = 200.0f * cosf(wt);
            pose.y = 200.0f * sinf(wt);
            pose.theta = wt + M_PI_2;
            ++t_index;

            // send pose update to HWPCS
            communication_writePacket(CH_OUT_POSE, (uint8_t*)&pose, sizeof(pose));
        }

        TIMETASK(INFRARED3_OUTPUT_TASK, 100) { // execute block approximately every 100ms
            if (infrared3_output_active) {
                uint16_t sensorValue = ADC_getFilteredValue(2); // Channel 2 = Sensor 3 (front)
                
                communication_log(LEVEL_INFO, "Infrared Sensor 3: %" PRIu16, sensorValue);
            }
        }

        TIMETASK(ENCODER_CALIBRATION_TASK, 100) { // execute block approximately every 100ms
            if (encoder_calibration_active && currentState == Encoder_Calibration) {
                int16_t enc1 = encoder_getValue(1);
                int16_t enc2 = encoder_getValue(2);
                
                if (!calibration_rotation_phase) {
                    // Phase 1: Forward movement (3 meters)
                    // Collect data (max 150 samples = 15 seconds for 3 meters)
                    if (calibration_sample_count < 150) {
                        // Calculate current PWM speed (ramp from 1000 to 4000 over 150 samples)
                        // Linear ramp: speed = 1000 + (calibration_sample_count * (4000-1000) / 150)
                        int16_t current_pwm = 1000 + (calibration_sample_count * 3000 / 150);
                        if (current_pwm > 4000) current_pwm = 4000;
                        
                        // Update motor speed
                        Motor_setPWM(current_pwm, current_pwm);
                        
                        // Store data locally (don't send during collection to avoid communication overload)
                        calibration_enc1[calibration_sample_count] = enc1;
                        calibration_enc2[calibration_sample_count] = enc2;
                        calibration_pwm[calibration_sample_count] = current_pwm;
                        calibration_phase[calibration_sample_count] = 1;
                        
                        // Log progress every 10 samples (1 second) to show activity
                        if ((calibration_sample_count % 10) == 0) {
                            uint8_t raw1 = encoder_getRawPins(1);
                            uint8_t raw2 = encoder_getRawPins(2);
                            communication_log(LEVEL_INFO, "Calib Forward: Sample %"PRIu8"/150, E1=%"PRId16" E2=%"PRId16" PWM=%"PRId16" Pins1=0x%02X Pins2=0x%02X", 
                                calibration_sample_count, enc1, enc2, current_pwm, raw1, raw2);
                        }
                        
                        calibration_sample_count++;
                    } else {
                        // Switch to rotation phase
                        calibration_rotation_phase = true;
                        calibration_sample_count = 0; // Reset counter for rotation phase
                        Motor_setPWM(3000, -3000); // Turn on spot (left motor forward, right motor backward)
                        communication_log_P(LEVEL_INFO, PSTR("Starting rotation phase"));
                    }
                } else {
                    // Phase 2: Rotation (360 degrees)
                    // Collect data (max 100 samples = 10 seconds for rotation)
                    if (calibration_sample_count < 100) {
                        // Store data locally
                        calibration_enc1[150 + calibration_sample_count] = enc1;
                        calibration_enc2[150 + calibration_sample_count] = enc2;
                        calibration_pwm[150 + calibration_sample_count] = 3000;
                        calibration_phase[150 + calibration_sample_count] = 2;
                        
                        // Log progress every 10 samples (1 second) to show activity
                        if ((calibration_sample_count % 10) == 0) {
                            communication_log(LEVEL_INFO, "Calib Rotation: Sample %"PRIu8"/100, E1=%"PRId16" E2=%"PRId16, 
                                calibration_sample_count, enc1, enc2);
                        }
                        
                        calibration_sample_count++;
                    } else {
                        // Stop after rotation phase
                        encoder_calibration_active = false;
                        calibration_rotation_phase = false;
                        Motor_stopAll();
                        setState(IDLE);
                        
                        communication_log_P(LEVEL_INFO, PSTR("Sending calibration data to HWPCS..."));
                        
                        // Send all data at once (after collection is complete)
                        // Send data individually to avoid packet size issues
                        remoteDataProcessing_clear();
                        remoteDataProcessing_command(false, "encoder_cal_data = zeros(250, 2);");
                        remoteDataProcessing_command(false, "sample_time = zeros(250, 1);");
                        remoteDataProcessing_command(false, "pwm_speed = zeros(250, 1);");
                        remoteDataProcessing_command(false, "phase = zeros(250, 1);");
                        
                        // Send data one by one (but only after collection, not during)
                        for (uint8_t i = 0; i < 250; i++) {
                            remoteDataProcessing_command(false, "encoder_cal_data(%"PRIu8",:) = [%"PRId16 ", %"PRId16 "];", 
                                i+1, calibration_enc1[i], calibration_enc2[i]);
                            remoteDataProcessing_command(false, "sample_time(%"PRIu8") = %"PRIu8";", 
                                i+1, i);
                            remoteDataProcessing_command(false, "pwm_speed(%"PRIu8") = %"PRId16 ";", 
                                i+1, calibration_pwm[i]);
                            remoteDataProcessing_command(false, "phase(%"PRIu8") = %"PRIu8 ";", 
                                i+1, calibration_phase[i]);
                        }
                        
                        communication_log_P(LEVEL_INFO, PSTR("Data sent, creating plots..."));
                        
                        // Calculate calibration factors for forward movement (first 150 samples)
                        // Note: Motor is 2x faster than wheels (2:1 gear ratio)
                        // Encoder counts motor rotations, so we need to divide by 2 to get wheel rotations
                        remoteDataProcessing_command(false, "forward_ticks_1 = encoder_cal_data(150,1) - encoder_cal_data(1,1);");
                        remoteDataProcessing_command(false, "forward_ticks_2 = encoder_cal_data(150,2) - encoder_cal_data(1,2);");
                        remoteDataProcessing_command(false, "avg_forward_ticks = (forward_ticks_1 + forward_ticks_2) / 2;");
                        remoteDataProcessing_command(false, "wheel_ticks = avg_forward_ticks / 2; % Motor 2x faster than wheels (2:1 gear ratio)");
                        remoteDataProcessing_command(false, "distance_mm = 3000; % 3 meters");
                        remoteDataProcessing_command(false, "ticks_per_mm = wheel_ticks / distance_mm; % Encoder ticks per mm (wheel movement)");
                        remoteDataProcessing_command(false, "mm_per_tick = distance_mm / wheel_ticks; % mm per encoder tick (wheel movement)");
                        remoteDataProcessing_command(false, "motor_ticks_per_mm = avg_forward_ticks / distance_mm; % Motor encoder ticks per mm");
                        
                        // Calculate rotation data (samples 151-250)
                        // For rotation: one wheel forward, one backward, so both encoders count
                        remoteDataProcessing_command(false, "rotation_ticks_1 = encoder_cal_data(250,1) - encoder_cal_data(151,1);");
                        remoteDataProcessing_command(false, "rotation_ticks_2 = encoder_cal_data(250,2) - encoder_cal_data(151,2);");
                        remoteDataProcessing_command(false, "rotation_diff = rotation_ticks_1 - rotation_ticks_2; % Should be positive for 360° turn");
                        remoteDataProcessing_command(false, "avg_rotation_ticks = (abs(rotation_ticks_1) + abs(rotation_ticks_2)) / 2;");
                        remoteDataProcessing_command(false, "wheel_rotation_ticks = avg_rotation_ticks / 2; % Motor 2x faster than wheels");
                        remoteDataProcessing_command(false, "ticks_per_degree = wheel_rotation_ticks / 360; % Encoder ticks per degree (wheel rotation)");
                        
                        // Create plot script with 4 subplots (after calculations)
                        // Use simpler plot commands that work in both MATLAB and Octave
                        remoteDataProcessing_command(false, "figure;");
                        remoteDataProcessing_command(false, "subplot(4,1,1); plot(sample_time, encoder_cal_data(:,1), 'b-'); hold on; plot(sample_time, encoder_cal_data(:,2), 'r-'); title('Encoder Calibration Data (3m forward + 360° rotation)'); xlabel('Sample Number'); ylabel('Encoder Ticks'); legend('Encoder 1', 'Encoder 2'); grid on; hold off;");
                        remoteDataProcessing_command(false, "subplot(4,1,2); plot(sample_time, encoder_cal_data(:,1) - encoder_cal_data(:,2), 'g-'); title('Encoder Difference (E1 - E2)'); xlabel('Sample Number'); ylabel('Tick Difference'); grid on;");
                        remoteDataProcessing_command(false, "subplot(4,1,3); plot(sample_time, pwm_speed, 'm-'); title('PWM Speed Over Time'); xlabel('Sample Number'); ylabel('PWM Value'); grid on;");
                        remoteDataProcessing_command(false, "subplot(4,1,4); plot(sample_time, phase, 'k-'); title('Phase (1=Forward, 2=Rotation)'); xlabel('Sample Number'); ylabel('Phase'); grid on;");
                        remoteDataProcessing_command(false, "drawnow;");
                        
                        // Execute script (calculations first, then plots, then output)
                        remoteDataProcessing_command(true, "fprintf('Calibration complete!\\nData points: %d\\n\\nMotor gear ratio: 2:1 (Motor 2x faster than wheels)\\n\\nForward (3m):\\nMotor ticks per mm: %.4f\\nWheel ticks per mm: %.4f\\nmm per wheel tick: %.4f\\n\\nRotation (360°):\\nWheel ticks per degree: %.4f\\nEncoder difference: %d\\n\\nPlots should be visible now.\\n', length(sample_time), motor_ticks_per_mm, ticks_per_mm, mm_per_tick, ticks_per_degree, rotation_diff);");
                        
                        communication_log_P(LEVEL_INFO, PSTR("Encoder calibration complete - 3m forward + 360° rotation - data plotted"));
                    }
                }
            }
        }

        // poll receive buffer (read and parse all available packets from UART buffer)
        // and execute registered callback functions
        communication_readPackets();
    }

    return 0;
}
