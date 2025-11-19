#include "ir_sensors.h"
#include "io/adc/adc.h"
#include "cfg/io/adc/adc_cfg.h"
#include <stddef.h>

// Threshold for wall detection in mm
#define WALL_DETECTION_THRESHOLD_MM 50
#define WALL_DETECTION_HYSTERESIS_MM 5

// Lookup Table for GP2Y0A41SK0F based on provided calibration data
// ADC values calculated for 5V reference (ADC = mV * 1023 / 5000)
// Pairs of {ADC_Value, Distance_mm}
// Ordered by ADC value ASCENDING (low ADC = far, high ADC = close)
// Calibration data from user measurements:
// 100mm: 420mV -> 86 ADC
// 90mm: 460mV -> 94 ADC
// 80mm: 530mV -> 108 ADC
// 70mm: 620mV -> 127 ADC
// 60mm: 700mV -> 143 ADC
// 50mm: 800mV -> 164 ADC
// 40mm: 930mV -> 190 ADC
// 35mm: 940mV -> 192 ADC (Peak - minimum valid distance, sensor unreliable < 4cm)
// 30mm: 930mV -> 190 ADC (Voltage drops again - invalid range)

typedef struct {
    uint16_t adc_val;
    uint16_t dist_mm;
} CalibrationPoint_t;

static const CalibrationPoint_t calibration_table[] = {
    {0, 300},   // Far away (extrapolated)
    {50, 150},  // Extrapolated
    {70, 120},  // Extrapolated
    {86, 100},  // Measured: 420mV
    {94, 90},   // Measured: 460mV
    {108, 80},  // Measured: 530mV
    {127, 70},  // Measured: 620mV
    {143, 60},  // Measured: 700mV
    {164, 50},  // Measured: 800mV
    {190, 40},  // Measured: 930mV
    {192, 35}   // Measured: 940mV (Peak - min valid distance)
};

#define CALIBRATION_POINTS (sizeof(calibration_table) / sizeof(CalibrationPoint_t))

void ir_sensors_init(void) {
    // ADC is initialized in main.c via ADC_init(true)
}

uint16_t ir_sensor_get_raw(uint8_t sensor_index) {
    if (sensor_index >= ADC_CHANNEL_COUNT) {
        return 0;
    }
    return ADC_getFilteredValue(sensor_index);
}

uint16_t ir_sensor_get_distance_mm(uint8_t sensor_index) {
    uint16_t adc = ir_sensor_get_raw(sensor_index);

    // Check limits - table is now ascending (low ADC = far, high ADC = close)
    if (adc > calibration_table[CALIBRATION_POINTS - 1].adc_val) {
        // Too close or invalid (voltage drops again < 4cm, or beyond peak)
        // Return min valid distance (35mm)
        return calibration_table[CALIBRATION_POINTS - 1].dist_mm;
    }
    if (adc < calibration_table[0].adc_val) {
        // Too far (below minimum ADC in table)
        return calibration_table[0].dist_mm;
    }

    // Linear Interpolation - table is ascending by ADC
    for (uint8_t i = 0; i < CALIBRATION_POINTS - 1; i++) {
        if (adc >= calibration_table[i].adc_val && adc <= calibration_table[i+1].adc_val) {
            uint16_t adc_low = calibration_table[i].adc_val;
            uint16_t adc_high = calibration_table[i+1].adc_val;
            uint16_t dist_low = calibration_table[i].dist_mm;      // Far distance (low ADC)
            uint16_t dist_high = calibration_table[i+1].dist_mm;   // Close distance (high ADC)

            // Interpolation formula (linear between two points):
            // dist = dist_low + (dist_high - dist_low) * (adc - adc_low) / (adc_high - adc_low)
            // Note: dist_high < dist_low because close = smaller distance value
            
            if (adc_high == adc_low) {
                // Avoid division by zero
                return dist_low;
            }
            
            uint32_t dist = (uint32_t)dist_low + 
                            ((uint32_t)(dist_high - dist_low) * (uint32_t)(adc - adc_low)) / 
                            (adc_high - adc_low);
            
            return (uint16_t)dist;
        }
    }

    return 300; // Default far (should not reach here)
}

bool ir_sensor_detects_wall(uint8_t sensor_index) {
    uint16_t dist = ir_sensor_get_distance_mm(sensor_index);
    
    // Simple hysteresis could be stateful, but here we just use the threshold
    // < 50mm is considered a wall based on specs
    return (dist < WALL_DETECTION_THRESHOLD_MM);
}

