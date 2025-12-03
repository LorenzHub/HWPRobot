#include "ir_sensors.h"
#include "io/adc/adc.h"
#include <communication/communication.h>
#include <stddef.h>

// Kalibrierungsparameter für jeden IR-Sensor
typedef struct {
    float x;  // Kalibrierungsparameter x
    float y;  // Kalibrierungsparameter y
} sensor_calibration_t;

// Sensor-spezifische Kalibrierungsparameter
static const sensor_calibration_t sensor_calibrations[] = {
    {1598.666f, 269.7707f},  // Sensor 0 (RIGHT_FRONT)
    {1927.106f, 228.9249f},  // Sensor 1 (RIGHT_BACK)
    {2403.686f, 183.5166f},  // Sensor 2 (FRONT)
    {2447.765f, 186.2681f},  // Sensor 3 (LEFT_FRONT)
    {2083.542f, 206.0224f}   // Sensor 4 (LEFT_BACK)
};

/**
 * @brief Konvertiert ADC-Wert eines IR-Sensors in Distanz (mm)
 * 
 * Formel: distance_mm = (x / (ADC_value - y)) * 1000.0
 * 
 * @param Index IR-Sensor-Kanal (0-4)
 * @return Distanz in mm
 */
int16_t CalibrateIRSensors(int16_t Index) {
    // Prüfe gültigen Bereich
    if (Index < 0 || Index > 4) {
        return 0; // Ungültiger Index
    }
    
    // Lese ADC-Wert
    uint16_t adc_value = ADC_getFilteredValue((uint8_t)Index);
    
    // Hole Kalibrierungsparameter
    const sensor_calibration_t* calib = &sensor_calibrations[Index];
    
    // Berechne Distanz: distance_mm = (x / (ADC_value - y)) * 1000.0
    float denominator = (float)adc_value - calib->y;
    
    // Vermeide Division durch Null oder negative Werte
    if (denominator <= 0.0f) {
        return 0; // Ungültiger Wert
    }
    
    float distance_mm = (calib->x / denominator) * 10.0f;
    
    // Konvertiere zu int16_t (begrenze auf sinnvollen Bereich)
    if (distance_mm > 32767.0f) {
        return 32767;
    }
    if (distance_mm < -32768.0f) {
        return -32768;
    }
    
    //communication_log(LEVEL_INFO, "Distance: %d mm for sensor %d", (int)distance_mm, Index);
    return (int16_t)distance_mm;
}
