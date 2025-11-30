#ifndef IR_SENSORS_H
#define IR_SENSORS_H

#include <stdint.h>

/**
 * @file ir_sensors.h
 * @brief Infrarot-Sensor-Modul mit Millivolt-zu-mm Umrechnung
 */

/**
 * @brief Konvertiert ADC-Wert eines IR-Sensors in Distanz (mm)
 * 
 * Verwendet sensor-spezifische Kalibrierungsparameter.
 * 
 * @param Index IR-Sensor-Kanal (0-4: RIGHT_FRONT, RIGHT_BACK, FRONT, LEFT_FRONT, LEFT_BACK)
 * @return Distanz in mm
 */
int16_t CalibrateIRSensors(int16_t Index);

#endif // IR_SENSORS_H
