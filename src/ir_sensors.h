#ifndef IR_SENSORS_H_
#define IR_SENSORS_H_

#include <stdint.h>
#include <stdbool.h>

/**
 * Initialize the IR sensor module.
 */
void ir_sensors_init(void);

/**
 * Get the distance in millimeters from a specific IR sensor.
 * Uses a lookup table for calibration based on the specific sensor characteristics.
 * 
 * @param sensor_index Index of the sensor (0-4)
 * @return Distance in mm. Returns 0xFFFF if distance is out of valid range (too far) 
 *         or 0 if too close/invalid.
 */
uint16_t ir_sensor_get_distance_mm(uint8_t sensor_index);

/**
 * Check if a wall is detected by a specific sensor.
 * 
 * @param sensor_index Index of the sensor (0-4)
 * @return true if a wall is detected within the threshold distance (e.g. < 50mm + hysteresis)
 */
bool ir_sensor_detects_wall(uint8_t sensor_index);

/**
 * Get the raw filtered ADC value for a sensor.
 * 
 * @param sensor_index Index of the sensor (0-4)
 * @return Raw ADC value (0-1023)
 */
uint16_t ir_sensor_get_raw(uint8_t sensor_index);

#endif /* IR_SENSORS_H_ */

