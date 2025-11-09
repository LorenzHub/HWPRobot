/**
 * @file motor.h
 * @ingroup motor
 *
 * Motor functions
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>


/**
 * Initialize the motor library.
 *
 * Timer 3 (for motor A) and timer 4 (for motor B) are used to generate PWM signals
 * with which the motors are driven through the motor driver TI DRV8833.
 * Phase correct PWM mode is used with a frequency of 488.759Hz.
 */
void Motor_init(void);


/**
 * Set duty cycle of the PWM signal for motor A.
 *
 * A maximum absolute value of 8191 corresponds to a duty cycle of 100%.
 *
 * The slow-decay mode (see datasheet of TI DRV8833) of the motor driver is used
 * since this provides a quite linear relationship between PWM duty cycle
 * and motor speed (under constant supply voltage and constant load conditions).
 *
 * If enabled, interrupts are disabled during this function and their state
 * restored afterwards.
 *
 * Note that due to the phase correct PWM mode, an updated duty cycle is
 * taken over by the PWM generator only once during a period which has a duration
 * of 1/488.759Hz = 2.046ms. This constitutes an upper limit on how often new
 * duty cycles should be set.
 *
 * @param   pwm   a value in [-8191...+8191] determining direction of motor (sign)
 *                and duty cycle of PWM signal (absolute value)
 */
void Motor_setPWM_A(const int16_t pwm);


/**
 * Set duty cycle of the PWM signal for motor B.
 *
 * A maximum absolute value of 8191 corresponds to a duty cycle of 100%.
 *
 * The slow-decay mode (see datasheet of TI DRV8833) of the motor driver is used
 * since this provides a quite linear relationship between PWM duty cycle
 * and motor speed (under constant supply voltage and constant load conditions).
 *
 * If enabled, interrupts are disabled during this function and their state
 * restored afterwards.
 *
 * Note that due to the phase correct PWM mode, an updated duty cycle is
 * taken over by the PWM generator only once during a period which has a duration
 * of 1/488.759Hz = 2.046ms. This constitutes an upper limit on how often new
 * duty cycles should be set.
 *
 * @param   pwm   a value in [-8191...+8191] determining direction of motor (sign)
 *                and duty cycle of PWM signal (absolute value)
 */
void Motor_setPWM_B(const int16_t pwm);


/**
 * Set duty cycles of the PWM signals for motors A and B.
 *
 * This function should be used in preference to Motor_setPWM_A() and
 * Motor_setPWM_B() when setting the duty cycles for both motors.
 *
 * A maximum absolute value of 8191 corresponds to a duty cycle of 100%.
 *
 * The slow-decay mode (see datasheet of TI DRV8833) of the motor driver is used
 * since this provides a quite linear relationship between PWM duty cycle
 * and motor speed (under constant supply voltage and constant load conditions).
 *
 * If enabled, interrupts are disabled during this function and their state
 * restored afterwards. This ensures that both duty cycles are set almost
 * simultaneously.
 *
 * Note that due to the phase correct PWM mode, an updated duty cycle is
 * taken over by the PWM generator only once during a period which has a duration
 * of 1/488.759Hz = 2.046ms. This constitutes an upper limit on how often new
 * duty cycles should be set.
 *
 * @param   pwmA   a value in [-8191...+8191] determining direction of motor (sign)
 *                 and duty cycle of PWM signal (absolute value) of motor A
 * @param   pwmB   a value in [-8191...+8191] determining direction of motor (sign)
 *                 and duty cycle of PWM signal (absolute value) of motor B
*/
void Motor_setPWM(const int16_t pwmA, const int16_t pwmB);


/**
 * Stop motor A.
 *
 * This function should be used in preference to Motor_setPWM_A(0).
 *
 * Note that Motor_setPWM_A(0) is not equivalent to Motor_stopA().
 * The former uses the slow-decay mode (breaking), leaving the outputs of the
 * motor driver low. The latter uses the fast-decay-mode (coasting) in which
 * the outputs of the motor driver become high-impedant.
 *
 * If enabled, interrupts are disabled during this function and their state
 * restored afterwards.
 */
void Motor_stopA(void);


/**
 * Stop motor B.
 *
 * This function should be used in preference to Motor_setPWM_B(0).
 *
 * Note that Motor_setPWM_B(0) is not equivalent to Motor_stopB().
 * The former uses the slow-decay mode (breaking), leaving the outputs of the
 * motor driver low. The latter uses the fast-decay-mode (coasting) in which
 * the outputs of the motor driver become high-impedant.
 *
 * If enabled, interrupts are disabled during this function and their state
 * restored afterwards.
 */
void Motor_stopB(void);


/**
 * Stop motors A and B.
 *
 * This function should be used in preference to Motor_stopA() and
 * Motor_stopB() when stopping both motors simultaneously.
 *
 * Note that Motor_setPWM(0,0) is not equivalent to Motor_stopAll().
 * The former uses the slow-decay mode (breaking), leaving the outputs of the
 * motor driver low. The latter uses the fast-decay-mode (coasting) in which
 * the outputs of the motor driver become high-impedant.
 *
 * If enabled, interrupts are disabled during this function and their state
 * restored afterwards. This ensures that both motors are stopped almost
 * simultaneously.
 */
void Motor_stopAll(void);


#endif /* MOTOR_H_ */
