#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

/*Codes developed by Henrique Caetano to control a DC motor*/
/*Using an encoder, a MD22 driver and a STM32*/

/*  This library contains:
        encoder support (to read the current speed)
        driver support (to control the motor with PWM)
        PI controller support (to obtain a reliable speed control)
*/

#include <stdint.h>
#include "tim.h"

//custom absolute function
#define ABS(x)  ((x) > 0 ? (x) : -(x))

#define CNT_DIR_CW 1
#define CNT_DIR_CCW -1


/*========== ENCODER  FEATURES ==========*/
#define ENC_DT 0.02 // 20 ms (50 Hz) -> updates encoder 50 times per second
#define ENC_DSTEP_MAX 30000
#define CTRL_DT 0.01
#define ENT_CNT_PER_REV 5000 // Encoder counts/rev

//motor features
#define MOTOR_GEARRATIO (1/48.0f) // Motor Gear_Ratio gearbox

/*MOTOR DRIVE FEATURES*/
#define DRV_PWM_CCRMAX 3359
#define duty_MIN 14.2


typedef enum
{
  DIR_REVERSE = -1,
  DIR_STOP = 0,
  DIR_FORWARD = 1
} Dir_1D_Enum;


/*============ ENCODER STRUCTURE ============*/ 
typedef struct{

	TIM_HandleTypeDef* timer;
	uint16_t cnt_current; //current encoder counter
	uint16_t cnt_prev; //previous encoder counter
	Dir_1D_Enum dir; //direction of movement
	int16_t step; // limit ENC_DSTEP_MAX < 30000 which (int16_t) = -32767 ~ 32767 so it could store dstep safely without overflow na ja!
	float w_speed; //angular speed in RPM
}Encoder_Struct;

/*============ENCODER FUNCTIONS============*/ 

/// @brief calculates the current angular speed in RPM
/// @param encoder
/// @param htim timer in encoder mode
void calculate_encoder(Encoder_Struct* encoder);


/// @brief speed in RPM
/// @param encoder 
/// @return current angular speed
float getEncoderSpeed(Encoder_Struct* const encoder);


/*============ DRIVER STRUCTURE ============*/ 
typedef struct{
  uint32_t volatile *const tim_pwm_fw; //store address of TIM3->CCR3 at initialize
  uint32_t volatile *const tim_pwm_rw; //store address of TIM3->CCR4 at initialize
  uint8_t enable;
  Dir_1D_Enum dir;
  float duty_prev;
  float pwm_ccr;
}DRV_Struct;

/*============DRIVER FUNCTIONS============*/ 

/// @brief 
/// @param duty 
/// @param driver
void drv_set_dutycycle(float duty, DRV_Struct* driver);

/// @brief 
/// @param driver
/// @param GPIOx 
/// @param GPIO_Pin 
void drv_set_enable_toggle(DRV_Struct *const driver, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

/// @brief 
/// @param driver
/// @param GPIOx 
/// @param GPIO_Pin 
void drv_set_enable(DRV_Struct *const driver, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

/// @brief 
/// @param driver
/// @param GPIOx 
/// @param GPIO_Pin 
void drv_set_disable(DRV_Struct *const driver, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);


/*============ PI STRUCTURE ============*/
typedef struct {
	//encoder linked to the PI
	Encoder_Struct* encoder;

	/* Controller gains */
	float Kp;
	float Ki;

	/* Output limits */
	float limMin;
	float limMax;
	
	/* Integrator limits - avoid overshooting*/
	float limMinInt;
	float limMaxInt;

	/* Sample time (in seconds) */
	float T;

	/* Controller "memory"  no need to instantiate with the variable*/
	float integrator;
	float prevError;			/* Required for integrator */

	/* Controller output - a speed*/
	float out;
} PIController;

/*============PI FUNCTIONS============*/ 

/// @brief resets controller cycle
/// @param pid 
void  PIController_Init(PIController *pid);


/// @brief 
/// @param pid the controller variable reference
/// @param setpoint target value of the controlled physical quantity
/// @param measurement current value of the controlled physical quantity
/// @return an amount of the controlled physical quantity
float PIController_Update(PIController *pid, float setpoint);

uint8_t convertsPiOutputToDriverInput(const PIController* pid);



#endif
