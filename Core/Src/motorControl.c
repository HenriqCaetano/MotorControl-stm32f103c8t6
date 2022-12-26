#include "motorControl.h"
#include "tim.h"

/*============ENCODER FUNCTIONS============*/ 

void calculate_encoder(Encoder_Struct* encoder)
{
	encoder->cnt_current = encoder->timer->Instance->CNT;

    if(encoder->cnt_current == encoder->cnt_prev){ // motor stop
        encoder->step = 0; // update step
        encoder->dir = DIR_STOP; // update dir 
        encoder->w_speed = 0;
    }
  else{ // motor in running
    if(ABS(encoder->cnt_prev - ENC_DSTEP_MAX) > ENC_DSTEP_MAX){ //overflow case: |current-prev| e.g. rw: |65534-320| or fw: |4000-63200|
      if(encoder->cnt_current > encoder->cnt_prev) //reverse
        encoder->step = -1*(encoder->cnt_prev + (65535 - encoder->cnt_current));
      else //forward
        encoder->step = (65535 - encoder->cnt_prev) + encoder->cnt_current;
    }
    else{ // general case
      encoder->step = encoder->cnt_current - encoder->cnt_prev;
      encoder->dir = (encoder->step > 0) ? DIR_FORWARD : DIR_REVERSE; //update dir
      encoder->w_speed = (((float)encoder->step / ENT_CNT_PER_REV) / ENC_DT) * 60.0f;// * MOTOR_GEARRATIO; //angular speed in RPM
    }
  }
  encoder->cnt_prev = encoder->cnt_current; //update cnt_prev counter.
}

float getEncoderSpeed(Encoder_Struct *const encoder)
{
    return encoder->w_speed;
}


/*============ DRIVER FUNCTIONS ============*/ 

void drv_set_dutycycle(float duty, DRV_Struct* driver)
{
  driver->dir = (duty > duty_MIN) ? DIR_FORWARD : (duty < -duty_MIN) ? DIR_REVERSE : DIR_STOP; //Experiment: ABS(duty) > ABS(duty_min) = 14.2%// 
  driver->pwm_ccr = ABS(duty) * DRV_PWM_CCRMAX / 100.0f;

  switch (driver->dir)
  {
    case DIR_FORWARD:
      *driver->tim_pwm_fw = (uint32_t)driver->pwm_ccr;
      *driver->tim_pwm_rw = 0;
      break;
    case DIR_REVERSE:
      *driver->tim_pwm_fw = 0;
      *driver->tim_pwm_rw = (uint32_t)driver->pwm_ccr;
      break;
    case DIR_STOP:
      *driver->tim_pwm_fw = 0;
      *driver->tim_pwm_rw = 0;
      break;
  }
}

void drv_set_enable_toggle(DRV_Struct* driver, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
  driver->enable = !driver->enable;
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, (GPIO_PinState)driver->enable);
}

void drv_set_enable(DRV_Struct *const driver, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}

void drv_set_disable(DRV_Struct *const driver, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}


/*============PI CONTROLLER FUNCTIONS============*/ 
void PIController_Init(PIController *pi)
{
  // resets controller variables
	pi->integrator = 0.0f;
	pi->prevError  = 0.0f;

	pi->out = 0.0f;
}

float PIController_Update(PIController *pi, float setpoint)
{
   calculate_encoder(pi->encoder);

  float error = setpoint - pi->encoder->w_speed;
  float proportional = pi->Kp * error;

  pi->integrator +=  0.5f * pi->Ki * pi->T * (error + pi->prevError);
  //pi->integrator += pi->T * error; QUAL VERSÃO USAR?


	// Anti-wind-up for the integrator
  if (pi->integrator > pi->limMaxInt) 
      pi->integrator = pi->limMaxInt;
  else if (pi->integrator < pi->limMinInt) 
      pi->integrator = pi->limMinInt;
  

  //pi output
  pi->out = proportional + pi->integrator;
  //pi->out = proportional + pi->ki * pi->integrator; QUAL VERSÃO USAR?

  //anti-wind up for the output
  if (pi->out > pi->limMax) {
      pi->out = pi->limMax;
    } 
  else if (pi->out < pi->limMin) {
      pi->out = pi->limMin;
  }

  //updates for next iteration
  pi->prevError = error;

  return pi->out;
}

//TODO: REBUILD MATH
uint8_t convertsPiOutputToDriverInput(const PIController* pid){
	float output = pid->out;
	uint8_t outCome = 0x00;
	if(output > 0 ){
		outCome = (output/pid->limMax) * 127;
	}
	else if(output < 0){
		outCome = (output/pid->limMin) * 127;
	}
	return outCome;
}
