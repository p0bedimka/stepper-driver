//
// Created by Pobed on 13.05.2026.
//

#include "step_motor.h"

#include <stdlib.h>

/** The frequency of timer interruptions at zero speed */
#define ZERO_SPEED_HZ   1000

TIM_CalcTypeDef TIM_CalcValReg(TIM_HandlerTypeDef *htim, uint32_t freq);
StepMotor_StatusTypeDef TIM_CheckCannel(TIM_HandlerTypeDef *htim);
void TIM_SetSettingsCannel(TIM_HandlerTypeDef *htim);
void TIM_EnableChannel(TIM_HandlerTypeDef *htim);
void TIM_DisableChannel(TIM_HandlerTypeDef *htim);
void TIM_ApplyChannel(TIM_HandlerTypeDef *htim, uint32_t pulse);

TIM_CalcTypeDef TIM_CalcValReg(TIM_HandlerTypeDef *htim, uint32_t freq) {
    TIM_CalcTypeDef calc = {0};
    if (freq == 0) {
        calc.period = htim->init.cloсk / (ZERO_SPEED_HZ * (htim->init.const_psc + 1)) - 1;
        calc.pulse = 0;
    }
    else {
        calc.period = htim->init.cloсk / (freq * (htim->init.const_psc + 1)) - 1;
        calc.pulse = (calc.period + 1) / 2;
    }

    if (calc.period > htim->init.max_arr)
        calc.status = STEP_MOTOR_FAILED;
    else
        calc.status = STEP_MOTOR_OK;
    return calc;
}

void TIM_SetSettingsCannel(TIM_HandlerTypeDef *htim) {
    const uint8_t PWM_MOD = 0b110U;
    switch (htim->channel) {
        case TIM_CHANNEL_1:
            htim->instance->CCMR1 = (htim->instance->CCMR1 & ~TIM_CCMR1_OC1M) | (PWM_MOD << TIM_CCMR1_OC1M_Pos);
            htim->instance->CCMR1 |= TIM_CCMR1_OC1PE;
            break;
        case TIM_CHANNEL_2:
            htim->instance->CCMR1 = (htim->instance->CCMR1 & ~TIM_CCMR1_OC2M) | (PWM_MOD << TIM_CCMR1_OC2M_Pos);
            htim->instance->CCMR1 |= TIM_CCMR1_OC2PE;
            break;
        case TIM_CHANNEL_3:
            htim->instance->CCMR2 = (htim->instance->CCMR2 & ~TIM_CCMR2_OC3M) | (PWM_MOD << TIM_CCMR2_OC3M_Pos);
            htim->instance->CCMR2 |= TIM_CCMR2_OC3PE;
            break;
        case TIM_CHANNEL_4:
            htim->instance->CCMR2 = (htim->instance->CCMR2 & ~TIM_CCMR2_OC4M) | (PWM_MOD << TIM_CCMR2_OC4M_Pos);
            htim->instance->CCMR2 |= TIM_CCMR2_OC4PE;
            break;
        default:
            break;
    };
}

StepMotor_StatusTypeDef TIM_CheckCannel(TIM_HandlerTypeDef *htim) {
    StepMotor_StatusTypeDef status;
    switch (htim->channel) {
        case TIM_CHANNEL_1:
            status = STEP_MOTOR_OK;
            break;
        case TIM_CHANNEL_2:
            status = STEP_MOTOR_OK;
            break;
        case TIM_CHANNEL_3:
            status = STEP_MOTOR_OK;
            break;
        case TIM_CHANNEL_4:
            status = STEP_MOTOR_OK;
            break;
        default:
            status = STEP_MOTOR_FAILED;
            break;
    }
    return status;
}

void TIM_EnableChannel(TIM_HandlerTypeDef *htim) {
    switch (htim->channel) {
        case TIM_CHANNEL_1:
            htim->instance->CCER |= TIM_CCER_CC1E;
            break;
        case TIM_CHANNEL_2:
            htim->instance->CCER |= TIM_CCER_CC2E;
            break;
        case TIM_CHANNEL_3:
            htim->instance->CCER |= TIM_CCER_CC3E;
            break;
        case TIM_CHANNEL_4:
            htim->instance->CCER |= TIM_CCER_CC4E;
            break;
        default:
            break;
    }
}

void TIM_DisableChannel(TIM_HandlerTypeDef *htim) {
    switch (htim->channel) {
        case TIM_CHANNEL_1:
            htim->instance->CCER &= ~TIM_CCER_CC1E;
            break;
        case TIM_CHANNEL_2:
            htim->instance->CCER &= ~TIM_CCER_CC2E;
            break;
        case TIM_CHANNEL_3:
            htim->instance->CCER &= ~TIM_CCER_CC3E;
            break;
        case TIM_CHANNEL_4:
            htim->instance->CCER &= ~TIM_CCER_CC4E;
            break;
        default:
            break;
    };
}

void TIM_ApplyChannel(TIM_HandlerTypeDef *htim, uint32_t pulse) {
    switch (htim->channel) {
        case TIM_CHANNEL_1:
            htim->instance->CCR1 = pulse;
            break;
        case TIM_CHANNEL_2:
            htim->instance->CCR2 = pulse;
            break;
        case TIM_CHANNEL_3:
            htim->instance->CCR3 = pulse;
            break;
        case TIM_CHANNEL_4:
            htim->instance->CCR4 = pulse;
            break;
        default:
            break;
    }
}

StepMotor_StatusTypeDef StepMotor_Initialization(StepMotor_TypeDef *motor);
void StepMotor_Enabled(StepMotor_TypeDef *motor);
StepMotor_StatusTypeDef StepMotor_Disabled(StepMotor_TypeDef *motor);
uint32_t StepMotor_GetAccel(StepMotor_TypeDef *motor);
StepMotor_StatusTypeDef StepMotor_SetAccel(StepMotor_TypeDef *motor, uint32_t acc);
int32_t StepMotor_GetSpeed(StepMotor_TypeDef *motor);
StepMotor_StatusTypeDef StepMotor_SetSpeed(StepMotor_TypeDef *motor, int32_t speed);
void StepMotor_Stop(StepMotor_TypeDef *motor);
void StepMotor_TIMx_PWM_IRQHandler(StepMotor_TypeDef *motor);
void StepMotor_TIMx_Manager_IRQHandler(StepMotor_TypeDef *motor);

StepMotor_StatusTypeDef StepMotor_Initialization(StepMotor_TypeDef *motor) {
    motor->dir.port->BSRR = motor->dir.pin << 16;
    motor->en.port->BSRR = motor->en.pin << 16;

    motor->targ.speed = 0;
    motor->targ.distance = 0;

    motor->cur.speed = 0;
    motor->cur.distance = 0;

    motor->pwm_param.speed = 0;
    motor->pwm_param.distance = 0;
    motor->pwm_param.period = 0;
    motor->pwm_param.pulse = 0;

    /** ----- Start of installation acceleration settings ----- */
    motor->accel = 1000;
    motor->speed_min = 25;
    // speed_update_timeout = 1000 / speed.min;
    motor->speed_step = motor->accel / motor->speed_min;

    TIM_CalcTypeDef calc = TIM_CalcValReg(&motor->htim_manager, motor->speed_min * 4);
    if (calc.status != STEP_MOTOR_OK)
        return STEP_MOTOR_FAILED;

    motor->htim_manager.instance->CR1 &= ~TIM_CR1_CEN;
    motor->htim_manager.instance->PSC = motor->htim_manager.init.const_psc;
    motor->htim_manager.instance->ARR = calc.period;
    motor->htim_manager.instance->CNT = 0;

    motor->htim_manager.instance->CR1 &= ~TIM_CR1_DIR;
    motor->htim_manager.instance->DIER |= TIM_DIER_UIE;
    motor->htim_manager.instance->CR1 |= TIM_CR1_ARPE;
    motor->htim_manager.instance->CR1 |= TIM_CR1_CEN;
    /** ----- Stop of installation acceleration settings ----- */

    calc = TIM_CalcValReg(&motor->htim_pwm, 0);
    if (calc.status != STEP_MOTOR_OK)
        return STEP_MOTOR_FAILED;

    if (TIM_CheckCannel(&motor->htim_pwm) != STEP_MOTOR_OK)
        return STEP_MOTOR_FAILED;

    motor->htim_pwm.instance->CR1 &= ~TIM_CR1_CEN;
    motor->htim_pwm.instance->PSC = motor->htim_pwm.init.const_psc;
    motor->htim_pwm.instance->ARR = calc.period;
    motor->htim_pwm.instance->CNT = 0;

    TIM_DisableChannel(&motor->htim_pwm);
    TIM_SetSettingsCannel(&motor->htim_pwm);
    TIM_ApplyChannel(&motor->htim_pwm, 0);

    motor->htim_pwm.instance->CR1 &= ~TIM_CR1_DIR;
    motor->htim_pwm.instance->DIER |= TIM_DIER_UIE;
    motor->htim_pwm.instance->CR1 |= TIM_CR1_ARPE;
    return STEP_MOTOR_OK;
}

void StepMotor_Enabled(StepMotor_TypeDef *motor) {
    motor->en.port->BSRR = motor->en.pin;
    TIM_EnableChannel(&motor->htim_pwm);
    motor->htim_pwm.instance->CR1 |= TIM_CR1_CEN;
}

StepMotor_StatusTypeDef StepMotor_Disabled(StepMotor_TypeDef *motor) {
    motor->htim_pwm.instance->CR1 &= ~TIM_CR1_CEN;
    TIM_DisableChannel(&motor->htim_pwm);
    motor->en.port->BSRR = motor->en.pin << 16;
    motor->dir.port->BSRR = motor->dir.pin << 16;
    return STEP_MOTOR_OK;
}

uint32_t StepMotor_GetAccel(StepMotor_TypeDef *motor) {
    return motor->accel;
}

StepMotor_StatusTypeDef StepMotor_SetAccel(StepMotor_TypeDef *motor, uint32_t acc) {
    StepMotor_StatusTypeDef status = STEP_MOTOR_OK;
    if (acc > 0) {
        motor->accel = acc;
        motor->speed_step = motor->accel / motor->speed_min;
    }
    else {
        status = STEP_MOTOR_FAILED;
    }
    return status;
}

int32_t StepMotor_GetSpeed(StepMotor_TypeDef *motor) {
    return motor->cur.speed;
}

StepMotor_StatusTypeDef StepMotor_SetSpeed(StepMotor_TypeDef *motor, int32_t speed) {
    if (speed != 0 && abs(speed) < motor->speed_min)
        speed = speed > 0 ? motor->speed_min : -motor->speed_min;

    const TIM_CalcTypeDef calc = TIM_CalcValReg(&motor->htim_pwm, abs(speed));
    if (calc.status != STEP_MOTOR_OK)
        return STEP_MOTOR_FAILED;

    motor->targ.speed = speed;
    return STEP_MOTOR_OK;
}

void StepMotor_Stop(StepMotor_TypeDef *motor) {
    StepMotor_SetSpeed(motor, 0);
}

void TIM_CalcParamPWM(StepMotor_TypeDef *motor) {
    /** Сalculation of velocities relative to acceleration **/
    int32_t speed, diff_speed = motor->targ.speed - motor->cur.speed;

    if (abs(diff_speed) > motor->speed_step) {
        if (diff_speed > 0)
            speed = motor->cur.speed + motor->speed_step;
        else
            speed = motor->cur.speed - motor->speed_step;
    }
    else {
        speed = motor->targ.speed;
    }

    if (motor->cur.speed != 0 && speed != 0 && (motor->cur.speed ^ speed) < 0)
        speed = 0;

    if (speed != 0 &&  motor->speed_min > abs(speed)) {
        if (abs(speed) > motor->speed_step)
            speed = speed > 0 ? motor->speed_min : -motor->speed_min;
        else
            speed = 0;
    }

    /** Calculation of timer registers for setting a new speed **/
    uint32_t period, pulse;
    uint32_t distance;

    if (speed == 0) {
        period = motor->htim_pwm.init.cloсk / (ZERO_SPEED_HZ * (motor->htim_pwm.init.const_psc + 1)) - 1;
        pulse = 0;
        distance = (ZERO_SPEED_HZ / motor->speed_min);
    }
    else {
        period = motor->htim_pwm.init.cloсk / (abs(speed) * (motor->htim_pwm.init.const_psc + 1)) - 1;
        pulse = (period + 1) / 2;
        distance = (abs(speed) / motor->speed_min);
    }

    if (period < motor->htim_pwm.init.max_arr) {
        motor->pwm_param.speed = speed;
        motor->pwm_param.period = period;
        motor->pwm_param.pulse = pulse;
        motor->pwm_param.distance = distance;
    }
}

void StepMotor_TIMx_PWM_IRQHandler(StepMotor_TypeDef *motor) {
    if (motor->htim_pwm.instance->SR & TIM_SR_UIF) {
        motor->htim_pwm.instance->SR &= ~TIM_SR_UIF;

        if (motor->cur.distance == 0) {
            if (motor->cur.speed > 0)
                motor->dir.port->BSRR = motor->dir.pin;
            else
                motor->dir.port->BSRR = motor->dir.pin << 16;
        }

        motor->cur.distance++;
        if (motor->cur.speed != motor->targ.speed && motor->cur.distance > motor->targ.distance) {
            if (motor->pwm_param.status != STEP_MOTOR_CACL_ACK)
                TIM_CalcParamPWM(motor);

            /** Setting timer registers for speed updates **/
            motor->htim_pwm.instance->ARR = motor->pwm_param.period;
            TIM_ApplyChannel(&motor->htim_pwm, motor->pwm_param.pulse);

            motor->cur.speed = motor->pwm_param.speed;
            motor->cur.distance = 0;
            motor->targ.distance = motor->pwm_param.distance;

            motor->pwm_param.status = STEP_MOTOR_CACL_REQ;
        }
    }
}

void StepMotor_TIMx_Manager_IRQHandler(StepMotor_TypeDef *motor) {
    if (motor->htim_manager.instance->SR & TIM_SR_UIF) {
        motor->htim_manager.instance->SR &= ~TIM_SR_UIF;

        if (motor->cur.speed != motor->targ.speed && motor->pwm_param.status == STEP_MOTOR_CACL_REQ) {
            TIM_CalcParamPWM(motor);
            motor->pwm_param.status = STEP_MOTOR_CACL_ACK;
        }
    }
}

const StepMotor_ItfTypeDef StepMotor_Interface = {
    .Initialization = StepMotor_Initialization,
    .Enabled = StepMotor_Enabled,
    .Disabled = StepMotor_Disabled,
    .GetSpeed = StepMotor_GetSpeed,
    .SetSpeed = StepMotor_SetSpeed,
    .Stop = StepMotor_Stop,
    .TIM_PWM_IRQHandler = StepMotor_TIMx_PWM_IRQHandler,
    .TIM_Manager_IRQHandler = StepMotor_TIMx_Manager_IRQHandler,
};
