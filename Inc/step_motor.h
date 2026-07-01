//
// Created by Pobed on 13.05.2026.
//

#ifndef STEP_MOTOR_H
#define STEP_MOTOR_H

#include "main.h"

typedef enum {
    STEP_MOTOR_FAILED = 0U,
    STEP_MOTOR_OK,
    STEP_MOTOR_CACL_REQ,
    STEP_MOTOR_CACL_ACK,
} StepMotor_StatusTypeDef;

typedef struct {
    uint32_t const_psc;
    uint32_t max_arr;
    uint32_t cloсk;
} TIM_InitTypeDef;

typedef struct {
    TIM_TypeDef *instance;
    TIM_InitTypeDef init;
    uint16_t channel;
} TIM_HandlerTypeDef;

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} GPIO_Init_TypeDef;

typedef struct {
    int32_t speed;
    uint64_t distance;
} StepMotor_ParamTypeDef;

typedef struct {
    int32_t speed;
    uint32_t period;
    uint32_t pulse;
    uint64_t distance;
    StepMotor_StatusTypeDef status;
} TIM_CalcTypeDef;

typedef struct {
    TIM_HandlerTypeDef htim_pwm;
    TIM_HandlerTypeDef htim_manager;

    GPIO_Init_TypeDef pul;
    GPIO_Init_TypeDef dir;
    GPIO_Init_TypeDef en;

    volatile StepMotor_ParamTypeDef targ;
    volatile StepMotor_ParamTypeDef cur;

    TIM_CalcTypeDef pwm_param;

    volatile uint16_t speed_min;
    volatile uint16_t speed_step;
    volatile uint32_t accel;
} StepMotor_TypeDef;

typedef struct {
    StepMotor_StatusTypeDef (*Initialization)(StepMotor_TypeDef *motor);
    void (*Enabled)(StepMotor_TypeDef *motor);
    StepMotor_StatusTypeDef (*Disabled)(StepMotor_TypeDef *motor);
    uint32_t (*GetAccel)(StepMotor_TypeDef *motor);
    StepMotor_StatusTypeDef (*SetAccel)(StepMotor_TypeDef *motor, uint32_t acc);
    uint32_t (*GetMinAllowSpeed)(StepMotor_TypeDef *motor);
    StepMotor_StatusTypeDef (*SetMinAllowSpeed)(StepMotor_TypeDef *motor, uint32_t acc);
    int32_t (*GetSpeed)(StepMotor_TypeDef *motor);
    StepMotor_StatusTypeDef (*SetSpeed)(StepMotor_TypeDef *motor, int32_t speed);
    void (*Stop)(StepMotor_TypeDef *motor);
    void (*TIM_PWM_IRQHandler)(StepMotor_TypeDef *motor);
    void (*TIM_Manager_IRQHandler)(StepMotor_TypeDef *motor);
} StepMotor_ItfTypeDef;

const extern StepMotor_ItfTypeDef StepMotor_Interface;

#endif