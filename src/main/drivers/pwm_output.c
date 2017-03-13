/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "common/maths.h"

#include "io.h"
#include "timer.h"
#include "pwm_output.h"

static pwmWriteFuncPtr pwmWritePtr;
static pwmOutputPort_t motors[MAX_SUPPORTED_MOTORS];
static pwmCompleteWriteFuncPtr pwmCompleteWritePtr = NULL;

static uint32_t motorTickOffset;
static uint32_t motorTickRange;

#ifdef USE_SERVOS
static pwmOutputPort_t servos[MAX_SUPPORTED_SERVOS];
#endif

bool pwmMotorsEnabled = false;

static void pwmOCConfig(TIM_TypeDef *tim, uint8_t channel, uint32_t value, uint8_t output)
{
#if defined(USE_HAL_DRIVER)
    TIM_HandleTypeDef* Handle = timerFindTimerHandle(tim);
    if(Handle == NULL) return;

    TIM_OC_InitTypeDef TIM_OCInitStructure;

    TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;

    if (output & TIMER_OUTPUT_N_CHANNEL) {
        TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_RESET;
        TIM_OCInitStructure.OCNPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCNPOLARITY_HIGH : TIM_OCNPOLARITY_LOW;
    } else {
        TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_SET;
        TIM_OCInitStructure.OCPolarity =  (output & TIMER_OUTPUT_INVERTED) ? TIM_OCPOLARITY_LOW : TIM_OCPOLARITY_HIGH;
    }

    TIM_OCInitStructure.Pulse = value;
    TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(Handle, &TIM_OCInitStructure, channel);
#else
    TIM_OCInitTypeDef TIM_OCInitStructure;

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;

    if (output & TIMER_OUTPUT_N_CHANNEL) {
        TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
        TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
        TIM_OCInitStructure.TIM_OCNPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCNPolarity_High : TIM_OCNPolarity_Low;
    } else {
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
        TIM_OCInitStructure.TIM_OCPolarity =  (output & TIMER_OUTPUT_INVERTED) ? TIM_OCPolarity_Low : TIM_OCPolarity_High;
    }
    TIM_OCInitStructure.TIM_Pulse = value;

    timerOCInit(tim, channel, &TIM_OCInitStructure);
    timerOCPreloadConfig(tim, channel, TIM_OCPreload_Enable);
#endif
}

static void pwmOutConfig(pwmOutputPort_t *port, const timerHardware_t *timerHardware, uint32_t hz, uint16_t period, uint32_t value)
{
#if defined(USE_HAL_DRIVER)
    TIM_HandleTypeDef* Handle = timerFindTimerHandle(timerHardware->tim);
    if(Handle == NULL) return;
#endif

    configTimeBaseHz(timerHardware->tim, period, hz);
    pwmOCConfig(timerHardware->tim, timerHardware->channel, value, timerHardware->output);

#if defined(USE_HAL_DRIVER)
    HAL_TIM_PWM_Start(Handle, timerHardware->channel);
    HAL_TIM_Base_Start(Handle);
#else
    TIM_CtrlPWMOutputs(timerHardware->tim, ENABLE);
    TIM_Cmd(timerHardware->tim, ENABLE);
#endif

    port->ccr = timerChCCR(timerHardware);
    port->period = period;
    port->tim = timerHardware->tim;

    *port->ccr = 0;
}

static void pwmWriteUnused(uint8_t index, float value)
{
    UNUSED(index);
    UNUSED(value);
}

uint32_t calculateMotorTick(float value) {
    return lrintf(((value - PWM_RANGE_US) / PWM_RANGE_US) * motorTickRange) + motorTickOffset;
}

static void pwmWriteStandard(uint8_t index, float value)
{
    *motors[index].ccr = lrintf(value);
}

static void pwmWriteBrushed(uint8_t index, float value)
{
    *motors[index].ccr = lrintf((value - PWM_RANGE_US) * motors[index].period / PWM_RANGE_US);
}

static void pwmWriteFastPwm(uint8_t index, float value)
{
    *motors[index].ccr = calculateMotorTick(value);
}

void pwmWriteMotor(uint8_t index, float value)
{
    pwmWritePtr(index, value);
}

void pwmShutdownPulsesForAllMotors(uint8_t motorCount)
{
    for (int index = 0; index < motorCount; index++) {
        // Set the compare register to 0, which stops the output pulsing if the timer overflows
        if (motors[index].ccr) {
            *motors[index].ccr = 0;
        }
    }
}

void pwmDisableMotors(void)
{
    pwmShutdownPulsesForAllMotors(MAX_SUPPORTED_MOTORS);
    pwmMotorsEnabled = false;
}

void pwmEnableMotors(void)
{
    /* check motors can be enabled */
    pwmMotorsEnabled = (pwmWritePtr != pwmWriteUnused);
}

bool pwmAreMotorsEnabled(void)
{
    return pwmMotorsEnabled;
}

static void pwmCompleteWriteUnused(uint8_t motorCount)
{
    UNUSED(motorCount);
}

static void pwmCompleteOneshotMotorUpdate(uint8_t motorCount)
{
    for (int index = 0; index < motorCount; index++) {
        if (motors[index].forceOverflow) {
            timerForceOverflow(motors[index].tim);
        }
        // Set the compare register to 0, which stops the output pulsing if the timer overflows before the main loop completes again.
        // This compare register will be set to the output value on the next main loop.
        *motors[index].ccr = 0;
    }
}

void pwmCompleteMotorUpdate(uint8_t motorCount)
{
    pwmCompleteWritePtr(motorCount);
}

uint8_t motorTimerDivisor;

uint32_t calculateTimerHz(float maxValue) {
    motorTimerDivisor = ceil((float)SystemCoreClock / (0xffffffff / maxValue));
    if (SystemCoreClock > 100000000)                       // Divider of 1 showed issues on higher frequency timers
        motorTimerDivisor = constrain(motorTimerDivisor, 2, motorTimerDivisor);
    if ((SystemCoreClock / motorTimerDivisor) == 48000000) // FIXME - 48Mhz seemed problematic somehow, while in theory it should work. Stepping down seems to work
        motorTimerDivisor += 1;
    return SystemCoreClock / motorTimerDivisor;
}

void motorInit(const motorConfig_t *motorConfig, uint16_t idlePulse, uint8_t motorCount)
{
    memset(motors, 0, sizeof(motors));
    
    uint32_t timerHz = 0;
    bool useUnsyncedPwm = motorConfig->useUnsyncedPwm;
    bool isDigital = false;
    uint32_t idleValue = 0;

    switch (motorConfig->motorPwmProtocol) {
    default:
    case PWM_TYPE_ONESHOT125:
        timerHz = calculateTimerHz(ONESHOT125_OFFSET_US + ONESHOT125_RANGE_US);
        pwmWritePtr = pwmWriteFastPwm;
        motorTickOffset = ONESHOT125_OFFSET_US * timerHz / US_SCALE;
        motorTickRange = lrintf( ((float)ONESHOT125_RANGE_US / US_SCALE) / (1.0f / timerHz) );
        idleValue = calculateMotorTick(idlePulse);
        break;
    case PWM_TYPE_ONESHOT42:
        timerHz = calculateTimerHz(ONESHOT42_OFFSET_US + ONESHOT42_RANGE_US);
        pwmWritePtr = pwmWriteFastPwm;
        motorTickOffset = ONESHOT42_OFFSET_US * timerHz / US_SCALE;
        motorTickRange = lrintf( ((float)ONESHOT42_RANGE_US / US_SCALE) / (1.0f / timerHz) );
        idleValue = calculateMotorTick(idlePulse);
        break;
    case PWM_TYPE_MULTISHOT:
        timerHz = calculateTimerHz(MULTISHOT_OFFSET_US + MULTISHOT_RANGE_US);
        pwmWritePtr = pwmWriteFastPwm;
        motorTickOffset = MULTISHOT_OFFSET_US * timerHz / US_SCALE;
        motorTickRange = lrintf( ((float)MULTISHOT_RANGE_US / US_SCALE) / (1.0f / timerHz) );
        idleValue = calculateMotorTick(idlePulse);
        break;
    case PWM_TYPE_BRUSHED:
        timerHz = calculateTimerHz(PWM_RANGE_US);
        pwmWritePtr = pwmWriteBrushed;
        useUnsyncedPwm = true;
        break;
    case PWM_TYPE_STANDARD:
        timerHz = PWM_TIMER_MHZ * 1000000;
        pwmWritePtr = pwmWriteStandard;
        useUnsyncedPwm = true;
        break;
#ifdef USE_DSHOT
    case PWM_TYPE_DSHOT1200:
    case PWM_TYPE_DSHOT600:
    case PWM_TYPE_DSHOT300:
    case PWM_TYPE_DSHOT150:
        pwmWritePtr = pwmWriteDigital;
        pwmCompleteWritePtr = pwmCompleteDigitalMotorUpdate;
        isDigital = true;
        break;
#endif
    }

    if (!isDigital) {
        pwmCompleteWritePtr = useUnsyncedPwm ? pwmCompleteWriteUnused : pwmCompleteOneshotMotorUpdate;
    }

    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < motorCount; motorIndex++) {
        const ioTag_t tag = motorConfig->ioTags[motorIndex];
        const timerHardware_t *timerHardware = timerGetByTag(tag, TIM_USE_ANY);

        if (timerHardware == NULL) {
            /* not enough motors initialised for the mixer or a break in the motors */
            pwmWritePtr = pwmWriteUnused;
            pwmCompleteWritePtr = pwmCompleteWriteUnused;
            /* TODO: block arming and add reason system cannot arm */
            return;
        }

        motors[motorIndex].io = IOGetByTag(tag);

#ifdef USE_DSHOT
        if (isDigital) {
            pwmDigitalMotorHardwareConfig(timerHardware, motorIndex, motorConfig->motorPwmProtocol);
            motors[motorIndex].enabled = true;
            continue;
        }
#endif

        IOInit(motors[motorIndex].io, OWNER_MOTOR, RESOURCE_INDEX(motorIndex));
#if defined(USE_HAL_DRIVER)
        IOConfigGPIOAF(motors[motorIndex].io, IOCFG_AF_PP, timerHardware->alternateFunction);
#else
        IOConfigGPIO(motors[motorIndex].io, IOCFG_AF_PP);
#endif

        if (useUnsyncedPwm) {
            pwmOutConfig(&motors[motorIndex], timerHardware, timerHz, timerHz / motorConfig->motorPwmRate, idleValue);
        } else {
            pwmOutConfig(&motors[motorIndex], timerHardware, timerHz, 0xFFFF, 0);
        }

        bool timerAlreadyUsed = false;
        for (int i = 0; i < motorIndex; i++) {
            if (motors[i].tim == motors[motorIndex].tim) {
                timerAlreadyUsed = true;
                break;
            }
        }
        motors[motorIndex].forceOverflow = !timerAlreadyUsed;
        motors[motorIndex].enabled = true;
    }

    pwmMotorsEnabled = true;
}

pwmOutputPort_t *pwmGetMotors(void)
{
    return motors;
}

#ifdef USE_DSHOT
uint32_t getDshotHz(motorPwmProtocolTypes_e pwmProtocolType)
{
    switch (pwmProtocolType) {
        case(PWM_TYPE_DSHOT1200):
            return MOTOR_DSHOT1200_MHZ * 1000000;
        case(PWM_TYPE_DSHOT600):
            return MOTOR_DSHOT600_MHZ * 1000000;
        case(PWM_TYPE_DSHOT300):
            return MOTOR_DSHOT300_MHZ * 1000000;
        default:
        case(PWM_TYPE_DSHOT150):
            return MOTOR_DSHOT150_MHZ * 1000000;
    }
}
#endif

#ifdef USE_SERVOS
void pwmWriteServo(uint8_t index, float value)
{
    if (index < MAX_SUPPORTED_SERVOS && servos[index].ccr) {
        *servos[index].ccr = lrintf(value);
    }
}

void servoInit(const servoConfig_t *servoConfig)
{
    for (uint8_t servoIndex = 0; servoIndex < MAX_SUPPORTED_SERVOS; servoIndex++) {
        const ioTag_t tag = servoConfig->ioTags[servoIndex];

        if (!tag) {
            break;
        }

        servos[servoIndex].io = IOGetByTag(tag);

        IOInit(servos[servoIndex].io, OWNER_SERVO, RESOURCE_INDEX(servoIndex));

        const timerHardware_t *timer = timerGetByTag(tag, TIM_USE_ANY);
#if defined(USE_HAL_DRIVER)
        IOConfigGPIOAF(servos[servoIndex].io, IOCFG_AF_PP, timer->alternateFunction);
#else
        IOConfigGPIO(servos[servoIndex].io, IOCFG_AF_PP);
#endif

        if (timer == NULL) {
            /* flag failure and disable ability to arm */
            break;
        }

        pwmOutConfig(&servos[servoIndex], timer, PWM_TIMER_MHZ, 1000000 / servoConfig->servoPwmRate, servoConfig->servoCenterPulse);
        servos[servoIndex].enabled = true;
    }
}

#endif
