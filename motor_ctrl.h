#ifndef _MOTOTOR_CTRL_H_
#define _MOTOTOR_CTRL_H_

#include <Arduino.h>
#include <stdint.h>

// ——— Encoder pins & counts ———
static const uint8_t A_ENC_A = 35;
static const uint8_t A_ENC_B = 34;
static const uint8_t B_ENC_A = 27;
static const uint8_t B_ENC_B = 16;
extern volatile int32_t enc_count_A;
extern volatile int32_t enc_count_B;

// ——— Motor driver pins ———
// TB6612: AIN1/AIN2 + PWMA, BIN1/BIN2 + PWMB
static const uint8_t AIN1 = 21;
static const uint8_t AIN2 = 17;
static const uint8_t PWMA = 25;
static const uint8_t BIN1 = 22;
static const uint8_t BIN2 = 23;
static const uint8_t PWMB = 26;

// ——— PWM (LEDC) configuration ———
static const int PWM_FREQ = 20000;   // 20 kHz
static const int PWM_RESOLUTION = 8; // 8-bit: 0–255
static const int PWM_CHANNEL_A = 0;
static const int PWM_CHANNEL_B = 1;
static const int MAX_PWM = (1 << PWM_RESOLUTION) - 1;
static const int MIN_PWM = MAX_PWM / 5; // overcome dead-band

void init_motor_ctrl();

/// Set A-motor PWM in range [–1.0 … +1.0] (negative = reverse)
void left_motor_ctrl(float duty);

/// Set B-motor PWM in range [–1.0 … +1.0] (negative = reverse)
void right_motor_ctrl(float duty);

/// Read & reset encoder counts
int32_t read_and_clear_enc_A();
int32_t read_and_clear_enc_B();

#endif