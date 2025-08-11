#include "motor_ctrl.h"

// encoder counters
volatile int32_t enc_count_A = 0;
volatile int32_t enc_count_B = 0;

// ISR callbacks
void IRAM_ATTR isr_encA() { enc_count_A += digitalRead(A_ENC_A) ? +1 : -1; }
void IRAM_ATTR isr_encB() { enc_count_B += digitalRead(B_ENC_A) ? +1 : -1; }

static const float DUTY_DEADBAND = 0.05f;   
static const int   MIN_EFFECTIVE = 35;       

static const float kL_f = 1.000f, kR_f = 0.95925f; // forward
static const float kL_r = 1.000f, kR_r = 0.995f; // reverse

void init_motor_ctrl() {
  pinMode(A_ENC_A, INPUT_PULLUP);
  pinMode(A_ENC_B, INPUT_PULLUP);
  pinMode(B_ENC_A, INPUT_PULLUP);
  pinMode(B_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(A_ENC_B), isr_encA, RISING);
  attachInterrupt(digitalPinToInterrupt(B_ENC_B), isr_encB, RISING);

  // Motor driver pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  // PWM setup
  ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWMA, PWM_CHANNEL_A);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWMB, PWM_CHANNEL_B);

  // Ensure motors stopped
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

// Helper to compute PWM with deadband compensation
static inline int duty_to_pwm(float duty_abs) {
  if (duty_abs <= DUTY_DEADBAND) return 0; // coast
  float scale = (duty_abs - DUTY_DEADBAND) / (1.0f - DUTY_DEADBAND);
  if (scale < 0) scale = 0;
  if (scale > 1) scale = 1;
  int pwm = (int)lroundf(MIN_EFFECTIVE + scale * (MAX_PWM - MIN_EFFECTIVE));
  if (pwm > MAX_PWM) pwm = MAX_PWM;
  return pwm;
}

// Safe direction apply: PWM=0 -> set direction -> small deadtime -> PWM
static inline void apply_motor(int pwm_ch, int in1, int in2, float duty, bool invert) {
  duty = constrain(duty, -1.0f, 1.0f);
  if (invert) duty = -duty;

  int pwm = duty_to_pwm(fabsf(duty));
  ledcWrite(pwm_ch, 0);

  if (pwm == 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  } else {
    bool forward = duty > 0;
    digitalWrite(in1, forward ? LOW : HIGH);
    digitalWrite(in2, forward ? HIGH : LOW);
    delayMicroseconds(4);
    ledcWrite(pwm_ch, pwm);
  }
}

void left_motor_ctrl(float duty) {
  float g = (duty > 0) ? kL_f : kL_r;
  apply_motor(PWM_CHANNEL_A, AIN1, AIN2, duty * g, true);
}

void right_motor_ctrl(float duty) {
  float g = (duty > 0) ? kR_f : kR_r;
  apply_motor(PWM_CHANNEL_B, BIN1, BIN2, duty * g, true);
}


portMUX_TYPE enc_mux = portMUX_INITIALIZER_UNLOCKED;

int32_t read_and_clear_enc_A() {
  int32_t v;
  portENTER_CRITICAL(&enc_mux);
  v = enc_count_A;
  enc_count_A = 0;          
  portEXIT_CRITICAL(&enc_mux);
  return v;
}

int32_t read_and_clear_enc_B() {
  int32_t v;
  portENTER_CRITICAL(&enc_mux);
  v = enc_count_B;
  enc_count_B = 0;       
  portEXIT_CRITICAL(&enc_mux);
  return v;
}
