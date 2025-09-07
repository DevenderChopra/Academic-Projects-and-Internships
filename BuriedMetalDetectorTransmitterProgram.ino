
  Buried‑metal‑detector transmitter • Raspberry Pi Pico
   – 100 Hz H‑bridge  – 16‑sample mid‑window bursts
   – PI regulator on pk‑pk current with 12‑bit, 20 kHz PWM
   – Runtime set‑point via Serial (UART0 IRQ)  – GPIO2 scope toggle
 
#include Arduino.h
#include cmath

 ───────── USER‑TUNABLE GAINS ───────── 
volatile float setPoint = 2.0f;              pk‑pk current target (A)
const float BAND_PCT = 0.10f;                ±10 % dead‑band
float kp = 600.0f, ki = 800.0f, kd = 50.0f;
float integ = 0.0f;
float lastSetPoint = 2.0f;   initial default

 ───────── PINS ───────── 
const uint8_t H1_PIN = 3, H2_PIN = 4, SNUB_PIN = 5;
const uint8_t PWM_PIN = 6, VIN_PIN = A0, TOGGLE_PIN = 2;

 ───────── TIMING ───────── 
const float Fm_Hz = 100.0f;
const int   SNUB_US = 150;
const float PERIOD_US = 1e6f  Fm_Hz;      10 000 µs
const float TON_US  = PERIOD_US  0.4f;
const float TOFF_US = PERIOD_US - TON_US;
const float NON_US  = TON_US   0.5f;
const float NOFF_US = TOFF_US  0.5f;

 ───────── ADC & SENSOR ───────── 
const float ADC_REF = 3.3f;
const int   ADC_MAX = 4095;
const float AMP_GAIN = 0.675f, AMP_OFF = 1.65f;
const float SENS = 0.100f;                  V A⁻¹
const int   NSAMPLES = 16;
const int   SAMPLE_SPACING_US = 5;          delay between successive samples

 ───────── PWM (Philhower) ───────── 
const uint16_t PWM_MAX = 4095;

 ───────── GLOBALS ───────── 
uint16_t pwmCmd = PWM_MAX  2;
unsigned long lastPrint = 0;
const uint16_t REPORT_MS = 500;

 ───────── UART RX ISR VARS ───────── 
char rxBuf[16];
volatile uint8_t rxIdx = 0;

 ───────── HELPERS ───────── 
inline float adcToVout(int a)  { return a  (ADC_REF  ADC_MAX); }
inline float voutToVin(float v){ return ((v - AMP_OFF)  AMP_GAIN) + 2.5f; }

 average 16 samples, evenly spaced 
float sampleBurst() {
  float acc = 0;
  for (int i = 0; i  NSAMPLES; ++i) {
    acc += voutToVin(adcToVout(analogRead(VIN_PIN)));
    delayMicroseconds(SAMPLE_SPACING_US);
  }
  return acc  NSAMPLES;
}

void setWindow(uint8_t pinLow, bool start) {
  digitalWrite(SNUB_PIN, LOW);
  digitalWrite(pinLow, start  LOW  HIGH);
  delayMicroseconds(SNUB_US);
  digitalWrite(SNUB_PIN, HIGH);
}

 ───────── ISR UART0 receive ───────── 
void onUART0Rx() {
  while (uart_is_readable(uart0)) {
    char c = uart_getc(uart0);
    if (c == 'r'  c == 'n') {                line end → parse
      rxBuf[rxIdx] = '0';
      float val = atof(rxBuf);
      val = roundf(val  10.0f)  10.0f;         keep 1 decimal
     if (val  0.0f && val  20.0f) {
  if (val  lastSetPoint) {
    setPoint = val - 0.4f;    👈 force undershoot
  } else {
    setPoint = val;
  }

  lastSetPoint = val;
  Serial.print(F(New set‑point ));
  Serial.println(val, 1);
}


      rxIdx = 0;
      memset(rxBuf, 0, sizeof(rxBuf));
      Serial.print(F(New set‑point ));
      Serial.println(setPoint, 1);
    } else if (rxIdx  sizeof(rxBuf) - 1) {
      rxBuf[rxIdx++] = c;
    }
  }
}

 ───────── FILTER 8‑point boxcar + EMA ───────── 
float smoothPkPk(float pk) {
  static float ring[8] = {0};
  static uint8_t idx = 0;
  static float ema = 0;
  ring[idx] = pk; idx = (idx + 1) & 7;
  float sum = 0; for (float v  ring) sum += v;
  float avg = sum  0.125f;
  ema = 0.05f  avg + 0.95f  ema;
  return ema;
}
 ── Serial command parser (poll, non‑blocking) ── 
void handleSerial() {
  static char buf[16];
  static uint8_t idx = 0;

  while (Serial.available()) {
    char c = Serial.read();

    if (c == 'r'  c == 'n') {       end‑of‑line
      buf[idx] = '0';

      float v = atof(buf);              string → float
      v = roundf(v  10.0f)  10.0f;    keep 1 decimal

     if (v  0.0f && v  20.0f) {
  if (v  lastSetPoint) {
    setPoint = v - 0.4f;    👈 force undershoot
  } else {
    setPoint = v;
  }

  lastSetPoint = v;
  Serial.print(F(New set‑point ));
  Serial.println(v, 1);
}

      idx = 0;                          reset for next line
    }
    else if (idx  sizeof(buf) - 1) {
      buf[idx++] = c;                   build the line
    }
  }
}


 ───────── SETUP ───────── 
void setup() {
  Serial.begin(9600);
  pinMode(TOGGLE_PIN, OUTPUT);

  analogReadResolution(12);
  analogWriteResolution(12);
  analogWriteFreq(20000);             Philhower only
  analogWriteRange(PWM_MAX);          Philhower only

  pinMode(H1_PIN, OUTPUT); pinMode(H2_PIN, OUTPUT);
  pinMode(SNUB_PIN, OUTPUT); pinMode(PWM_PIN, OUTPUT);
  pinMode(VIN_PIN, INPUT);
  analogWrite(PWM_PIN, pwmCmd);

   UART0 IRQ for Serial (USB CDC uses different path, so echo stays) 
  uart_set_irq_enables(uart0, true, false);
  irq_set_exclusive_handler(UART0_IRQ, onUART0Rx);
  irq_set_enabled(UART0_IRQ, true);
}

 ───────── LOOP ───────── 
void loop() {
  uint32_t tStart = micros();
 handleSerial();    
 pwmCmd = constrain(pwmCmd, PWM_MAX  0.1f, PWM_MAX);   Clamp between 10%–100% duty  
  analogWrite(PWM_PIN, pwmCmd);                   apply previous duty

   +ve half‑cycle 
  setWindow(H1_PIN, true);
  uint32_t remain = NON_US - SNUB_US;
  delayMicroseconds(remain  2);
  uint32_t t0 = micros();
  float VpRaw = sampleBurst();
  uint32_t dur = micros() - t0;
  delayMicroseconds(remain  2 - dur);

  float BasePos = 0;
  setWindow(H1_PIN, false);
  remain = NOFF_US - SNUB_US;
  delayMicroseconds(remain  2);
  t0 = micros();
  BasePos = sampleBurst();
  dur = micros() - t0;
  delayMicroseconds(remain  2 - dur);

   –ve half‑cycle 
  setWindow(H2_PIN, true);
  remain = NON_US - SNUB_US;
  delayMicroseconds(remain  2);
  t0 = micros();
  float VnRaw = sampleBurst();
  dur = micros() - t0;
  delayMicroseconds(remain  2 - dur);

  float BaseNeg = 0;
  setWindow(H2_PIN, false);
  remain = NOFF_US - SNUB_US;
  delayMicroseconds(remain  2);
  t0 = micros();
  BaseNeg = sampleBurst();
  dur = micros() - t0;
  delayMicroseconds(remain  2 - dur);

   pk‑pk compute + filter 
  float pk2pk = ( (VpRaw - BasePos) - (VnRaw - BaseNeg) )  SENS;
  float pkSmooth = smoothPkPk(pk2pk);

   PI control 
  Ultra-fast PID control 
 Fast PID with Anti-Windup 
static float lastErr = 0;
float err  = setPoint - pkSmooth;
float band = setPoint  BAND_PCT;
float dt   = (micros() - tStart)  1e-6f;

float pTerm = kp  err;
float dErr  = (err - lastErr)  dt;
float dTerm = kd  dErr;

int32_t rawOutput = (int32_t)(pTerm + integ + dTerm) + (PWM_MAX  1);

 Anti-windup logic only integrate if PWM is not saturated or helps
if (fabs(err)  band) {
  bool canIntegrate = true;

  if ((rawOutput = PWM_MAX && err  0)  (rawOutput = 0 && err  0)) {
    canIntegrate = false;
  }

  if (canIntegrate) {
    integ += ki  err  dt;
    integ = constrain(integ, -PWM_MAX  0.6f, PWM_MAX  0.6f);
  }
}

lastErr = err;

 Recalculate final output after I update
rawOutput = (int32_t)(pTerm + integ + dTerm) + (PWM_MAX  1);
pwmCmd = constrain(rawOutput, 0, PWM_MAX);

   telemetry & GPIO toggle 
  if (millis() - lastPrint = REPORT_MS) {
    digitalWrite(TOGGLE_PIN, !digitalRead(TOGGLE_PIN));   scope marker
    Serial.print(F(pk2pk=)); Serial.print(pkSmooth+0.32, 2);
    Serial.print(F( A  pwm=)); Serial.println(pwmCmd);
    lastPrint = millis();
  }
}