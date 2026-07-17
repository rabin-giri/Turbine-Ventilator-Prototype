/**
 * @file Turbine_Ventilator_Prototype.ino
 * @brief 2019 Prototype Firmware for Turbine-Based Ventilator
 * 
 * This firmware implements Volume Control (VC), Pressure Control (PC), and CPAP 
 * modes using a turbine blower driven by a high-frequency PWM signal.
 * It includes non-linear PID control to manage turbine inertia and compensates
 * flow sensor readings dynamically using calibration arrays stored in PROGMEM.
 * 
 * Intended for open-source reference and educational purposes.
 */

#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <avr/pgmspace.h> // Required for safe PROGMEM reading

// -----------------------------------------------------------------------------
// System Configuration & Tuning Constants
// -----------------------------------------------------------------------------
#define CONTROL_LOOP_MS         10     // Control loop interval in milliseconds (100 Hz)
#define TRIGGER_THRESHOLD_CMH2O -0.2f  // Pressure drop required to trigger a breath in AC modes
#define TRIGGER_DEBOUNCE_CYCLES 3      // Number of consecutive loops the pressure must stay below threshold

// -----------------------------------------------------------------------------
// Global Variables & Sensor Coefficients
// -----------------------------------------------------------------------------
// Normal air flow meter coefficients
volatile float K0, K1, K2, K3, K4;
volatile float Tcal, S, Z, Tcorr;
volatile uint8_t Sets_of_coeffs;
volatile float Vf[9], A[9], B[9], C[9];
volatile float observed_tidal_volume = 0;

// Oxygen flow meter coefficients
volatile float Oxygen_K0, Oxygen_K1, Oxygen_K2, Oxygen_K3, Oxygen_K4;
volatile float Oxygen_Tcal, Oxygen_S, Oxygen_Z, Oxygen_Tcorr;
volatile uint8_t Oxygen_Sets_of_coeffs;
volatile float Oxygen_Vf[9], Oxygen_A[9], Oxygen_B[9], Oxygen_C[9];
volatile float Oxygen_volume = 0;

// Ventilator Modes & Target Parameters
enum vent_mode { PC_AC, VC_AC, VC_CMV, PC_CMV, PC_CMV_VG, CPAP, BIPAP };
enum breath_phase { PHASE_INIT, PHASE_INSPIRATION, PHASE_BLOW_OFF, PHASE_EXPIRATION };

float   target_IE = 0.5;
uint8_t target_RR = 18;
uint8_t target_FIO2 = 40;
uint8_t target_upper_pressure_limit = 15;
uint8_t target_lower_pressure_limit = 5;
int     target_tidal_volume = 400;
float   target_flow_rate = 35;
int     target_pressure = 15;
uint8_t target_PEEP = 5;

int inspiratory_time = 100;
int expiratory_time = 100;
int inspiratory_hold_time = 100;

vent_mode mode;
bool AC_mode = false;

// Control system parameters
float Kp = 0.0f;
float Ke = 0;
float Kd = 0;

// -----------------------------------------------------------------------------
// ROM Data for Flowmeter Non-Linear Calibration
// -----------------------------------------------------------------------------
const PROGMEM uint8_t rom[] = {
  83,169,119,222,251,51,0,12,210,9,66,0,7,219,1,5,63,112,140,114,59,9,216,6,59,31,115,
  77,55,27,193,15,52,19,135,102,65,168,225,72,63,217,219,35,63,159,59,100,189,187,213,
  80,0,0,0,0,0,0,0,0,0,0,9,0,62,174,134,246,190,141,51,159,64,15,30,217,64,93,182,70,
  63,35,176,23,191,15,16,118,64,95,60,220,63,169,122,62,63,123,33,14,191,152,15,254,
  64,147,227,234,63,80,222,26,63,168,64,134,192,89,213,116,64,211,196,132,63,0,104,
  69,63,227,141,108,193,37,198,251,65,29,95,204,62,164,41,158,64,9,157,221,193,79,
  157,179,65,57,156,169,62,76,204,205,64,50,54,46,65,223,205,181,64,155,60,6,62,137,
  55,76,64,81,23,173,194,71,107,60,65,127,79,37,62,34,111,97,64,113,127,167,194,71,
  107,60,65,127,79,37,62,34,111,97,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
};

const PROGMEM uint8_t Oxygen_rom[] = {
  65,26,120,118,203,195,0,12,210,10,66,0,7,218,12,21,63,112,166,1,59,23,21,48,59,28,
  26,211,55,43,254,183,52,7,28,182,65,168,225,72,63,217,219,35,63,159,59,100,189,134,
  36,253,0,0,0,0,0,0,0,0,0,0,9,0,62,186,27,104,190,151,210,122,64,6,154,133,64,60,
  11,193,63,41,174,173,191,18,14,61,64,78,176,140,63,155,35,145,63,129,82,32,191,156,
  56,69,64,139,22,61,63,63,18,142,63,172,153,154,192,127,133,222,64,212,173,194,62,
  220,68,164,63,231,100,24,193,37,234,174,65,22,76,189,62,151,70,93,64,12,145,171,
  193,115,61,106,65,54,130,233,62,76,204,205,64,51,153,137,65,57,154,17,64,234,229,
  65,62,107,133,31,64,82,95,144,194,13,210,197,65,101,153,165,62,30,252,196,64,116,
  76,110,194,13,210,197,65,101,153,165,62,30,252,196,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
};

const float T_conversion_table[] = {
  4.585,4.561,4.536,4.510,4.483,4.455,4.426,4.396,4.364,4.332,4.298,4.262,4.226,4.188,
  4.150,4.110,4.068,4.026,3.982,3.937,3.891,3.844,3.796,3.747,3.697,3.645,3.593,3.540,
  3.486,3.431,3.376,3.320,3.263,3.206,3.148,3.090,3.031,2.972,2.913,2.854,2.795,2.736,
  2.676,2.617,2.559,2.500,2.442,2.384,2.327,2.270,2.213,2.158,2.102,2.048,1.994,1.942,
  1.889,1.838,1.788,1.738,1.690,1.642,1.595,1.550,1.505,1.461,1.418,1.377,1.336,1.296,
  1.257,1.219,1.183,1.147,1.112,1.078,1.045,1.012,0.981,0.951,0.921,0.893,0.865,0.838,
  0.812,0.787,0.762,0.738,0.715,0.693,0.671
};

// -----------------------------------------------------------------------------
// Helper Functions
// -----------------------------------------------------------------------------

/**
 * Safely extracts a 32-bit float from Big Endian PROGMEM bytes.
 * Corrects alignment issues present in the original prototype.
 */
float extract_float_from_progmem(const uint8_t* rom_array, int start_idx) {
  union {
    float f;
    uint8_t b[4];
  } val;
  // Read in reverse order (Big-Endian to Little-Endian AVR conversion)
  val.b[3] = pgm_read_byte(&rom_array[start_idx]);
  val.b[2] = pgm_read_byte(&rom_array[start_idx + 1]);
  val.b[1] = pgm_read_byte(&rom_array[start_idx + 2]);
  val.b[0] = pgm_read_byte(&rom_array[start_idx + 3]);
  return val.f;
}

void init_coefficient_air_flowmeter(void) {
  K0    = extract_float_from_progmem(rom, 16);
  K1    = extract_float_from_progmem(rom, 20);
  K2    = extract_float_from_progmem(rom, 24);
  K3    = extract_float_from_progmem(rom, 28);
  K4    = extract_float_from_progmem(rom, 32);
  Tcal  = extract_float_from_progmem(rom, 36);
  S     = extract_float_from_progmem(rom, 40);
  Z     = extract_float_from_progmem(rom, 44);
  Tcorr = extract_float_from_progmem(rom, 48);

  for (int i = 0; i < 9; i++) {
    int offset = 64 + (i * 16);
    Vf[i] = extract_float_from_progmem(rom, offset);
    A[i]  = extract_float_from_progmem(rom, offset + 4);
    B[i]  = extract_float_from_progmem(rom, offset + 8);
    C[i]  = extract_float_from_progmem(rom, offset + 12);
  }
  Sets_of_coeffs = pgm_read_byte(&rom[62]);
}

void init_coefficient_Oxygen_flowmeter(void) {
  Oxygen_K0    = extract_float_from_progmem(Oxygen_rom, 16);
  Oxygen_K1    = extract_float_from_progmem(Oxygen_rom, 20);
  Oxygen_K2    = extract_float_from_progmem(Oxygen_rom, 24);
  Oxygen_K3    = extract_float_from_progmem(Oxygen_rom, 28);
  Oxygen_K4    = extract_float_from_progmem(Oxygen_rom, 32);
  Oxygen_Tcal  = extract_float_from_progmem(Oxygen_rom, 36);
  Oxygen_S     = extract_float_from_progmem(Oxygen_rom, 40);
  Oxygen_Z     = extract_float_from_progmem(Oxygen_rom, 44);
  Oxygen_Tcorr = extract_float_from_progmem(Oxygen_rom, 48);

  for (int i = 0; i < 9; i++) {
    int offset = 64 + (i * 16);
    Oxygen_Vf[i] = extract_float_from_progmem(Oxygen_rom, offset);
    Oxygen_A[i]  = extract_float_from_progmem(Oxygen_rom, offset + 4);
    Oxygen_B[i]  = extract_float_from_progmem(Oxygen_rom, offset + 8);
    Oxygen_C[i]  = extract_float_from_progmem(Oxygen_rom, offset + 12);
  }
}

// -----------------------------------------------------------------------------
// Sensor Data Acquisition
// -----------------------------------------------------------------------------
int get_gas_temperature(float vt) {
  int first = 0, last = 90, middle = (first + last) / 2;
  while (first <= last) {
    if (fabs(T_conversion_table[middle] - vt) < 0.05) {
      float error1 = fabs(T_conversion_table[middle + 1] - vt);
      float error2 = fabs(T_conversion_table[middle] - vt);
      float error3 = fabs(T_conversion_table[middle - 1] - vt);
      if (error1 < error2 && error1 < error3) return middle - 19;
      else if (error2 < error1 && error2 < error3) return middle - 20;
      else return middle - 21;
    }
    if (T_conversion_table[middle] < vt) last = middle - 1;
    else first = middle + 1;
    
    middle = (first + last) / 2;
  }
  return 0;
}

int get_ABC_parameter_index(float vf, const volatile float* vf_array) {
  if (vf > vf_array[8]) return 8;
  if (vf_array[0] > vf) return 0;
  
  for (int i = 1; i < 9; i++) {
    if (vf_array[i] > vf) {
      if (i == 0 || i == 8) return i;
      float error1 = fabs(vf_array[i] - vf);
      float error2 = fabs(vf_array[i - 1] - vf);
      return (error1 < error2) ? i : (i - 1);
    }
  }
  return 0;
}

float get_air_flowmeter_reading() {
  float vf = analogRead(A3) * 0.0048875855f;
  float vt = analogRead(A4) * 0.0048875855f;
  float T = get_gas_temperature(vt) + Tcorr;
  float vb = (vf + Z) / S;
  float TempCompFactor = K0 + (K1 * vb) + (T * (K2 + T * (K3 + K4 * T)));
  float Vfstd = ((vf + Z) * TempCompFactor) - Z;
  int index = get_ABC_parameter_index(Vfstd, Vf);
  float Vfstd_pow_2 = Vfstd * Vfstd;
  float Q = A[index] + ((B[index] + C[index] * Vfstd_pow_2 * Vfstd) * Vfstd_pow_2);
  return Q * 0.75f;
}

float get_pressure_data(void) {
  float pressure = 0;
  Wire.requestFrom(0x78, 2);    
  while (Wire.available()) {
    int pValue;
    unsigned char *ptr;
    unsigned char v1 = Wire.read(); 
    unsigned char v2 = Wire.read(); 
    ptr = (unsigned char *)&pValue;
    v1 = v1 & 0x7f;
    ptr[0] = v2;
    ptr[1] = v1;
    pressure = 0.002361f * pValue - 6.3228f;    
  }
  return pressure;
}

// -----------------------------------------------------------------------------
// Hardware Initialization
// -----------------------------------------------------------------------------
static void pwmInit(void) {
  pinMode(8, OUTPUT); pinMode(9, OUTPUT); pinMode(10, OUTPUT);
  pinMode(4, OUTPUT); pinMode(5, OUTPUT); pinMode(6, OUTPUT);    
  pinMode(7, OUTPUT);

  // Configure Timer 4 for high-frequency PWM
  TCCR4A = 0; TCCR4B = 0; TCCR4C = 0; TIMSK4 = 0;
  TCCR4A = (1 << COM4A1) | (1 << COM4B1) | (1 << COM4C1);
  TCCR4B = (1 << CS40) | (1 << WGM43);
  ICR4 = 1999;
  OCR4A = 0;
  OCR4B = 0;
  OCR4C = 1500;
  
  digitalWrite(4, HIGH);
  digitalWrite(5, LOW);
  digitalWrite(9, HIGH);
  digitalWrite(10, LOW);
}

// -----------------------------------------------------------------------------
// Controller Logic
// -----------------------------------------------------------------------------
static void calculate_required_parameter(void) {
  AC_mode = false;
  int cycle_time = 60000 / target_RR;
  
  switch(mode) {
    case VC_AC:
      AC_mode = true;
      // Fallthrough intentional here to share parameter math with VC_CMV
    case VC_CMV:
      inspiratory_time = cycle_time * target_IE;
      expiratory_time = cycle_time - inspiratory_time;
      inspiratory_hold_time = inspiratory_time * 0.4;
      inspiratory_time = inspiratory_time - inspiratory_hold_time;
      target_flow_rate = 68.0f * ((float)target_tidal_volume) / inspiratory_time; 
      break;

    case PC_AC:
      AC_mode = true;
    case PC_CMV:
      inspiratory_time = cycle_time * target_IE;
      expiratory_time = cycle_time - inspiratory_time;
      float pressure_difference = target_pressure - target_PEEP;
      
      Kp = 0.7f;
      Ke = 1.4f;
      Kd = 15.0f;
      break;
  }
}

void CPAP_mode_process(void) {
  static unsigned long local_ref_time = millis();
  static float pwm = 200;
  static float prev_error = 0;
  
  if ((millis() - local_ref_time) >= CONTROL_LOOP_MS) {
    local_ref_time = millis();
    float flow_rate = get_air_flowmeter_reading();
    float pressure = get_pressure_data() * 10;
    float error = target_pressure * 10 - pressure;
    
    // Accumulating velocity-form PID
    pwm += error * 2.5f + (error - prev_error) * 15.0f;
    
    if (pwm > 1900) pwm = 1900;
    else if (pwm < 200) pwm = 200;
    
    OCR4C = pwm;
    prev_error = error;
    
    Serial.print(flow_rate); Serial.print(" ");
    Serial.print(pressure / 10); Serial.print(" 0\n");
  }
}

void volume_control_mode_process(void) {
  static unsigned long ref_time = millis();
  static unsigned long local_ref_time = millis();
  static float prev_error = 0;
  static breath_phase phase = PHASE_INIT;
  static float pwm = 0;
  static uint8_t trigger_count = 0;

  switch(phase) {
    case PHASE_INIT:
      observed_tidal_volume = 0;
      pwm = 400;
      OCR4A = 1999;
      ref_time = millis();
      local_ref_time = millis();
      phase = PHASE_INSPIRATION;
      break;

    case PHASE_INSPIRATION:
      if ((millis() - ref_time) < inspiratory_time) {
        if ((millis() - local_ref_time) >= CONTROL_LOOP_MS) {
          local_ref_time = millis();
          float flow_rate = get_air_flowmeter_reading();
          float pressure = get_pressure_data();
          float error = target_flow_rate - flow_rate;
          
          observed_tidal_volume += flow_rate;
          pwm += error * 11.0f + (error - prev_error) * 30.0f;
          
          if (pwm > 1900) pwm = 1900;
          else if (pwm < 200) pwm = 200;
          OCR4C = pwm;
          prev_error = error;
          
          Serial.print(flow_rate); Serial.print(" ");
          Serial.print(pressure * 10); Serial.print(" ");
          Serial.println(observed_tidal_volume * 0.5f);
        }
      } else {
        OCR4A = 0;
        OCR4C = 200;
        prev_error = 0;
        pwm = 200;
        ref_time = millis();
        local_ref_time = millis();
        phase = PHASE_EXPIRATION;
      }
      break;

    case PHASE_EXPIRATION:
      if ((millis() - ref_time) < (expiratory_time + inspiratory_hold_time)) {
        if ((millis() - ref_time) >= inspiratory_hold_time) {
          OCR4A = 1999; // Re-engage turbine for PEEP maintenance
        }
        
        if ((millis() - local_ref_time) >= CONTROL_LOOP_MS) {
          local_ref_time = millis();
          float pressure = get_pressure_data();
          
          // Patient Trigger Detection (Assist-Control)
          if (AC_mode && pressure < TRIGGER_THRESHOLD_CMH2O) {
            if (++trigger_count >= TRIGGER_DEBOUNCE_CYCLES) {
              phase = PHASE_INIT;
              trigger_count = 0;
              return;
            }
          } else {
            trigger_count = 0;
          }

          float error = 10.0f * (target_PEEP - pressure);
          pwm += error * 1.5f + (error - prev_error) * 5.0f;
          
          if (pwm > 1900) pwm = 1900;
          else if (pwm < 200) pwm = 200;
          OCR4C = pwm;
          prev_error = error;
        }
      } else {
        phase = PHASE_INIT;
      }
      break;
  }
}

void pressure_control_mode_process(void) {
  static unsigned long ref_time = millis();
  static unsigned long local_ref_time = millis();
  static float prev_error = 0;
  static breath_phase phase = PHASE_INIT;
  static float pwm = 200;
  static uint8_t trigger_count = 0;

  switch(phase) {
    case PHASE_INIT:
      observed_tidal_volume = 0;
      pwm = 0;
      OCR4A = 1000;
      OCR4C = 200;
      ref_time = millis();
      local_ref_time = millis();
      phase = PHASE_INSPIRATION;
      break;

    case PHASE_INSPIRATION:
      if ((millis() - ref_time) < inspiratory_time) {
        if ((millis() - local_ref_time) >= CONTROL_LOOP_MS) {
          local_ref_time = millis();
          float flow_rate = get_air_flowmeter_reading();
          float pressure = get_pressure_data() * 10.0f;
          float error = target_pressure * 10.0f - pressure;
          
          // Fixed float truncating bug in abs() -> fabs()
          float proportional = (error > 0.0f ? 1.0f : -1.0f) * pow(fabs(error), Ke) * Kp; 
          pwm += proportional + (error - prev_error) * Kd;
          observed_tidal_volume += flow_rate;
          
          if (pwm > 1900) pwm = 1900;
          else if (pwm < 200) pwm = 200;
          
          OCR4C = pwm;
          prev_error = error;
        }
      } else {
        OCR4C = 100;
        OCR4A = 0;
        ref_time = millis();
        local_ref_time = millis();
        phase = PHASE_BLOW_OFF;
      }
      break;

    case PHASE_BLOW_OFF:
      // Wait for airway pressure to fall near PEEP before engaging active PEEP control
      if (get_pressure_data() > target_PEEP) {
        if ((millis() - local_ref_time) >= CONTROL_LOOP_MS) {
           local_ref_time = millis();
        }
      } else {
        prev_error = 0;
        pwm = 0;
        phase = PHASE_EXPIRATION;
      }
      break;

    case PHASE_EXPIRATION:
      if ((millis() - ref_time) < expiratory_time) {
        if ((millis() - local_ref_time) >= CONTROL_LOOP_MS) {
          local_ref_time = millis();
          float pressure = get_pressure_data() * 10.0f;
          
          // Patient Trigger Detection
          if (AC_mode && pressure < (TRIGGER_THRESHOLD_CMH2O * 10.0f)) {
            if (++trigger_count >= TRIGGER_DEBOUNCE_CYCLES) {
              phase = PHASE_INIT;
              trigger_count = 0;
              return;
            }
          } else {
             trigger_count = 0;
          }

          float error = target_PEEP * 10.0f - pressure;
          float proportional = (error > 0.0f ? 1.0f : -1.0f) * pow(fabs(error), 1.2f) * 1.7f; 
          pwm += proportional + (error - prev_error) * 10.0f;
          
          if (pwm > 1900) pwm = 1900;
          else if (pwm < 200) pwm = 200;
          OCR4C = pwm;
          prev_error = error;
        }
      } else {
        phase = PHASE_INIT;
      }
      break;
  }
}

void setup() {
  Serial.begin(115200); 
  Wire.begin();
  
  init_coefficient_Oxygen_flowmeter();
  init_coefficient_air_flowmeter();
  pwmInit();
  
  mode = CPAP;
  calculate_required_parameter();
  OCR4A = 1999;
}

void loop() {
  switch(mode) {
    case VC_AC:
    case VC_CMV:
      volume_control_mode_process();
      break;
    case PC_AC:
    case PC_CMV:
      pressure_control_mode_process();
      break;
    case CPAP:
      CPAP_mode_process();
      break;
    default:
      break;
  }
}