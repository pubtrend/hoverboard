
#include "TWI.h"
#include "init.h"
static volatile uint16_t ADC_acc = 0;
static volatile uint8_t us_ovf = 0;
static volatile uint16_t t2_ovf = 0;
int8_t exit_dir = 0; // -1 = left, +1 = right


/*
Wiring:
US: trig PB3, echo PD2
IR: left ADC0, top ADC1, right ADC2
servo: D9
lift fan: P3
thrust fan: P4/P11
IMU: P7/P19
*/

 
  //
 // TUNING CONSTANTS
//

/* Lift fan — OCR0A, range 0-255. */
#define LIFT_SPEED              180
 
/* Thrust fan — OCR0B, range 0-255. */
#define THRUST_CRUISE           120
#define THRUST_SLOW             70
#define THRUST_OFF              0
 
/* Servo index - goes into Servo_angle[].
   127 = center. Lower = left, higher = right. */
#define SERVO_CENTER            127
#define SERVO_TURN_RIGHT        160
#define SERVO_TURN_LEFT         94
 
/* IR centering gain — lower number means more aggressive correction */
#define IR_CENTER_GAIN          4

/* Left wall disappears = exit gap found. */
#define IR_LEFT_GAP_THRESHOLD   150
 
/* Bar detection — IR sensor overhead. */
#define BAR_THRESHOLD           180
 
/* Front wall detection — US sensor pulse count. CALIBRATE. */
#define WALL_NEAR               30
 
/* How long to pause under the bar, in 20ms ticks. */
#define BAR_PAUSE_TICKS         150      /* ~3 second */
 
// Debugging block
#define DEBUG
 
/* ---- Standard includes ---- */
#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

// IMU

// imu flags
typedef struct {
    uint8_t TX_new_data:1;
    uint8_t TX_finished:1;
    uint8_t TX_buffer1_empty:1;
    uint8_t TX_buffer2_empty:1;
    uint8_t RX_flag:3;
    uint8_t TWI_ACK:1;
} flags_t;
 
/* MPU-6050 register addresses — from the IMU datasheet, do not change */
#define MPU6050_ADDR        0x68
#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_XOUT_H    0x3B
#define REG_PWR_MGMT_1      0x6B
#define REG_WHO_AM_I        0x75
 
/* IMU settings */
#define MPU_GYRO_FS_SEL     0       /* ±250 deg/s */
#define MPU_ACCEL_FS_SEL    0       /* ±2g */
#define MPU_DLPF_CFG        3
#define MPU_SMPLRT_DIV      9       /* 100 Hz sample rate */
 
/* TWI shared variables — must match TWI_290.c */
volatile flags_t flags; 
volatile uint8_t TWI_status = 0;
volatile uint8_t TWI_byte   = 0;
 
/* IMU state — yaw_deg is used by navigation to correct drift */
static float gyro_lsb_per_dps  = 131.0f;
static float accel_lsb_per_g   = 16384.0f;
static float gyro_bias_z_dps   = 0.0f;
static float accel_bias_x_g    = 0.0f;
static float yaw_deg            = 0.0f;
static float ax_g               = 0.0f;
static float ay_g               = 0.0f;
static float az_g               = 0.0f;
static float gx_dps             = 0.0f;
static float gy_dps             = 0.0f;
static float gz_dps             = 0.0f;
 
/* ---- UART helpers ---- */
static void uart_send(char c) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = (uint8_t)c;
}
static void uart_print(const char *s) {
    while (*s) uart_send(*s++);
}
static void uart_print_int(long val) {
    char buf[12];
    sprintf(buf, "%ld\r\n", val);
    uart_print(buf);
}
static void uart_print_float(const char *label, float value) {
    char buf[24];
    uart_print(label);
    dtostrf(value, 0, 2, buf);
    uart_print(buf);
}
 
// Assign1 mapping function 
int map_val(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


int constrain_val(int x, int lo, int hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}


// Returns distance in cm from raw 8-bit ADC value
        // Based on GP2Y0A21YK0F datasheet curve, 5V Vref
    int ir_to_cm(uint8_t adc) {
    if (adc > 200) return 10;
    if (adc < 30)  return 80;
    
    static const uint8_t adc_pts[] = {200, 170, 140, 110,  90,  75,  60,  45,  30};
    static const uint8_t  cm_pts[] = { 10,  12,  15,  20,  25,  30,  40,  55,  80};
    
    for (uint8_t i = 0; i < 8; i++) {
        if (adc >= adc_pts[i+1]) {
            uint8_t adc_range = adc_pts[i] - adc_pts[i+1];
            uint8_t cm_range  = cm_pts[i+1] - cm_pts[i];
            int raw = cm_pts[i] + (int)(adc_pts[i] - adc) * cm_range / adc_range;
            return raw * 20 / 35 + 4;  // calibration applied here
        }
    }
    return 80;
}
 
// TWI helpers
static uint8_t mpu_write(uint8_t reg, uint8_t value) {
    return Write_Reg(MPU6050_ADDR, reg, value);
}
static uint8_t mpu_read_reg(uint8_t reg, uint8_t *value) {
    uint8_t status = Read_Reg(MPU6050_ADDR, reg);
    if (status) return status;
    *value = TWI_byte;
    return 0;
}
static uint8_t mpu_read_burst(uint8_t start_reg, uint8_t *buf, uint8_t count) {
    return Read_Reg_N(MPU6050_ADDR, start_reg, count, (int16_t)buf);
}
static int16_t be16_to_i16(uint8_t hi, uint8_t lo) {
    return (int16_t)((((uint16_t)hi) << 8) | lo);
}
 
// IMU init
static uint8_t mpu_init(void) {
    uint8_t who_am_i = 0;
    
    /* Send reset — IMU drops bus immediately so ignore ACK */
    mpu_write(REG_PWR_MGMT_1, 0x80);
    _delay_ms(100);
    
    /* Wake up and configure — ignore ACK on writes, TWI_ACK unreliable */
    mpu_write(REG_PWR_MGMT_1, 0x01);
    _delay_ms(10);
    mpu_write(REG_SMPLRT_DIV, MPU_SMPLRT_DIV);
    mpu_write(REG_CONFIG, MPU_DLPF_CFG & 0x07);
    mpu_write(REG_GYRO_CONFIG,  (uint8_t)((MPU_GYRO_FS_SEL  & 0x03) << 3));
    mpu_write(REG_ACCEL_CONFIG, (uint8_t)((MPU_ACCEL_FS_SEL & 0x03) << 3));
    
    /* WHO_AM_I read is the real confirmation everything worked */
    if (mpu_read_reg(REG_WHO_AM_I, &who_am_i)) return 7;
    uart_print("whoami="); uart_print_int(who_am_i);
    if ((who_am_i & 0x7EU) != 0x68U) return 8;
    return 0;
}
 
// read accel/gyro
static uint8_t mpu_read_scaled(void) {
    uint8_t raw[14];
    if (mpu_read_burst(REG_ACCEL_XOUT_H, raw, 14)) return 1;
    ax_g   = (float)be16_to_i16(raw[0],  raw[1])  / accel_lsb_per_g;
    ay_g   = (float)be16_to_i16(raw[2],  raw[3])  / accel_lsb_per_g;
    az_g   = (float)be16_to_i16(raw[4],  raw[5])  / accel_lsb_per_g;
    gx_dps = (float)be16_to_i16(raw[8],  raw[9])  / gyro_lsb_per_dps;
    gy_dps = (float)be16_to_i16(raw[10], raw[11]) / gyro_lsb_per_dps;
    gz_dps = (float)be16_to_i16(raw[12], raw[13]) / gyro_lsb_per_dps;
    return 0;
}
 
// gyro bias calibration while still
static void mpu_calibrate_still(void) {
    const uint16_t samples = 500;
    uint16_t good = 0;
    float sum_gz = 0.0f, sum_ax = 0.0f;
    uart_print("Hold still — calibrating IMU...\r\n");
    _delay_ms(500);
    for (uint16_t i = 0; i < samples; i++) {
        if (!mpu_read_scaled()) {
            sum_gz += gz_dps;
            sum_ax += ax_g;
            good++;
        }
        _delay_ms(5);
    }
    if (good == 0) good = 1;
    gyro_bias_z_dps = sum_gz / good;
    accel_bias_x_g  = sum_ax / good;
    yaw_deg = 0.0f;
    uart_print("Calibration done.\r\n");
}
 
// integrate z gyro into yaw
static void update_yaw(float dt_s) {
    yaw_deg += (gz_dps - gyro_bias_z_dps) * dt_s;
    if (yaw_deg >  180.0f) yaw_deg -= 360.0f;
    if (yaw_deg < -180.0f) yaw_deg += 360.0f;
}
 
// SENSOR VARIABLES
 
#define PWM_TOP 2500
 
// flags updated in ISRs
volatile struct {
    uint8_t TX_finished:1;
    uint8_t sample:1; // pulses on and off every 20ms to slow the main loop down
    uint8_t mode:1; // not used rn
    uint8_t stop:1; // not used rn
    uint8_t T1_ovf0:2; // US flag rollover, 2 bits so we can have 0 to 3 wrap arounds
} nav_flags;
 
/* ADC results — 
   ADC0 = left IR, ADC1 = bar IR, ADC2 = right IR */
static volatile struct {
    uint8_t ADC0;
    uint8_t ADC1;
    uint8_t ADC2;
    uint8_t ADC3; //not used rn
    uint8_t ADC6; //not used rn
    uint8_t ADC7; //not used rn
} ADC_data;
 
/* US sensor pulse — filled by INT0 ISR interrupt */
static volatile struct {
    uint16_t pulse0;      // final duration (send this to main loop)
    uint16_t t_start0;    // time recorded when echo pin goes HIGH
    uint16_t t_end0;      // time recorded when echo pin goes LOW
} PULSE_data;
 
static volatile uint16_t sys_time   = 0;
static volatile uint16_t delay_ms   = 0;
static volatile uint8_t  ADC_sample = 0;
 
#define ADC_SAMPLE_MAX 4
 
// ISRS

/* Timer1 ISR — fires every 20ms. Set nav_flag.sample to 1 to start while loop */
ISR(TIMER1_CAPT_vect) {
    sys_time++;
    delay_ms += 20;
    nav_flags.sample = 1;
    nav_flags.T1_ovf0++;
    us_ovf++;
}

ISR(TIMER2_OVF_vect) {
        t2_ovf++;
    }
 
/* INT0 ISR — fires when PD2 (echo pin) changes
   Measures how long the echo pulse lasts = distance to wall
   records the start and end times and lets the main loop keep running. */

ISR(INT0_vect) {
    if (PIND & (1 << PD2)) {
        t2_ovf = 0;      
        TCNT2 = 0;
        PULSE_data.t_start0 = 0;
    } else {
        uint16_t ticks = t2_ovf * 256 + TCNT2;
        PULSE_data.pulse0 = ticks / 7;
    }
}
 
/* ADC ISR — fires after every ADC conversion
   Cycles through all IR channels automatically.
   Averages 4 readings per channel to reduce noise */
ISR(ADC_vect) {
    if (ADC_sample == 0) {
        ADC_acc = 0;
        ADC_sample++;
        return;
    }
    if (ADC_sample <= ADC_SAMPLE_MAX) {
        ADC_acc += ADCH; // ADCH - predefined register that holds the result of the last ADC conversion 
        ADC_sample++;                 // -- exclusively using the high (8bits) to avoid overflow
        return;
    }
    /* have 4 samples — average them and store result */
    ADC_sample = 0;
    ADC_acc /= ADC_SAMPLE_MAX;
    switch (ADMUX & 7) {
        case 0: ADMUX = (ADMUX & 0xF0) | 0x01; ADC_data.ADC0 = (uint8_t)ADC_acc; return;    //flips btwn channels
        case 1: ADMUX = (ADMUX & 0xF0) | 0x02; ADC_data.ADC1 = (uint8_t)ADC_acc; return;
        case 2: ADMUX = (ADMUX & 0xF0) | 0x03; ADC_data.ADC2 = (uint8_t)ADC_acc; return;
        case 3: ADMUX = (ADMUX & 0xF0) | 0x06; ADC_data.ADC3 = (uint8_t)ADC_acc; return;    // not used rn
        case 6: ADMUX = (ADMUX & 0xF0) | 0x07; ADC_data.ADC6 = (uint8_t)ADC_acc; return;    // not used rn
        case 7: ADMUX = (ADMUX & 0xF0) | 0x00; ADC_data.ADC7 = (uint8_t)ADC_acc; return;    // not used rn
        default: ADMUX = (ADMUX & 0xF0) | 0x00;
    }
}
 
/* Catch-all — prevents crash if an unexpected interrupt fires */
ISR(__vector_default) {}
 

//   US SENSOR TRIGGER

static void us_trigger(void) {
    PORTB &= ~(1 << PB3);
    _delay_us(2);
    PORTB |=  (1 << PB3);
    _delay_us(11);
    PORTB &= ~(1 << PB3);
}
 
// NAVIGATION
 
typedef enum {
    LAUNCH,         
    FORWARD,        
    BAR_DETECTED,   
    TURN_RIGHT,     
    TURN_LEFT,      
    TURN_TO_EXIT,   
    EXIT            
} State;
 

// machine variables
    State    state        = LAUNCH;
    uint16_t tick_counter = 0;      /* counts 20ms ticks for timing */
    uint8_t  turn_count   = 0;      /* tracks how many turns completed */
    const float dt_s      = 0.020f; /* 20ms in seconds for IMU integration */
 
// MAIN
void setup() {
    
    // Hardware init
    UBRR0H = 0;
    UBRR0L = 103;
    UCSR0B = (1 << TXEN0);
    UCSR0C = (3 << UCSZ00);
    uart_print("BOOT\r\n");  // add this immediately after UART init
    gpio_init();
    PORTD &= ~(1 << PD2);  // force pull-up OFF on echo pin
    DDRD  &= ~(1 << PD2);  // ensure input
 
    /* Power all sensor connectors (PD4-PD7 are power control pins) */
    PORTD |= (1 << PD7) | (1 << PD6) | (1 << PD5) | (1 << PD4);
 
    /* UART: 9600 baud */
    UBRR0H = 0;
    UBRR0L = 103;
    UCSR0B = (1 << TXEN0);
    UCSR0C = (3 << UCSZ00);
 
    /* Timer0: fan PWM. Both fans off at startup. */
    timer0_init();
    OCR0A = 0;  
    OCR0B = 0;  
 
    /* Timer1: servo PWM + 20ms heartbeat.
       Passing 1 enables TIMER1_CAPT IRQ which drives nav_flags.sample */
    timer1_50Hz_init(1);
    OCR1A = Servo_angle[SERVO_CENTER];
    OCR1B = 0;
 
    /* US sensor setup
       3 = trig (output) 
       PD2 = echo (input)  — moved from PB0 to use INT0 */
    DDRB  |=  (1 << PB3);              /* PB3 trig as output */
    PORTB &= ~(1 << PB3);              /* trig starts LOW */
    DDRD  &= ~(1 << PD2);              /* PD2 echo as input */
    PORTD &= ~(1 << PD2);              /* no pull-up on echo */
    EICRA |= (1 << ISC00);             /* any change on PD2 triggers INT0 */
    EIMSK |= (1 << INT0);              /* enable INT0 */
 
    /* ADC: start on channel 0, interrupt enabled
       Cycles through ADC0 (left IR), ADC1 (bar IR), ADC2 (right IR) */
    adc_init(0, 1);
 
    /* I2C bus clear — unsticks IMU if left mid-transaction from a previous run */
    DDRC  |=  (1 << PC5) | (1 << PC4);
    PORTC |=  (1 << PC5) | (1 << PC4);
    for (uint8_t i = 0; i < 9; i++) {
        PORTC &= ~(1 << PC5); _delay_us(5);
        PORTC |=  (1 << PC5); _delay_us(5);
    }
    PORTC &= ~(1 << PC4); _delay_us(5);
    PORTC |=  (1 << PC4); _delay_us(5);
    DDRC  &= ~((1 << PC5) | (1 << PC4));
    PORTC |=  (1 << PC5) | (1 << PC4);
 
    twi_init();
    _delay_ms(1000);
 
    /* sei() — open the interrupt gate.
       Must come after ALL init calls.
       Without this nothing fires: no ADC, no heartbeat, no US timing. */
    sei();

    // Timer2: normal mode, prescaler 128
    // Each tick = 128/16MHz = 8us
    // Max range = 256 ticks × 8us = 2ms per overflow
    TCCR2A = 0;
    TCCR2B = (1<<CS22)|(1<<CS20);  // prescaler 128
    TIMSK2 = (1<<TOIE2);           // overflow interrupt

/* Disable ADC interrupt during IMU init to prevent I2C corruption */
ADCSRA &= ~(1 << ADIE);

/* IMU init — if it fails, blink LED and halt */
uint8_t imu_status = mpu_init();
if (imu_status != 0) {
    uart_print("IMU init failed code: ");
    uart_print_int(imu_status);
    DDRB |= (1 << PB5);
    while (1) {
        PORTB ^= (1 << PB5);
        _delay_ms(200);
    }
}



/* Re-enable ADC interrupt after IMU is initialized */
ADCSRA |= (1 << ADIE);

    /* Calibrate IMU — hovercraft must be sitting still during this */
    mpu_calibrate_still();
 
    uart_print("THIS IS THE NEW CODE.\r\n");
 
}
    // Main loop
void loop() {

        while(1) {
 
        /* Wait for 20ms heartbeat from Timer1 ISR.
           Everything below runs at exactly 50Hz. */
        if (!nav_flags.sample) continue;
        nav_flags.sample = 0;
 
        // Trigger US sensor
        us_trigger();
 
        // Read IMU
        if (!mpu_read_scaled()) {
            update_yaw(dt_s);
        }
 
        /* Snapshot all sensors
           Read the latest values filled by ISRs.
           ir_left/ir_right: used to keep hovercraft centered in corridor
           ir_bar: detects overhead bar
           us_dist: distance to front wall */
        uint8_t  ir_left  = ADC_data.ADC0;
        uint8_t  ir_bar   = ADC_data.ADC1;
        uint8_t  ir_right = ADC_data.ADC2;
        uint16_t us_dist  = PULSE_data.pulse0;
 
        // States
        switch (state) {

            case LAUNCH:
            /* Wait 1.5 seconds after lift to thrust */
                OCR0A = LIFT_SPEED;
                OCR0B = THRUST_OFF;
                OCR1A = Servo_angle[SERVO_CENTER];
                tick_counter++;
                if (tick_counter >= 150) {
                    tick_counter = 0;
                    OCR0B = THRUST_CRUISE;
                    yaw_deg = 0.0f;
                    state = FORWARD;
                    uart_print("State: FORWARD\r\n");
                }
                break;
 
            case FORWARD:
            // center using left/right IR


                OCR0B = THRUST_CRUISE;
                {
                    int16_t error = (int16_t)ir_left - (int16_t)ir_right;
                    int servo_idx = SERVO_CENTER + (error / IR_CENTER_GAIN);
                    servo_idx = constrain_val(servo_idx, 80, 174);
                    OCR1A = Servo_angle[servo_idx];
                }
               
                // Wall disappeared → exit gap found, turn 90 degrees then exit 
                if (ir_left < IR_LEFT_GAP_THRESHOLD) {
                    exit_dir = -1;   // LEFT EXIT
                    OCR0B = THRUST_SLOW;
                    yaw_deg = 0.0f;
                    state = TURN_TO_EXIT;
                    uart_print("EXIT LEFT\r\n");
                    break;
                }
                else if (ir_right < IR_LEFT_GAP_THRESHOLD) {
                    exit_dir = +1;   // RIGHT EXIT
                    OCR0B = THRUST_SLOW;
                    yaw_deg = 0.0f;
                    state = TURN_TO_EXIT;
                    uart_print("EXIT RIGHT\r\n");
                    break;
                }
 
                // Bar detected overhead → stop and pause 
                if (ir_bar < BAR_THRESHOLD) {
                    OCR0B = THRUST_OFF;
                    tick_counter = 0;
                    state = BAR_DETECTED;
                    uart_print("State: BAR_DETECTED\r\n");
                    break;
                }
 
                /* Wall close ahead → decide which turn comes next.
                   Exit approach triggers earlier than turns. */
                
                    if (us_dist < WALL_NEAR && us_dist > 0) {
                        OCR0B = THRUST_SLOW;
                        yaw_deg = 0.0f;

                        if (ir_left > ir_right) {
                            state = TURN_LEFT;   // more space on LEFT
                            uart_print("TURN LEFT\r\n");
                        } else {
                            state = TURN_RIGHT;  // more space on RIGHT
                            uart_print("TURN RIGHT\r\n");
                        }
                        turn_count++;                  
                        break;
                        }

                break;
                

            case BAR_DETECTED:
            /* Pause under bar for BAR_PAUSE_TICKS, then resume forward. */
                tick_counter++;
                if (tick_counter >= BAR_PAUSE_TICKS) {
                    tick_counter = 0;
                    OCR0B = THRUST_CRUISE;
                    yaw_deg = 0.0f;
                    state = FORWARD;
                    uart_print("State: FORWARD (after bar)\r\n");
                }
                break;

            case TURN_RIGHT:
            // Turn right until IMU says we've rotated ~180 degrees
                OCR0B = THRUST_SLOW;
                OCR1A = Servo_angle[SERVO_TURN_RIGHT];
                // Turn until IMU says we've rotated ~180 degrees
                if (fabsf(yaw_deg) >= 175.0f) {
                    OCR1A = Servo_angle[SERVO_CENTER];
                    yaw_deg = 0.0f;    // reset yaw for next straight
                    state = FORWARD;
                    uart_print("State: FORWARD (after right turn)\r\n");
                }
                break;
 
            case TURN_LEFT:
            /* Same as TURN_RIGHT but other direction. */
                OCR0B = THRUST_SLOW;
                OCR1A = Servo_angle[SERVO_TURN_LEFT];
                // Turn until IMU says we've rotated ~180 degrees
                if (fabsf(yaw_deg) >= 175.0f) {
                    OCR1A = Servo_angle[SERVO_CENTER];
                    yaw_deg = 0.0f;    // reset yaw for next straight
                    state = FORWARD;
                    uart_print("State: FORWARD (after left turn)\r\n");
                }
                break;

            case TURN_TO_EXIT:
                OCR0B = THRUST_SLOW;

                if (exit_dir == -1) {
                    OCR1A = Servo_angle[SERVO_TURN_LEFT];
                } else {
                    OCR1A = Servo_angle[SERVO_TURN_RIGHT];
                }

                if (fabsf(yaw_deg) >= 85.0f) {
                    OCR1A = Servo_angle[SERVO_CENTER];
                    yaw_deg = 0.0f;
                    state = EXIT;
                    uart_print("State: EXIT\r\n");
                }
                break;
 
            case EXIT:
            /* Left gap detected in FORWARD, servo already steering left.
            Just keep going until off the course. */
                OCR0B = THRUST_CRUISE;
                break;
 
        } // end of states


        
 
        // Debug output 
        
        #ifdef DEBUG
        static uint8_t debug_tick = 0;
        debug_tick++;

        if (debug_tick >= 100) {
    debug_tick = 0;
    uart_print("State="); uart_print_int(state);
    uart_print("Turns="); uart_print_int(turn_count);
    uart_print("IrL_cm="); uart_print_int(ir_to_cm(ir_left)-5);
    uart_print("IrR_cm="); uart_print_int(ir_to_cm(ir_right)-5);
    uart_print("US=");  uart_print_int(us_dist);
    uart_print_float(" Yaw=", yaw_deg);
    uart_print("\r\n");
}

        #endif
 
    } /* end while(1) */
}
 
 
 int main(void) {
    setup();
    loop();
    return 0;
 }
