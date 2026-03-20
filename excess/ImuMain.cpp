#include <stdint.h>
#include <avr/wdt.h>

// Define flags_t struct for TWI linkage
typedef struct {
    uint8_t TX_new_data:1;
    uint8_t TX_finished:1;
    uint8_t TX_buffer1_empty:1;
    uint8_t TX_buffer2_empty:1;
    uint8_t RX_flag:3;
    uint8_t TWI_ACK:1;
} flags_t;

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "init_290.h"
#include "TWI_290.h"

/*
 * ENGR290 Technical Assignment #2
 * IMU: MPU-6050 on P7
 * Servo: P9 (PB1 / OC1A)
 * D3 brightness: PB3 / OC2A, active LOW
 * LED L: assumed PB5
 */

/* ---------------- Experiment settings ---------------- */
#define LOOP_DT_MS              5.0f
#define MPU_GYRO_FS_SEL         0
#define MPU_ACCEL_FS_SEL        0
#define MPU_DLPF_CFG            3
#define MPU_SMPLRT_DIV          9
#define YAW_LIMIT_DEG           85.0f
#define X_LED_OFF_G             0.08f
#define X_LED_FULL_G            1.08f
#define X_DEADBAND_G            0.03f
#define DIST_GAIN               1.00f
#define G_MPS2                  9.80665f

/* ---------------- LED L configuration ---------------- */
#define LEDL_DDR                DDRB
#define LEDL_PORT               PORTB
#define LEDL_PIN                PB5
#define LEDL_ACTIVE_HIGH        1

/* ---------------- MPU-6050 registers ---------------- */
#define MPU6050_ADDR            0x68
#define REG_SMPLRT_DIV          0x19
#define REG_CONFIG              0x1A
#define REG_GYRO_CONFIG         0x1B
#define REG_ACCEL_CONFIG        0x1C
#define REG_ACCEL_XOUT_H        0x3B
#define REG_PWR_MGMT_1          0x6B
#define REG_WHO_AM_I            0x75

extern const uint16_t Servo_angle[256];

volatile flags_t flags = { .TWI_ACK = 1 }; /* TWI_ACK must start 1; Write_Reg checks it after writes even though only reads update it */
volatile uint8_t TWI_status = 0;
volatile uint8_t TWI_byte = 0;

static float gyro_lsb_per_dps = 131.0f;
static float accel_lsb_per_g  = 16384.0f;

static float gyro_bias_x_dps = 0.0f;
static float gyro_bias_y_dps = 0.0f;
static float gyro_bias_z_dps = 0.0f;
static float accel_bias_x_g  = 0.0f;

static float roll_deg  = 0.0f;
static float pitch_deg = 0.0f;
static float yaw_deg   = 0.0f;

static float ax_g = 0.0f;
static float ay_g = 0.0f;
static float az_g = 0.0f;
static float gx_dps = 0.0f;
static float gy_dps = 0.0f;
static float gz_dps = 0.0f;

static float ax_lin_g  = 0.0f;
static float vel_x_mps = 0.0f;
static float dist_x_m  = 0.0f;

/* ---------------- UART helpers ---------------- */
static void uart_putc(char c) {
    while (!(UCSR0A & (1 << UDRE0))) {
    }
    UDR0 = (uint8_t)c;
}

static void uart_print(const char *s) {
    while (*s) {
        uart_putc(*s++);
    }
}

static void uart_print_crlf(void) {
    uart_print("\r\n");
}

static void uart_print_u8(uint8_t value) {
    char buf[6];
    itoa(value, buf, 10);
    uart_print(buf);
}

static void uart_print_float(const char *label, float value) {
    char buf[24];
    uart_print(label);
    dtostrf(value, 0, 2, buf);
    uart_print(buf);
}

/* ---------------- Generic helpers ---------------- */
static float clampf_local(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float gyro_sensitivity_from_fs(uint8_t fs_sel) {
    switch (fs_sel & 0x03) {
        case 0: return 131.0f;
        case 1: return 65.5f;
        case 2: return 32.8f;
        default: return 16.4f;
    }
}

static float accel_sensitivity_from_fs(uint8_t fs_sel) {
    switch (fs_sel & 0x03) {
        case 0: return 16384.0f;
        case 1: return 8192.0f;
        case 2: return 4096.0f;
        default: return 2048.0f;
    }
}

static int16_t be16_to_i16(uint8_t hi, uint8_t lo) {
    return (int16_t)((((uint16_t)hi) << 8) | lo);
}

/* ---------------- LED / PWM helpers ---------------- */
static void led_l_init(void) {
    LEDL_DDR |= (1 << LEDL_PIN);
#if LEDL_ACTIVE_HIGH
    LEDL_PORT &= ~(1 << LEDL_PIN);
#else
    LEDL_PORT |= (1 << LEDL_PIN);
#endif
}

static void led_l_on(void) {
#if LEDL_ACTIVE_HIGH
    LEDL_PORT |= (1 << LEDL_PIN);
#else
    LEDL_PORT &= ~(1 << LEDL_PIN);
#endif
}

static void led_l_off(void) {
#if LEDL_ACTIVE_HIGH
    LEDL_PORT &= ~(1 << LEDL_PIN);
#else
    LEDL_PORT |= (1 << LEDL_PIN);
#endif
}

static void timer2_d3_init(void) {
    /* OC2A -> PB3 -> D3 */
    TCCR2A = 0;
    TCCR2B = 0;
    TCCR2A |= (1 << COM2A1);
    TCCR2A |= (1 << WGM20);
    TCCR2B |= (1 << CS22);
    OCR2A = 255;
}

static void set_d3_brightness(uint8_t brightness) {
    OCR2A = (uint8_t)(255U - brightness);
}

static uint8_t angle_deg_to_servo_index(float angle_deg) {
    float limited = clampf_local(angle_deg, -90.0f, 90.0f);
    float index = ((limited + 90.0f) * 255.0f) / 180.0f;
    if (index < 0.0f) index = 0.0f;
    if (index > 255.0f) index = 255.0f;
    return (uint8_t)(index + 0.5f);
}

static void set_servo_from_yaw(float yaw) {
    float limited_yaw = clampf_local(-yaw, -YAW_LIMIT_DEG, YAW_LIMIT_DEG);
    uint8_t table_index = angle_deg_to_servo_index(limited_yaw);
    OCR1A = Servo_angle[table_index];
}

/* ---------------- TWI / MPU-6050 helpers ---------------- */
static uint8_t mpu_write(uint8_t reg, uint8_t value) {
    return Write_Reg(MPU6050_ADDR, reg, value);
}

static uint8_t mpu_read(uint8_t reg, uint8_t *value) {
    uint8_t status = Read_Reg(MPU6050_ADDR, reg);
    if (status) return status;
    *value = TWI_byte;
    return 0;
}

static uint8_t mpu_read_burst(uint8_t start_reg, uint8_t *buf, uint8_t count) {
    /* TWI_290.h declares int16_t data (by value) but TWI_290.c treats it as
       int16_t* internally. Pass buf's address cast to int16_t so the function
       writes all count bytes directly into buf. On AVR, pointers are 16-bit =
       same width as int16_t, so this cast is safe. */
    return Read_Reg_N(MPU6050_ADDR, start_reg, count, (int16_t)(uint16_t)buf);
}

static uint8_t mpu_init(void) {
    uint8_t who_am_i = 0;

    gyro_lsb_per_dps = gyro_sensitivity_from_fs(MPU_GYRO_FS_SEL);
    accel_lsb_per_g  = accel_sensitivity_from_fs(MPU_ACCEL_FS_SEL);

    _delay_ms(50);

    if (mpu_write(REG_PWR_MGMT_1, 0x80)) return 1;
    _delay_ms(100);
    if (mpu_write(REG_PWR_MGMT_1, 0x01)) return 2;
    _delay_ms(10);

    if (mpu_write(REG_SMPLRT_DIV, MPU_SMPLRT_DIV)) return 3;
    if (mpu_write(REG_CONFIG, MPU_DLPF_CFG & 0x07)) return 4;
    if (mpu_write(REG_GYRO_CONFIG,  (uint8_t)((MPU_GYRO_FS_SEL  & 0x03) << 3))) return 5;
    if (mpu_write(REG_ACCEL_CONFIG, (uint8_t)((MPU_ACCEL_FS_SEL & 0x03) << 3))) return 6;

    if (mpu_read(REG_WHO_AM_I, &who_am_i)) return 7;
    if ((who_am_i & 0x7EU) != 0x68U) return 8;

    return 0;
}

static uint8_t mpu_read_scaled(void) {
    uint8_t raw[14];
    int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;

    if (mpu_read_burst(REG_ACCEL_XOUT_H, raw, 14)) {
        return 1;
    }

    ax_raw = be16_to_i16(raw[0], raw[1]);
    ay_raw = be16_to_i16(raw[2], raw[3]);
    az_raw = be16_to_i16(raw[4], raw[5]);
    gx_raw = be16_to_i16(raw[8], raw[9]);
    gy_raw = be16_to_i16(raw[10], raw[11]);
    gz_raw = be16_to_i16(raw[12], raw[13]);

    ax_g = ((float)ax_raw) / accel_lsb_per_g;
    ay_g = ((float)ay_raw) / accel_lsb_per_g;
    az_g = ((float)az_raw) / accel_lsb_per_g;
    gx_dps = ((float)gx_raw) / gyro_lsb_per_dps;
    gy_dps = ((float)gy_raw) / gyro_lsb_per_dps;
    gz_dps = ((float)gz_raw) / gyro_lsb_per_dps;

    return 0;
}

static void mpu_calibrate_still(void) {
    const uint16_t samples = 500;
    uint16_t good_samples = 0;
    float sum_gx = 0.0f;
    float sum_gy = 0.0f;
    float sum_gz = 0.0f;
    float sum_ax = 0.0f;

    uart_print("Keep IMU still for calibration...");
    uart_print_crlf();
    _delay_ms(500);

    for (uint16_t i = 0; i < samples; i++) {
        if (!mpu_read_scaled()) {
            sum_gx += gx_dps;
            sum_gy += gy_dps;
            sum_gz += gz_dps;
            sum_ax += ax_g;
            good_samples++;
        }
        _delay_ms(5);
    }

    if (good_samples == 0U) {
        good_samples = 1U;
    }

    gyro_bias_x_dps = sum_gx / good_samples;
    gyro_bias_y_dps = sum_gy / good_samples;
    gyro_bias_z_dps = sum_gz / good_samples;
    accel_bias_x_g  = sum_ax / good_samples;

    roll_deg = 0.0f;
    pitch_deg = 0.0f;
    yaw_deg = 0.0f;
    vel_x_mps = 0.0f;
    dist_x_m = 0.0f;

    uart_print("Calibration complete.");
    uart_print_crlf();
}

/* ---------------- Assignment logic ---------------- */
static void update_orientation_and_distance(float dt_s) {
    float acc_roll_deg;
    float acc_pitch_deg;
    float ax_lin_mps2;

    acc_roll_deg  = atan2f(ay_g, az_g) * (180.0f / 3.14159265f);
    acc_pitch_deg = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * (180.0f / 3.14159265f);

    roll_deg  = 0.98f * (roll_deg  + (gx_dps - gyro_bias_x_dps) * dt_s) + 0.02f * acc_roll_deg;
    pitch_deg = 0.98f * (pitch_deg + (gy_dps - gyro_bias_y_dps) * dt_s) + 0.02f * acc_pitch_deg;
    yaw_deg  += (gz_dps - gyro_bias_z_dps) * dt_s;

    if (yaw_deg > 180.0f) yaw_deg -= 360.0f;
    if (yaw_deg < -180.0f) yaw_deg += 360.0f;

    ax_lin_g = ax_g - accel_bias_x_g;

    if (fabsf(ax_lin_g) < X_DEADBAND_G) {
        ax_lin_g = 0.0f;
        vel_x_mps *= 0.98f;
        if (fabsf(vel_x_mps) < 0.01f) {
            vel_x_mps = 0.0f;
        }
    }

    ax_lin_mps2 = ax_lin_g * G_MPS2 * DIST_GAIN;
    vel_x_mps += ax_lin_mps2 * dt_s;
    dist_x_m  += vel_x_mps * dt_s;
}

static void update_outputs(void) {
    float abs_yaw = fabsf(yaw_deg);
    float abs_ax  = fabsf(ax_lin_g);
    uint8_t d3_value = 0;

    set_servo_from_yaw(yaw_deg);

    if (fabsf(yaw_deg) > YAW_LIMIT_DEG) {
    PORTB |= (1 << PB5);    // LED ON
} else {
    PORTB &= ~(1 << PB5);   // LED OFF
}

    if (abs_ax < X_LED_OFF_G) {
        d3_value = 0;
    } else if (abs_ax > X_LED_FULL_G) {
        d3_value = 255;
    } else {
        d3_value = (uint8_t)(255.0f * (abs_ax - X_LED_OFF_G) / (X_LED_FULL_G - X_LED_OFF_G));
    }

    set_d3_brightness(d3_value);
}

static void print_status_once_per_second(void) {
    uart_print_float("Roll[deg]=", roll_deg);
    uart_print("  ");
    uart_print_float("Pitch[deg]=", pitch_deg);
    uart_print("  ");
    uart_print_float("Yaw[deg]=", yaw_deg);
    uart_print("  ");
    uart_print_float("Ax[g]=", ax_g);
    uart_print("  ");
    uart_print_float("Ay[g]=", ay_g);
    uart_print("  ");
    uart_print_float("Az[g]=", az_g);
    uart_print("  ");
    uart_print_float("Xdist[cm]=", dist_x_m * 100.0f);
    uart_print_crlf();
}

int main(void) {
    MCUSR = 0;
    wdt_disable();
    uint8_t init_status;
    uint16_t print_timer_ms = 0U;
    const float dt_s = LOOP_DT_MS / 1000.0f;

    gpio_init();

    /* Power on ALL sensor connectors immediately after gpio_init */
    PORTD |= (1 << PD7) | (1 << PD6) | (1 << PD5) | (1 << PD4);

    /* Custom UART TX init — polling only, NO TXCIE interrupt (avoids MCU reset on TX) */
    UBRR0H = 0;
    UBRR0L = 103; /* 9600 baud @ 16 MHz: (16e6/(16*9600))-1 = 103 */
    UCSR0B = (1 << TXEN0);
    UCSR0C = (3 << UCSZ00); /* 8N1 */

    led_l_init();
    DDRB |= (1 << PB5);
    PORTB &= ~(1 << PB5);

    PORTC |= (1 << PC4) | (1 << PC5); /* SDA/SCL pull-ups */

    timer1_50Hz_init(0);
    timer2_d3_init();

    /* I2C bus-clear: clock SCL 9x with SDA high to unstick any device
       left mid-transaction by a previous MCU reset, then generate STOP */
    DDRC  |=  (1 << PC5) | (1 << PC4);   /* SCL, SDA as outputs  */
    PORTC |=  (1 << PC5) | (1 << PC4);   /* both HIGH            */
    for (uint8_t i = 0; i < 9; i++) {
        PORTC &= ~(1 << PC5);             /* SCL LOW  */
        _delay_us(5);
        PORTC |=  (1 << PC5);             /* SCL HIGH */
        _delay_us(5);
    }
    PORTC &= ~(1 << PC4);                 /* SDA LOW while SCL HIGH */
    _delay_us(5);
    PORTC |=  (1 << PC4);                 /* SDA HIGH = STOP condition */
    _delay_us(5);
    DDRC  &= ~((1 << PC5) | (1 << PC4)); /* release back to inputs  */
    PORTC |=  (1 << PC5) | (1 << PC4);   /* re-enable pull-ups      */

    twi_init();
    _delay_ms(300); /* allow sensors to fully power up before I2C */

    set_servo_from_yaw(0.0f);
    set_d3_brightness(0U);
    led_l_off();

    uart_print("ENGR290 Technical Assignment #2\r\n");
    uart_print("MPU-6050 on P7, servo on P9, UART 9600 8N1\r\n");

    init_status = mpu_init();
    if (init_status != 0U) {
        uart_print("MPU6050 init failed, code=");
        uart_print_u8(init_status);
        uart_print(" TWI_st=");
        uart_print_u8(TWI_status); /* 1=START fail, 2=SLA+W NACK */
        uart_print_crlf();
        led_l_on();
        while (1) {
            _delay_ms(200);
        }
    }

    mpu_calibrate_still();
    uart_print("Running...");
    uart_print_crlf();

    uint16_t fail_count = 0;

    while (1) {
        if (!mpu_read_scaled()) {
            fail_count = 0;
            update_orientation_and_distance(dt_s);
            update_outputs();
        } else {
            fail_count++;
            if (fail_count >= 200U) {
                fail_count = 0;
                uart_print("IMU read FAILING - check I2C wiring\r\n");
                /* Rapid blink to show IMU error visually */
                PORTB ^= (1 << PB5);
            }
        }

        print_timer_ms += (uint16_t)LOOP_DT_MS;
        if (print_timer_ms >= 1000U) {
            print_timer_ms = 0U;
            print_status_once_per_second();
        }

        _delay_ms((uint8_t)LOOP_DT_MS);
    }
}