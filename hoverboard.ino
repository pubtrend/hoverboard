// Getting it to blink!

/*void setup() {
  pinMode(11, OUTPUT);   // D3 LED on your board
}

void loop() {
  digitalWrite(11, HIGH); // LED ON
  delay(500);             // wait 0.5 sec
  digitalWrite(11, LOW);  // LED OFF
  delay(500);             // wait 0.5 sec
}*/

// SECTION DONE


























// GETTING THE IR SENSOR TO WORK
/* 
#define DIST_14CM 450       // set the max ADC for 14 cm
#define DIST_42CM 140       // set the max ADC for 42 cm

void setup() {
  pinMode(11, OUTPUT);      // says which ouput pin we're dealing with (D3 LED)
  Serial.begin(9600);       // allows us to see live output values in the serial monitor in the IDE
}

void loop() {
  int adc = analogRead(A0);   // reads voltage and converts it to ADC                         
  Serial.println(adc);        // prints those live output values

  int brightness = map(adc, DIST_14CM, DIST_42CM, 0, 255);  // sets 
  brightness = constrain(brightness, 0, 255);
  analogWrite(11, 255 - brightness);

  Serial.print("ADC: "); Serial.print(adc);
  Serial.print(" Brightness: "); Serial.println(brightness);

  delay(1000);
}

*/

// SECTION DONE
































// GETTING THE IR SENSOR TO WORK WITH C

/**/
#include <avr/io.h>        // renames registers to PORTB, etc, so we don't need to use hex to find em
#include <util/delay.h>    // allows handy delay function
#include <stdio.h>         // allows sprintf(__) which converts number to a string. Helps w/ printing out ADC etc

#define F_CPU 16000000UL   // This is like initializing but uses no RAM and acts as constant
#define BAUD 9600
#define DIST_14CM 380
#define DIST_42CM 140

// UART init                // allows the arduino to communicate w/ computer and then w/ the serial monitor to show ADC etc THRU THE USB
void uart_init() {
    uint16_t ubrr = F_CPU / 16 / BAUD - 1; // the computer and arduino must match bits per second
    UBRR0H = (ubrr >> 8); 
    UBRR0L = ubrr;
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // making this comment to test git
}

void uart_send(char c) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

void uart_print(const char* s) {
    while (*s) uart_send(*s++);
}

void uart_print_int(int val) {
    char buf[10];
    sprintf(buf, "%d\r\n", val);
    uart_print(buf);
}

// ADC init
void adc_init() {
    ADMUX = (1 << REFS0);  
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); 
}

uint16_t adc_read() {
    ADCSRA |= (1 << ADSC);           
    while (ADCSRA & (1 << ADSC));    
    return ADC;
}

// PWM initialize (PB3, pin 11)
void pwm_init() {
    DDRB |= (1 << PB3);              
    TCCR2A = (1 << COM2A1) | (1 << WGM21) | (1 << WGM20); 
    TCCR2B = (1 << CS21);            
    OCR2A = 0;                       
}

void pwm_set(uint8_t val) {
    OCR2A = val;
}

// map function
int map_val(int x, int in_min, int in_max, int out_min, int out_max) {
    return (long)(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// constrain
int constrain_val(int x, int lo, int hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

int main() {
    uart_init();
    adc_init();
    pwm_init();

    

    while (1) {
        uint16_t adc = adc_read();

        uart_print("ADC: ");
        uart_print_int(adc);

        int brightness = map_val(adc, DIST_14CM, DIST_42CM, 0, 255);
        brightness = constrain_val(brightness, 0, 255);
        pwm_set(255 - brightness);  

        uart_print("Brightness: ");
        uart_print_int(brightness);

        // Flash LED L if outside [d1, d2] range
        // Outside range = ADC > DIST_14CM or ADC < DIST_42CM
        if (adc > DIST_14CM || adc < DIST_42CM) {
            PORTB |= (1 << PB5);   // ON
            _delay_ms(500);
            PORTB &= ~(1 << PB5);  // OFF
            _delay_ms(500);
        } else {
            PORTB &= ~(1 << PB5);  // keep off inside range
        }
    }
}


// SECTION DONE



































// GETTING THE US SENSOR TO WORK.
/*
#define TRIG 13
#define ECHO 8
#define DIST_14CM 850   // microseconds at 14cm (placeholder)
#define DIST_42CM 2355  // microseconds at 42cm (placeholder)

void setup() {
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(11, OUTPUT);
  Serial.begin(9600);
}


void loop() {
  // Send pulse
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  // Read echo
  long duration = pulseIn(ECHO, HIGH);
  
  Serial.print("Duration: "); Serial.println(duration);

  int brightness = map(duration, DIST_14CM, DIST_42CM, 0, 255);
  brightness = constrain(brightness, 0, 255);
  analogWrite(11, 255 - brightness);

  delay(1000);
}
*/

// SECTION DONE



































/*
// GETTING THE US SENSOR TO WORK WITH C
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

#define F_CPU 16000000UL
#define BAUD 9600
#define DIST_14CM 710    // ~14cm in microseconds
#define DIST_42CM 2436   // ~42cm in microseconds

// UART init
void uart_init() {
    uint16_t ubrr = F_CPU / 16 / BAUD - 1;
    UBRR0H = (ubrr >> 8);
    UBRR0L = ubrr;
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_send(char c) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

void uart_print(const char* s) {
    while (*s) uart_send(*s++);
}

void uart_print_int(long val) {
    char buf[12];
    sprintf(buf, "%ld\r\n", val);
    uart_print(buf);
}

// PWM init on OC2A (PB3, pin 11)
void pwm_init() {
    DDRB |= (1 << PB3);
    TCCR2A = (1 << COM2A1) | (1 << WGM21) | (1 << WGM20);
    TCCR2B = (1 << CS21);
    OCR2A = 0;
}

void pwm_set(uint8_t val) {
    OCR2A = val;
}

// Pulse measurement using Timer1
// TRIG = PB5 (pin 13), ECHO = PB0 (pin 8)
long pulse_in() {
    // Send 10us trigger pulse
    DDRB |= (1 << PB5);          
    DDRB &= ~(1 << PB0);         

    PORTB &= ~(1 << PB5);
    _delay_us(2);
    PORTB |= (1 << PB5);
    _delay_us(10);
    PORTB &= ~(1 << PB5);

    // Wait for ECHO to go HIGH
    while (!(PINB & (1 << PB0)));

    // Count microseconds while ECHO is HIGH
    TCNT1 = 0;
    TCCR1B = (1 << CS11);         
    while (PINB & (1 << PB0));
    TCCR1B = 0;                   

    return TCNT1 / 2;             
}

// map function
int map_val(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// constrain
int constrain_val(int x, int lo, int hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

int main() {
    uart_init();
    pwm_init();

    while (1) {
        long duration = pulse_in();

        uart_print("Duration (us): ");
        uart_print_int(duration);

        int brightness = map_val(duration, DIST_42CM, DIST_14CM, 0, 255);
        brightness = constrain_val(brightness, 0, 255);
        pwm_set(brightness);


        int brightness_display = map_val(duration, DIST_14CM, DIST_42CM, 0, 255);
        brightness_display = constrain_val(brightness_display, 0, 255);
        uart_print("Brightness: ");
        uart_print_int(brightness);

        _delay_ms(1000);
    }
}
*/

