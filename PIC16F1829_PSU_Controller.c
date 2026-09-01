/*
 * PIC16F1829 PSU Controller Firmware - PUSH-PULL TOPOLOGY
 * Compiler: MPLAB XC8
 * Device: PIC16F1829
 * 
 * Features:
 * - Push-Pull topology with complementary PWM on RC5 and RC4
 * - Fan speed control via temperature sensing (NTC thermistor)
 * - 187kHz constant frequency output (RC6)
 * - 95V rail monitoring and regulation
 * - 12V battery voltage sensing (UVP/OVP)
 * - 7-second soft start ramp
 * - LED indicators for status and protection
 * - Dead-time management for shoot-through prevention
 */

#include <xc.h>
#include <stdint.h>

// Configuration Bits
#pragma config FOSC = INTOSC    // Internal oscillator
#pragma config WDTE = OFF       // Watchdog timer disabled
#pragma config PWRTE = OFF      // Power-up timer disabled
#pragma config MCLRE = ON       // MCLR pin enabled
#pragma config CP = OFF         // Code protection disabled
#pragma config CPD = OFF        // Data code protection disabled
#pragma config BOREN = ON       // Brown-out reset enabled
#pragma config CLKOUTEN = OFF   // Clock out disabled
#pragma config IESO = ON        // Internal/External switchover enabled
#pragma config FCMEN = ON       // Fail-safe clock monitor enabled
#pragma config LVP = OFF        // Low voltage programming disabled

// Frequency definitions
#define _XTAL_FREQ 8000000     // 8MHz internal oscillator

// Pin Definitions
#define FAN_ON_SIGNAL   RA5    // PIN 2 - Fan ON control signal
#define NTC_SENSOR      RA4    // PIN 3 - Temperature sensor input (AN3)
#define PWM_HIGH        RC5    // PIN 5 - 30kHz PWM HIGH side (CCP1/P1A)
#define PWM_LOW         RC4    // PIN 14- 30kHz PWM LOW side (CCP2/P2A) - Complementary
#define LED_ON          RC3    // PIN 7 - ON indicator LED
#define CONST_187K      RC6    // PIN 8 - 187kHz constant frequency
#define LED_CLIP        RC7    // PIN 9 - Clipping indicator LED
#define LED_PROT        RB7    // PIN 12- Protection LED
#define BATT_12V        RC1    // PIN 25- 12V battery sensing (AN5)
#define AUDIO_SENSE     RA1    // PIN 18- External audio 5V sense (AN1)

// NOTE: PIN 6 (RC2) has NO OUTPUT - Used only for 95V rail sensing input (AN6)
// RC2 is configured as analog input only, no digital output capability

// Temperature Thresholds (ADC values for 10K NTC)
#define TEMP_UNDER_50C   768    // ADC threshold for under 50°C (low speed)
#define TEMP_FULL_SPEED  512    // ADC threshold for full speed (50°C)
#define TEMP_LATCH_90C   256    // ADC threshold for fault latch (90°C)

// Voltage Thresholds (ADC values)
// 12V Rail: 47K to 12V, 10K to GND = 2.0V @ ADC
#define BATT_UVP_10_5V   205    // Under-voltage protection (10.5V)
#define BATT_OVP_14_8V   288    // Over-voltage protection (14.8V)

// 95V Rail: Resistor divider for ADC sensing (INPUT ONLY)
#define RAIL_95V_TARGET  732    // 95V target (ADC value)
#define RAIL_95V_MIN     710    // 92.5V minimum
#define RAIL_95V_MAX     754    // 97.5V maximum

// PWM Parameters for Push-Pull
#define PWM_FREQ_30K    0x53   // PR2 value for 30kHz @ 8MHz
#define SOFT_START_TIME 7000   // 7 seconds in milliseconds
#define PWM_MIN_DUTY    5      // Minimum duty cycle (5% - prevent DC offset)
#define PWM_MAX_DUTY    250    // Maximum duty cycle (98% - leave headroom)
#define DEAD_TIME       8      // Dead time between HIGH and LOW transition

// 187kHz Timer Parameters
#define TIMER1_RELOAD   0xFE0D // Timer1 reload for 187kHz @ 8MHz

// Global Variables
uint16_t adc_temp = 0;
uint16_t adc_rail = 0;
uint16_t adc_batt = 0;
uint16_t adc_audio = 0;
uint8_t pwm_duty = 128;        // 50% nominal for push-pull
uint8_t soft_start_counter = 0;
uint16_t timer_ms = 0;
uint8_t fault_latch = 0;
uint8_t system_state = 0;
int16_t voltage_error_accum = 0;
uint8_t toggle_187k = 0;       // Toggle flag for 187kHz output

#define STATE_OFF           0
#define STATE_SOFT_START    1
#define STATE_NORMAL        2
#define STATE_FAULT         3

// Function Prototypes
void Init_Oscillator(void);
void Init_IO(void);
void Init_ADC(void);
void Init_PWM_PushPull(void);
void Init_187K_Timer(void);
void Init_Timer(void);
void Read_Sensors(void);
void Fan_Speed_Control(void);
void PWM_Soft_Start(void);
void Voltage_Regulation(void);
void LED_Control(void);
void Set_PWM_Duty(uint8_t duty);
void Toggle_187K_Output(void);
void __interrupt() ISR(void);

/*
 * Main Function
 */
void main(void)
{
    Init_Oscillator();
    Init_IO();
    Init_ADC();
    Init_PWM_PushPull();
    Init_187K_Timer();
    Init_Timer();
    
    __delay_ms(100);
    
    system_state = STATE_SOFT_START;
    soft_start_counter = 0;
    pwm_duty = PWM_MIN_DUTY;
    
    while(1)
    {
        Read_Sensors();
        
        // Check battery voltage for protection
        if(adc_batt < BATT_UVP_10_5V || adc_batt > BATT_OVP_14_8V)
        {
            system_state = STATE_FAULT;
            fault_latch = 1;
        }
        
        // Check temperature for fault latch (90°C cutoff)
        if(adc_temp < TEMP_LATCH_90C)
        {
            system_state = STATE_FAULT;
            fault_latch = 1;
        }
        
        if(system_state == STATE_SOFT_START)
        {
            PWM_Soft_Start();
        }
        else if(system_state == STATE_NORMAL)
        {
            Fan_Speed_Control();
            Voltage_Regulation();
        }
        else if(system_state == STATE_FAULT)
        {
            pwm_duty = 128;        // Center PWM to disable power
            Set_PWM_Duty(128);     // Safe state
        }
        
        LED_Control();
        
        __delay_ms(10);
    }
}

/*
 * Oscillator Initialization - 8MHz Internal
 */
void Init_Oscillator(void)
{
    OSCCON = 0x70;      // 8MHz, internal oscillator
    while(!OSCSTATbits.OSTS);  // Wait for stable
}

/*
 * I/O Initialization
 * PIN 6 (RC2) is ANALOG INPUT ONLY - NO DIGITAL OUTPUT
 */
void Init_IO(void)
{
    // TRISA: RA1(input/AN1), RA4(input/AN3), RA5(output)
    TRISA = 0b00110010;
    
    // TRISB: RB7(input-protection), others output
    TRISB = 0b10000000;
    
    // TRISC: RC1(input/AN5), RC2(input/AN6-NO OUTPUT), RC3-7(output/PWM)
    // RC2 is configured as analog input only
    TRISC = 0b00000110;
    
    // Analog inputs configuration
    ANSELA = 0b00010010; // RA1, RA4 analog
    ANSELB = 0b00000000;
    ANSELC = 0b00000110; // RC1, RC2 analog (RC2 has NO digital output)
    
    // Initialize outputs to safe state
    PORTA = 0x00;
    PORTB = 0x00;
    PORTC = 0x00;
}

/*
 * ADC Initialization
 */
void Init_ADC(void)
{
    ADCON0 = 0b00000011;  // AN3 (RA4) selected initially
    ADCON1 = 0b00010000;  // Right justified, FOSC/8
    ADCON2 = 0x00;
    
    ADIE = 1;             // Enable ADC interrupt
    PEIE = 1;
    GIE = 1;
}

/*
 * PWM Initialization for Push-Pull Topology
 * RC5 (CCP1) = PWM_HIGH
 * RC4 (CCP2) = PWM_LOW (Complementary)
 * Both driven at 30kHz
 */
void Init_PWM_PushPull(void)
{
    // CCP1 on RC5 - PWM HIGH side
    CCP1CON = 0x0C;           // PWM mode
    CCP1SEbits.P1SE = 1;      // P1 (single output) enabled
    CCP1SEbits.P1A = 1;       // P1A output on RC5
    
    // CCP2 on RC4 - PWM LOW side (complementary)
    CCP2CON = 0x0C;           // PWM mode
    CCP2SEbits.P2SE = 1;      // P2 (single output) enabled
    CCP2SEbits.P2A = 1;       // P2A output on RC4
    
    // Timer2 configuration for 30kHz (PWM frequency)
    // Fosc = 8MHz, Prescale = 1:1, PR2 = 0x53
    T2CON = 0b00000100;       // Timer2 ON, 1:1 prescaler
    PR2 = PWM_FREQ_30K;       // 30kHz base frequency
    TMR2 = 0;
    T2IE = 1;
    
    // Initialize duty cycles (50% nominal for push-pull)
    CCPR1L = 128;
    CCPR2L = 128;
}

/*
 * Initialize 187kHz Output on RC6 using Timer1
 * RC6 toggled in interrupt for 187kHz @ 50% duty
 */
void Init_187K_Timer(void)
{
    TRISC6 = 0;                // RC6 as output
    PORTC6 = 0;               // Start LOW
    
    // Timer1 Configuration for 187kHz toggle timing
    // For 187kHz square wave: toggle every ~2.67us
    // Timer1: 16-bit, 1:1 prescaler, internal clock
    T1CON = 0b00000000;        // Timer1 OFF, 16-bit, 1:1 prescaler
    TMR1H = (TIMER1_RELOAD >> 8);  // Reload value high byte
    TMR1L = (TIMER1_RELOAD & 0xFF); // Reload value low byte
    
    T1IE = 1;                  // Enable Timer1 interrupt
    GIE = 1;                   // Global interrupt enable
}

/*
 * Timer Initialization (1ms tick for timing)
 */
void Init_Timer(void)
{
    // Timer0: 8-bit, 1:64 prescaler
    T0CON = 0b11000101;       // Timer0 ON, 8-bit, 1:64 prescaler
    TMR0 = 0;
    T0IE = 1;
    GIE = 1;
}

/*
 * Read all Sensors via ADC
 */
void Read_Sensors(void)
{
    static uint8_t channel = 0;
    uint16_t adc_result = 0;
    
    // Round-robin ADC reading
    switch(channel)
    {
        case 0: // AN6 (RC2) - 95V Rail (ANALOG INPUT ONLY)
            ADCON0bits.CHS = 0b00110;
            break;
        case 1: // AN5 (RC1) - 12V Battery
            ADCON0bits.CHS = 0b00101;
            break;
        case 2: // AN3 (RA4) - Temperature (NTC)
            ADCON0bits.CHS = 0b00011;
            break;
        case 3: // AN1 (RA1) - Audio Sense
            ADCON0bits.CHS = 0b00001;
            break;
    }
    
    ADCON0bits.GO = 1;        // Start ADC conversion
    while(ADCON0bits.GO);     // Wait for completion
    
    adc_result = (ADRESH << 8) | ADRESL;
    adc_result = adc_result >> 2;  // Convert to 10-bit
    
    // Store sensor values
    switch(channel)
    {
        case 0:
            adc_rail = adc_result;  // 95V rail voltage (INPUT ONLY - NO OUTPUT)
            break;
        case 1:
            adc_batt = adc_result;  // 12V battery voltage
            break;
        case 2:
            adc_temp = adc_result;  // Temperature from NTC
            break;
        case 3:
            adc_audio = adc_result; // Audio sense
            break;
    }
    
    channel = (channel + 1) % 4;
}

/*
 * Fan Speed Control Based on Temperature (RA5 pin control)
 */
void Fan_Speed_Control(void)
{
    uint8_t fan_speed = 0;
    
    // Check FAN_ON_SIGNAL from RA5
    if(FAN_ON_SIGNAL == 0)
    {
        fan_speed = 0;  // Fan disabled
    }
    else
    {
        // Temperature-based speed control
        if(adc_temp > TEMP_FULL_SPEED)
        {
            // Between 50°C and 90°C - linear ramp
            if(adc_temp > TEMP_LATCH_90C)
            {
                fan_speed = 100;  // Full speed
            }
            else
            {
                // Linear interpolation from 50% to 100%
                fan_speed = 50 + ((TEMP_FULL_SPEED - adc_temp) * 50 / 
                            (TEMP_FULL_SPEED - TEMP_LATCH_90C));
            }
        }
        else if(adc_temp >= TEMP_UNDER_50C)
        {
            // Under 50°C - low speed
            fan_speed = 30;
        }
    }
    
    // Map fan speed (0-100%) to PWM duty (50-250 for push-pull)
    pwm_duty = 128 + (fan_speed * 122 / 100);
}

/*
 * PWM Soft Start - 7 Second Ramp
 */
void PWM_Soft_Start(void)
{
    uint16_t max_soft_start_ticks = (SOFT_START_TIME / 10);
    
    if(soft_start_counter < max_soft_start_ticks)
    {
        // Linear ramp from center (128) to target
        uint8_t target_duty = 128 + ((PWM_MAX_DUTY - 128) * soft_start_counter / max_soft_start_ticks);
        Set_PWM_Duty(target_duty);
        soft_start_counter++;
    }
    else
    {
        system_state = STATE_NORMAL;
    }
}

/*
 * 95V Rail Regulation with PI Controller
 * Maintains steady 95V by adjusting PWM duty
 */
void Voltage_Regulation(void)
{
    int16_t error;
    int16_t correction;
    
    // Calculate error from target
    error = RAIL_95V_TARGET - adc_rail;
    
    // PI controller (Proportional + Integral)
    voltage_error_accum += error;
    
    // Clamp accumulator to prevent windup
    if(voltage_error_accum > 5000) voltage_error_accum = 5000;
    if(voltage_error_accum < -5000) voltage_error_accum = -5000;
    
    // Calculate correction: P term + I term
    correction = (error >> 3) + (voltage_error_accum >> 8);
    
    // Apply correction to base duty
    int16_t new_duty = (int16_t)pwm_duty + correction;
    
    // Clamp within safe limits
    if(new_duty > PWM_MAX_DUTY) new_duty = PWM_MAX_DUTY;
    if(new_duty < PWM_MIN_DUTY) new_duty = PWM_MIN_DUTY;
    
    pwm_duty = (uint8_t)new_duty;
    Set_PWM_Duty(pwm_duty);
}

/*
 * Set PWM Duty Cycle for Push-Pull Topology
 */
void Set_PWM_Duty(uint8_t duty)
{
    uint8_t pwm_high, pwm_low;
    
    // Ensure complementary outputs with dead time
    pwm_high = duty;
    pwm_low = (255 - duty);
    
    // Add dead time by reducing overlap
    if(pwm_high > 0) pwm_high -= DEAD_TIME;
    if(pwm_low > 0) pwm_low -= DEAD_TIME;
    
    // Clamp to valid range
    if(pwm_high > PWM_MAX_DUTY) pwm_high = PWM_MAX_DUTY;
    if(pwm_low > PWM_MAX_DUTY) pwm_low = PWM_MAX_DUTY;
    
    CCPR1L = pwm_high;    // HIGH side (RC5)
    CCPR2L = pwm_low;     // LOW side (RC4)
}

/*
 * Toggle 187kHz Output on RC6
 */
void Toggle_187K_Output(void)
{
    RC6 = !RC6;  // Toggle RC6 for 187kHz square wave
}

/*
 * LED Control and Status Indication
 */
void LED_Control(void)
{
    // RC3: ON indicator
    LED_ON = (system_state != STATE_OFF && system_state != STATE_FAULT) ? 1 : 0;
    
    // RB7: Protection/Fault indicator
    LED_PROT = (fault_latch || system_state == STATE_FAULT) ? 1 : 0;
    
    // RC7: Clipping indicator (95V rail too high)
    LED_CLIP = (adc_rail > RAIL_95V_MAX) ? 1 : 0;
}

/*
 * Interrupt Service Routine
 */
void __interrupt() ISR(void)
{
    // Timer0 interrupt - 1ms tick
    if(T0IE && T0IF)
    {
        T0IF = 0;
        timer_ms++;
    }
    
    // Timer1 interrupt - 187kHz output toggle (RC6)
    if(T1IE && T1IF)
    {
        T1IF = 0;
        
        // Reload Timer1 for next toggle
        TMR1H = (TIMER1_RELOAD >> 8);
        TMR1L = (TIMER1_RELOAD & 0xFF);
        
        // Toggle RC6 for 187kHz square wave @ 50% duty
        Toggle_187K_Output();
    }
    
    // ADC interrupt
    if(ADIE && ADIF)
    {
        ADIF = 0;
    }
    
    // Timer2 interrupt - PWM timing
    if(T2IE && T2IF)
    {
        T2IF = 0;
    }
}

/*
 * End of File
 * 
 * PIN CONFIGURATION SUMMARY:
 * PIN 2 (RA5): FAN ON SIGNAL - Digital Output
 * PIN 3 (RA4): NTC Temperature Sensor - Analog Input
 * PIN 5 (RC5): 30kHz PWM High Side - Output
 * PIN 6 (RC2): 95V Rail Sense - ANALOG INPUT ONLY (NO OUTPUT)
 * PIN 7 (RC3): ON LED - Digital Output
 * PIN 8 (RC6): 187kHz Constant - Digital Output
 * PIN 9 (RC7): Clip LED - Digital Output
 * PIN 12 (RB7): Protection LED - Digital Output
 * PIN 14 (RC4): 30kHz PWM Low Side (Inverted) - Output
 * PIN 18 (RA1): External Audio 5V Sense - Analog Input
 * PIN 25 (RC1): 12V Battery Sense - Analog Input
 * 
 * PUSH-PULL TOPOLOGY NOTES:
 * - RC5 and RC4 are complementary outputs (180° out of phase)
 * - Dead time inserted between transitions to prevent shoot-through
 * - PWM duty varies around 50% center point (128)
 * - 187kHz on RC6 generated via Timer1 interrupt toggle
 */
