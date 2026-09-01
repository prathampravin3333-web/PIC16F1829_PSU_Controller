void Set_PWM_Duty(uint8_t duty) {
    uint16_t duty_value;
    uint8_t duty_high;
    uint8_t duty_low;
    
    // Convert 0-100 to 10-bit value
    duty_value = (uint16_t)((((uint16_t)duty) * 1024U) / 100U);
    
    duty_high = (uint8_t)(duty_value >> 2);
    duty_low = (uint8_t)(duty_value & 0x03);
    
    // Set PWM1 on RC5 (HIGH side)
    CCPR1L = duty_high;
    DC1B1 = (duty_low >> 1) & 1;
    DC1B0 = duty_low & 1;
    
    // Set PWM2 on RC4 (LOW side) - SAME duty cycle (via CCP2 complementary mode)
    // The CCP2 module inverts the signal automatically
    CCPR2L = duty_high;
    DC2B1 = (duty_low >> 1) & 1;
    DC2B0 = duty_low & 1;
}
