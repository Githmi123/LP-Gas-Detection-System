// For Physical Implementation

#include <xc.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#define _XTAL_FREQ 20000000  // Crystal Oscillator Frequency : Used for accurate delays, serves as a precise time base 

// Configuration settings
#pragma config FOSC = HS       // HS oscillator : Time-sensitive application, high oscillator frequency
#pragma config WDTE = OFF      // Watchdog timer disabled : Avoid unnecessary resets when stuck in an infinite loop or encounters a fault.
#pragma config PWRTE = ON      // Power-up timer enabled : For Power stability. Microcontroller waits for a specified time after power-up before executing any instructions.
#pragma config BOREN = ON      // Brown-out reset enabled : To prevent unpredictable manner during low-voltage conditions. Microcontroller resets when the supply voltage drops below a certain threshold.
#pragma config LVP = OFF       // Low-voltage programming disabled : Turns off the ability to program the microcontroller with a low voltage.
#pragma config CPD = OFF       // Data memory code protection disabled : Full access to data memory for reading and writing, useful during development.
#pragma config WRT = OFF       // Flash write protection disabled : Change its own program memory, useful during development to update the software.
#pragma config CP = OFF        // Flash code protection disabled : Allows external devices to read and write the entire program memory.


// Control pins for the LCD
#define RS PORTDbits.RD0
#define RW PORTDbits.RD1
#define EN PORTDbits.RD2

// Digital output pins
#define BUZZER_PIN PORTDbits.RD3

//#define SERVO_PIN PORTDbits.RD4

// Range of ADC values
#define MIN_ADC_VALUE 0
#define MAX_ADC_VALUE 1023

// HX711 module pins
#define HX711_DT PORTCbits.RC0
#define HX711_SCK PORTCbits.RC1

// Range of weight sensor values
#define MIN_WEIGHT_VALUE 0
#define MAX_WEIGHT_VALUE 200000 // max weight

#define RO_VALUE_CLEAN_AIR 9.83  // Sensor resistance in clean air/RO

#define THRESHOLD 500 //Threshold value for gas detection.


float MQ6_curve[3] = {2.3, 0.30, -0.41}; // Curve parameters from the project, 
// coefficients for the logarithmic equation used to convert the sensor's analog output into a gas concentration

#define RL_VALUE 20     //define the load resistance on the board, in kilo ohms


void lcd_data(unsigned char data) // Sends one character at a time
{
    
    PORTB = data;
    RS = 1; // data being sent is actual character data
    RW = 0; // write operation
    EN = 1; // tells the LCD that data is being sent
    __delay_ms(5); // TO ensure that the LCD has enough time to read the data
    EN = 0; // Indicates that the data transfer process is over

}



void lcd_command(unsigned char cmd)
{
    
    PORTB = cmd;
    RS = 0;
    RW = 0;
    EN = 1;
    __delay_ms(5);
    EN = 0;
}



void lcd_string(const unsigned char *str, unsigned char num) // To send a string of characters to the LCD
{
    
    unsigned char i;
    for (i=0;i<num;i++)
    {
        lcd_data(str[i]);
    }
    
}




void lcd_initialise() // to initialize the LCD display with specific configuration settings by sending a command to the LCD controller
{
    lcd_command(0x38); // sets the LCD to 2-line mode and 5x7 dot format to represent a single character
    lcd_command(0x06); // sets the cursor to automatically increment to the right after each character is displayed
    lcd_command(0x0C); // sets the display to be on and shows the cursor as an underscore (no blinking)
    lcd_command(0x01); // Clears the display
}



void initADC(void) {
    ADCON0 = 0b00000001; // Select AN0 as the input channel for gas sensor, ADC on - to configure
    ADCON1 = 0b10001110; // Configure AN0 and AN1 as analog inputs, others as digital
    ADRESH = 0; // Clear ADC result high register
    ADRESL = 0; // Clear ADC result low register
    ADCON0bits.ADON = 1; // Turn on ADC module
}


unsigned int readGasSensorPot(void) {
    ADCON0bits.CHS = 0; // Select AN0
    __delay_us(25); // Acquisition time to charge hold capacitor
    ADCON0bits.GO = 1; // Start conversion
    while (ADCON0bits.GO); // Wait for conversion to complete
    unsigned int sensorValue = ((ADRESH << 8) + ADRESL); // Get the 10-bit ADC result
    return sensorValue;
}



unsigned int getPPMValue(unsigned int sensorValue, float *curve)
{
    float rs = (((1023.0 * RL_VALUE) / sensorValue) - RL_VALUE); 
    float rs_r0_ratio = rs / RO_VALUE_CLEAN_AIR;
    float num = ((log10(rs_r0_ratio) - curve[1]) / curve[2]) + curve[0];
    unsigned int ppm = 1;
    if (num > 0) {
        for (int i = 0; i < (int)num; i++) {
            ppm *= 10;
        }
    } 
    
    return ppm;
}

void displayGasConcentration(unsigned int adcValue) { // Display the ADC value on the LCD
    
    unsigned int ppm = getPPMValue(adcValue, MQ6_curve);
    lcd_command(0x01); // Clear display
    __delay_ms(5); // Delay to allow LCD to clear
    lcd_command(0x80); // Set cursor to the first line
    char concentrationStr[10];
    sprintf(concentrationStr, "%u PPM", ppm); // Format ADC value as a string
    lcd_string(concentrationStr, strlen(concentrationStr)); // Display ADC value on LCD
}



void initUART(void) {
    TRISCbits.TRISC6 = 0; // TX pin as output
    TRISCbits.TRISC7 = 1; // RX pin as input
    SPBRG = 32; // Baud rate 9600 for 20MHz
    TXSTAbits.TXEN = 1; // Enable transmission
    RCSTAbits.SPEN = 1; // Enable serial port
    RCSTAbits.CREN = 1; // Enable continuous receive
}

void sendUART(char data) {
    while (!TXSTAbits.TRMT); // Wait until the transmit shift register is empty
    TXREG = data; // Load the data into the transmit register
}

char readUART(void) {
    while (!PIR1bits.RCIF); // Wait until data is received (Receive Interrupt Flag is set)
    return RCREG;
}


void sendSMS(char* phoneNumber, char* message) {
    // Send AT command to set SMS mode to text
    sendUART('A');
    sendUART('T');
    sendUART('+');
    sendUART('C');
    sendUART('M');
    sendUART('G');
    sendUART('F');
    sendUART('=');
    sendUART('1');
    sendUART('\r');
    sendUART('\n');
    __delay_ms(1000); // Wait for the command to be processed

    // Send AT command to set the recipient phone number
    sendUART('A');
    sendUART('T');
    sendUART('+');
    sendUART('C');
    sendUART('M');
    sendUART('G');
    sendUART('S');
    sendUART('=');
    sendUART('\"');
    while (*phoneNumber) {
        sendUART(*phoneNumber++);
    }
    sendUART('\"');
    sendUART('\r');
    sendUART('\n');
    __delay_ms(1000); // Wait for the command to be processed

    // Send the actual message
    while (*message) {
        sendUART(*message++);
    }

    // Send Ctrl+Z to indicate the end of the message
    sendUART(26); // ASCII code for Ctrl+Z
    __delay_ms(1000); // Wait for the message to be sent
}

// PWM (Pulse Width Modulation) module of a PIC microcontroller for generating PWM signals
void initPWM(void) {
    PR2 = 6249;                 // Set PWM period (20ms for 50Hz)
    T2CON = 0b00000111;        // Enable Timer2
    CCP1CON = 0b00001100;      // Configure CCP1 module for PWM mode
    CCPR1L = 0x0F; // sets the initial duty cycle
}

void setServoAngle(int angle) {
    float duty_cycle;

    // Convert angle to duty cycle (1ms to 2ms pulse)
    duty_cycle = ((float)(angle + 90) / 180.0) * 1.0 + 1.0;

    // Convert the duty cycle to a value that fits within the 8-bit resolution of the PWM register
    unsigned int pwmValue = (unsigned int)(duty_cycle * (_XTAL_FREQ / (4.0 * 16.0 * 50.0)) - 1.0);

    // Set the PWM duty cycle (CCPR1L register and CCP1CON<5:4> bits)
    CCPR1L = (pwmValue >> 2) & 0xFF;
    CCP1CON = (CCP1CON & 0xCF) | ((pwmValue & 0x03) << 4);
}




long HX711_Read(void) {
    long count = 0; // store the 24-bit digital value read from the HX711 module
    unsigned char i;
    HX711_DT = 1; // signaling the start of a communication sequence (data pin)
    HX711_SCK = 0; // (clock pin)
    while (HX711_DT); // Waits until the data pin (HX711_DT) of the HX711 module becomes 0, indicating that the module is ready to send data.
    for (i = 0; i < 24; i++) {
        HX711_SCK = 1; // Sets the clock pin (HX711_SCK) of the HX711 module to 1 for the next bit.
        count = count << 1;
        HX711_SCK = 0;
        if (HX711_DT) count++;
    }
    HX711_SCK = 1;
    count = count ^ 0x800000; // converting the 24-bit two's complement value to a signed 32-bit integer.
    HX711_SCK = 0;
    return count; // a signed 32-bit integer
}

unsigned int getWeightFromSensor(void) {
    long loadCell1 = HX711_Read(); // Reading the value from the HX711 module 4 times
    long loadCell2 = HX711_Read();
    long loadCell3 = HX711_Read();
    long loadCell4 = HX711_Read();
    long totalWeight = loadCell1 + loadCell2 + loadCell3 + loadCell4;
    unsigned int weight = totalWeight / 4; // Average the weight readings
    return weight;
}


void displayWeightOnLCD(unsigned int weight) {
    // Convert weight to string for display
    char weightStr[10];
    sprintf(weightStr, "%d grams", weight);
    // Display weight on LCD
    lcd_command(0x01); // Clear display
    __delay_ms(5); // Delay to allow LCD to clear
    lcd_command(0x80); // Set cursor to first line
    lcd_string(weightStr, strlen(weightStr));
}


void main(void) {
    TRISCbits.TRISC6 = 0; // TX pin as output
    TRISCbits.TRISC7 = 1; // RX pin as input
    TRISCbits.TRISC0 = 1;
    TRISCbits.TRISC1 = 0;
    TRISB = 0x00;
    TRISD = 0x00;
    TRISDbits.TRISD3 = 0; // Setting the buzzer pin as output
    TRISCbits.TRISC2 = 0; 
      
    PORTDbits.RD3 = 0; 
    
    lcd_initialise();
    initADC();
    initPWM();
    initUART(); // Initialize UART for GSM communication

    
    while(1)
    {
        unsigned int sensorValue = readGasSensorPot();
        unsigned int weight = getWeightFromSensor();

        if(sensorValue > THRESHOLD)
        {
            PORTDbits.RD3 = 1; // To activate the buzzer
            
            setServoAngle(90);
            __delay_ms(40);
            
            lcd_command(0x01); // Clear display
            __delay_ms(5); // Delay to allow LCD to clear

            displayGasConcentration(sensorValue);
            __delay_ms(50);

            lcd_command(0x01); // Clear display
            __delay_ms(5); // Delay to allow LCD to clear
            displayWeightOnLCD(weight);
    
            __delay_ms(50);
            
//            testGSM(); 
            
            lcd_command(0x01); // Clear display
            __delay_ms(5); // Delay to allow LCD to clear
            lcd_command(0x80); // Force cursor to beginning of first line
            lcd_string("Gas Leak!!!!!!", 14);
            __delay_ms(5);
            
            sendSMS("0772900383", "Gas Leak Detected!");
//            makeMissedCall("0772900383");
            PORTDbits.RD3 = 0; // Deactivate the buzzer
        }
        
        else
        {
            PORTDbits.RD3 = 0; // To deactivate the buzzer
            lcd_command(0x01); // Clear display
            __delay_ms(5); // Delay to allow LCD to clear
            displayGasConcentration(sensorValue);
            __delay_ms(50);

            lcd_command(0x01); // Clear display
            __delay_ms(5); // Delay to allow LCD to clear
            displayWeightOnLCD(weight);
            __delay_ms(50);
            
            lcd_command(0x01); // Clear display
            __delay_ms(5); // Delay to allow LCD to clear
            lcd_command(0x80);
            lcd_string("No gas detected", 15);
            
        }
        
        __delay_ms(5);
        
    }
    
    return;

}