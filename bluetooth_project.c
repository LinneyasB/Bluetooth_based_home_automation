#include <lpc21xx.h>

#define LCD_D (0xFF << 2)
#define RS (1 << 10) 
#define E (1 << 11)
#define LED (1 << 27)
#define FAN (1 << 28)
#define TV (1 << 29)
#define MOTORP1 (1 << 23)
#define MOTORP2 (1 << 24)
// project_bluetooth

// Function prototypes
void DELAY_MILLISECONDS(unsigned int ms);
void LCD_INIT(void);
void LCD_COMMAND(unsigned char cmd);
void LCD_DATA(unsigned char d);
void UART_CONFIG(void);
void LIGHT(void);
void FAN_CONTROL(void);
void MOTOR_CONTROL(void);
void TV_CONTROL(void);
void TURN_OFF(void);
void UART0_TX(unsigned char);
unsigned char UART0_RX(void) ;
void LCD_STR(unsigned char *);
void USR_INPUT(void) __irq;
void CURRENT_STS(int ,int ,int ,int );



// Global variables
int L_FLAG = 0;
int F_FLAG = 0;
int T_FLAG = 0;
int M_FLAG = 0;



int main() 
{	
    // Configure UART
    UART_CONFIG();
    
    // Initialize LCD
    LCD_INIT();
    LCD_COMMAND(0X80);
    LCD_STR("HOME AUTOMATION");

    // Configure interrupt for UART
    VICIntSelect = 0; // All interrupts are IRQs
    VICVectCntl0 = (1 << 5) | 6; // Set priority and enable interrupt
    VICVectAddr0 = (int)USR_INPUT; // Set the interrupt handler
    VICIntEnable = (1 << 6); // Enable the specific interrupt for UART0
    U0IER = (1 << 0); // Enable UART0 interrupt

   while (1) 
{
    CURRENT_STS(L_FLAG, F_FLAG, T_FLAG, M_FLAG);
    // Optionally add a small delay if necessary to prevent spamming the LCD
    DELAY_MILLISECONDS(500); // Adjust as needed
}


     
}

void USR_INPUT(void) __irq 
{
    unsigned char ch;
    ch = UART0_RX();
	
    UART0_TX(ch); // Echo received data
    // Display on LCD

    // Check the command and control devices accordingly
    switch(ch) {
        case 'L': LIGHT(); break;
        case 'F': FAN_CONTROL(); break;
        case 'T': TV_CONTROL(); break;
        case 'M': MOTOR_CONTROL(); break;
        case '0': TURN_OFF(); break;
    }
    
    VICVectAddr = 0; // Acknowledge the interrupt
}


void DELAY_MILLISECONDS(unsigned int ms) 
{
    T0PR = 15000 - 1; // Prescale for 1 ms
    T0TCR = 0X01; // Start timer
    while (T0TC < ms); // Wait
    T0TCR = 0X03; // Stop timer
    T0TCR = 0X00; // Reset timer
}

void LCD_INIT(void) 
{
    IODIR0 |= LCD_D | RS | E;
    DELAY_MILLISECONDS(20); // Wait for LCD to power up
    LCD_COMMAND(0X01); // Clear display
    DELAY_MILLISECONDS(2);
    LCD_COMMAND(0X02); // Initialize in 4-bit mode
    LCD_COMMAND(0X0C); // Display ON, Cursor OFF
    LCD_COMMAND(0X38); // 8-bit mode
    LCD_COMMAND(0X80); // Set cursor position
}

void LCD_COMMAND(unsigned char cmd) 
{
    IOCLR0 = LCD_D;
    IOSET0 = cmd << 2;
    IOCLR0 = RS; // Command mode
    IOSET0 = E; // Enable
    DELAY_MILLISECONDS(2);
    IOCLR0 = E; // Disable
}

void LCD_DATA(unsigned char d) 
{
    IOCLR0 = LCD_D;
    IOSET0 = d << 2;
    IOSET0 = RS; // Data mode
    IOSET0 = E; // Enable
    DELAY_MILLISECONDS(2);
    IOCLR0 = E; // Disable
}

void LCD_STR(unsigned char* s) 
{
    while (*s) {
        LCD_DATA(*s++);
    }
}

void UART_CONFIG(void) 
{    
    PINSEL0 |= 0x05; // Configure UART0
    U0LCR = 0X83; // 8 bits, no parity, 1 stop bit
    U0DLL = 97; // Baud rate 9600
    U0DLM = 0;
    U0LCR = 0X03; // 8 bits, no parity, 1 stop bit
}

unsigned char UART0_RX(void) 
{
    while ((U0LSR & 1) == 0); // Wait until data is received
    return U0RBR; // Return received character
}

void UART0_TX(unsigned char ch) 
{
    U0THR = ch; // Transmit character
    while ((U0LSR & (1 << 5)) == 0); // Wait for transmission to complete
}




void TURN_OFF(void) 
{

    IOSET1 = LED | FAN | TV | MOTORP1 | MOTORP2;
    L_FLAG = F_FLAG = T_FLAG = M_FLAG = 0;
}

void LIGHT(void) 
{
    IODIR1 |= LED;
    if (L_FLAG == 0) {
        IOCLR1 = LED; // Turn ON LED
        L_FLAG = 1;
    } else {
        IOSET1 = LED; // Turn OFF LED
        L_FLAG = 0;
    }    
}

void FAN_CONTROL(void) 
{
    IODIR1 |= FAN;
    if (F_FLAG == 0) {
        IOCLR1 = FAN; // Turn ON Fan
        F_FLAG = 1;
    } else {
        IOSET1 = FAN; // Turn OFF Fan
        F_FLAG = 0;
    }    
}

void TV_CONTROL(void) 
{
    IODIR1 |= TV;
    if (T_FLAG == 0) {
        IOCLR1 = TV; // Turn ON TV
        T_FLAG = 1;
    } else {
        IOSET1 = TV; // Turn OFF TV
        T_FLAG = 0;
    }    
}

void MOTOR_CONTROL(void) 
{
    IODIR1 |= MOTORP1 | MOTORP2;
    if (M_FLAG == 0) {    
        IOSET1 = MOTORP1; // Start motor
        IOCLR1 = MOTORP2; 
        M_FLAG = 1; // Update the flag
    } else {
        IOCLR1 = MOTORP1; // Stop motor
        IOCLR1 = MOTORP2;
        M_FLAG = 0; // Update the flag
    }    
}

void CURRENT_STS(int a, int b, int c, int d)
{
    unsigned char status[20]; 
    LCD_COMMAND(0xC0); 

    // Prepare the status string
    // Adjust size as needed

    // Build the status string
    status[0] = 'L'; // Light
    status[1] = '-';
    status[2] = (a == 1) ? 'O' : 'F'; // ON/OFF for light
    status[3] = ' ';
    
    status[4] = 'F'; // Fan
    status[5] = '-';
    status[6] = (b == 1) ? 'O' : 'F'; // ON/OFF for fan
    status[7] = ' ';

    status[8] = 'T'; // TV
    status[9] = '-';
    status[10] = (c == 1) ? 'O' : 'F'; // ON/OFF for TV
    status[11] = ' ';

    status[12] = 'M'; // Motor
    status[13] = '-';
    status[14] = (d == 1) ? 'O' : 'F'; // ON/OFF for motor
    status[15] = '\0'; // Null-terminate the string

    // Display the status on LCD
    LCD_STR(status);

    
    
}
