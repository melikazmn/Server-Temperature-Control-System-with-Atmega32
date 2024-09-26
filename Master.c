#include <avr/io.h>
#include <util/delay.h> // using for making delay
#include <avr/interrupt.h>
#include <LCD.h>

#define KEYPAD_PORT PORTB
#define KEYPAD_DDR DDRB
#define KEYPAD_PIN PINB
#define F_CPU 80000000UL
#define BAUD 9600
#define MYUBRR F_CPU / 16 / BAUD - 1

// LED status pins
#define LED_GREEN_PIN PC3
#define LED_YELLOW_PIN PC4
#define LED_RED_PIN PC5
#define SPEAKER_PIN PORTD5


char userPassword[20];
uint8_t menuMode = 0; // menu = 1 password = 0
volatile uint8_t isDataRecived = 0;
char recievedData[4];
char receivedChar;
int dataIndex = 0;
int attempts = 0;
int endOfPass = 0;
int unlocked = 0;
int index = 0;
char response;
volatile int endProgram = 0;

// Keypad character map
char keypad_chars[4][3] = {
    {'1', '2', '3'},
    {'4', '5', '6'},
    {'7', '8', '9'},
    {'*', '0', '#'}};

void init_keypad(void)
{
  KEYPAD_DDR = 0xF0;  // Set PB4-PB7 (rows) as output, PB0-PB3 (columns) as input
  KEYPAD_PORT = 0x0F; // Enable pull-ups for columns and set rows to high
}

char get_keypad_char(void)
{
  for (uint8_t row = 0; row < 4; row++)
  {
    KEYPAD_PORT = ~(1 << (row + 4)); // Ground one row at a time (PB4, PB5, PB6, PB7)
    _delay_us(5);                    // Wait for signal to settle

    for (uint8_t col = 0; col < 3; col++)
    {
      if (!(KEYPAD_PIN & (1 << col)))
      { // Check if any column is low (PB0, PB1, PB2)
        while (!(KEYPAD_PIN & (1 << col)))
          ;                            // Wait for key release
        return keypad_chars[row][col]; // Return corresponding character
      }
    }
  }
  return 0; // No key pressed
}

void USART_Init(unsigned int ubrr)
{
  /* Set baud rate */
  UBRRH = (unsigned char)(ubrr >> 8);
  UBRRL = (unsigned char)ubrr;
  /* Enable receiver and transmitter */
  UCSRB = (1 << RXEN) | (1 << TXEN) | (1 << RXCIE) | (1 << TXCIE);
  /* Set frame format: 8 data bits, 1 stop bit */
  UCSRC = (1 << URSEL) | (0 << UPM1) | (0 << UPM0) | (0 << USBS) | (1 << UCSZ1) | (1 << UCSZ0);
}

void USART_Transmit(unsigned char data)
{
  /* Wait for empty transmit buffer */
  while (((UCSRA >> UDRE) & 1) == 0)
    ;

  /* Put data into buffer, sends the data */
  UDR = data;
}

unsigned char USART_Receive(void)
{
  /* Wait for data to be received */
  while (((UCSRA >> RXC) & 1) == 0)
    ;
  /* Get and return received data from buffer */
  return UDR;
}

void menu()
{
  while (1)
  {
    LCD_cmd(0x01); // Clear display
    LCD_cmd(0x14);
    lcd_print("1.Motor  2.Temperature  #.Exit");
    char selectedOpt;
    while (selectedOpt != '1' && selectedOpt != '2' && selectedOpt != '#')
    {
      selectedOpt = get_keypad_char();
    }

    if (selectedOpt == '1')
    {
      LCD_cmd(0x01); // Clear display
      LCD_cmd(0x14);
      selectedOpt = '\0';
      lcd_print("1.M1 2.M2 3.M3 4.Exit");
      while (selectedOpt != '1' && selectedOpt != '2' && selectedOpt != '3' && selectedOpt != '#')
      {
        selectedOpt = get_keypad_char();
      }
      if (selectedOpt == '1')
      {
        USART_Transmit('o');
        while (isDataRecived == 0)
          ;
        if (isDataRecived)
        {
          LCD_cmd(0x01); // Clear display
          LCD_cmd(0x14);
          lcd_print("Duty Cycle of Motor 1:");
          lcd_print(recievedData);
          _delay_ms(400);
          isDataRecived = 0;
        }
      }
      else if (selectedOpt == '2')
      {
        USART_Transmit('m');
        while (isDataRecived == 0)
          ;
        if (isDataRecived)
        {
          LCD_cmd(0x01); // Clear display
          LCD_cmd(0x14);
          lcd_print("Duty Cycle of Motor 2:");
          lcd_print(recievedData);
          _delay_ms(400);
          isDataRecived = 0;
        }
      }
      else if (selectedOpt == '3')
      {
        USART_Transmit('n');
        while (isDataRecived == 0)
          ;
        if (isDataRecived)
        {
          LCD_cmd(0x01); // Clear display
          LCD_cmd(0x14);
          lcd_print("Duty Cycle of Motor 3:");
          lcd_print(recievedData);
          _delay_ms(400);
          isDataRecived = 0;
        }
      }
    }
    else if (selectedOpt == '2')
    {
      USART_Transmit('t');
      while (isDataRecived == 0)
        ;
      if (isDataRecived)
      {
        LCD_cmd(0x01); // Clear display
        LCD_cmd(0x14);
        lcd_print("Temperature is:");
        lcd_print(recievedData);
        _delay_ms(200);
        isDataRecived = 0;
      }
    }
    else if (selectedOpt == '#')
    {
      LCD_cmd(0x01); // Clear display
      LCD_cmd(0x14);
      lcd_print("Bye Bye!");
      _delay_ms(100);
      endProgram = 1;
      break;
    }
    selectedOpt = '\0';
  }
}

void receivingData(char character)
{
  if (character == '*')
  {
    recievedData[dataIndex] = '\0';
    dataIndex = 0;
    isDataRecived = 1;
  }
  else
  {
    recievedData[dataIndex++] = character;
  }
}

void play_tone(uint16_t frequency, uint16_t duration_ms)
{
  uint16_t delay = 1000000 / (2 * frequency); // Calculate delay for the desired frequency
  uint16_t cycles = (duration_ms * frequency) / 1000;

  for (uint16_t i = 0; i < cycles; i++)
  {
    PORTD ^= (1 << SPEAKER_PIN); // Toggle the speaker pin
    _delay_us(delay);            // Delay for half the period
  }
}



ISR(USART_RXC_vect)
{
  if (menuMode == 1) // gets information
  {
    receivedChar = UDR;
    if (receivedChar == 'R')
    {
      PORTD |= (1 << PORTD2);  // red
      PORTD &= ~(1 << PORTD4); // Turn off yellow LED
      play_tone(1000, 2000); // Play 1kHz tone for 2 seconds
    }
    else if (receivedChar == 'Y')
    {
      PORTD &= ~(1 << PORTD2); // Turn off red LED
      PORTD |= (1 << PORTD4);  // yellow
      PORTD &= ~(1 << PORTD3); // Turn off green LED
      PORTD &= ~(1 << PORTD5); // speaker off
    }
    else if (receivedChar == 'G')
    {
      PORTD &= ~(1 << PORTD2); // Turn off red LED
      PORTD &= ~(1 << PORTD4); // Turn off yellow LED
      PORTD |= (1 << PORTD3);  // green
      PORTD &= ~(1 << PORTD5); // speaker off
    }
    else
    {
      receivingData(receivedChar);
    }
  }
}

ISR(USART_TXC_vect)
{
}

int main(void)
{

  DDRA = 0xFF;                                     // all A's are ouput for the LCD
  DDRC = 0x07;                                     // just need 3 of them D0, D1 , D2 be output  0000 0111
  DDRD |= (1 << DDD2) | (1 << DDD3) | (1 << DDD4); // for red , green , yellow LED
  DDRD |= (1 << DDD5); // Set PD5 as output for the speaker
  sei();

  // Initialize peripherals
  init_LCD();         // initializing lCD
  init_keypad();      // Initialize keypad
  USART_Init(MYUBRR); // Initialize UART with baud rate 9600 (UBRR value 103 for 16MHz clock)

  _delay_ms(5);
  LCD_cmd(0x0F); // make blinking cursor

  while (1)
  {
    if (endProgram)
      break;

    LCD_cmd(0x14);
    lcd_print("Enter PassWord:");
    _delay_ms(10);


    while (unlocked != 1 && !endProgram)
    {
      while (endOfPass != 1 && !endProgram)
      {
        char key;               
        key = get_keypad_char(); 
        if (key && key != '*')
        {
          LCD_write(key); 
          _delay_ms(10);  
          userPassword[index] = key;
          index++;
        }
        else if (key == '*')
        {
          userPassword[index] = '\0';
          endOfPass = 1;
          attempts++;
        }
      }

      // Send the password to the slave
      for (uint8_t i = 0; i < index; i++)
      {
        USART_Transmit(userPassword[i]); // Send each character of the password
      }
      USART_Transmit('*'); 

      response = USART_Receive(); // Wait for response from slave

      if (response == 'U')
      {                // 'U' for unlocked
        LCD_cmd(0x01); // Clear display
        lcd_print("Welcome!!");
        _delay_ms(700);
        menuMode = 1;
        unlocked = 1;
        attempts = 0;
        PORTD |= (1 << PORTD3); // green on
        menu();
        response = 'U';
      }
      else if (response == 'L')
      { // 'L' for locked
        if (attempts == 3)
        {
          LCD_cmd(0x01); // Clear display
          lcd_print("Too much attempts.wait for 1 second...");
          _delay_ms(1000);
        }

        LCD_cmd(0x01); // Clear display
        lcd_print("Wrong Password!");
        _delay_ms(400);
        LCD_cmd(0x01); // Clear display
        lcd_print("Enter Password again:");
        endOfPass = 0;
        index = 0; // Reset password index
      }
    } // end of unlock
  }
}
