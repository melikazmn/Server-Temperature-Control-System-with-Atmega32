#include <avr/io.h>
#include <string.h>
#include <util/delay.h> // using for making delay
#include <LCD.h>
#include <avr/interrupt.h>
#include <stdio.h>

#define F_CPU 80000000UL
#define BAUD 9600
#define MYUBRR F_CPU / 16 / BAUD - 1

volatile char received_char;
volatile uint8_t data_ready = 0;
uint16_t adc_value;
float DutyCycle = 0.0;
uint8_t motor1_status = 1;
uint8_t motor2_status = 0;
uint8_t motor3_status = 0;
uint8_t motor1_duty;
uint8_t motor2_duty;
uint8_t motor3_duty;
char dataToSend[4];
int logedIn = 0;
float temperature;
char PrevState = 'G';
char NewState = 'G';

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

unsigned char USART_Receive(void)
{
  /* Wait for data to be received */
  while (((UCSRA >> RXC) & 1) == 0)
    ;
  return UDR;
}

void USART_Transmit(unsigned char data)
{
  /* Wait for empty transmit buffer */
  while (((UCSRA >> UDRE) & 1) == 0)
    ;
  /* Put data into buffer, sends the data */
  UDR = data;
}

void init_ADC()
{
  ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);//system clock by 64
}

uint16_t read_ADC(uint8_t channel)
{
  //selects which ADC channel (0-7) to read from
  ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC))
    ;
  return ADCW;
}

void init_PWM()
{
  // WGM01 and WGM00 Fast PWM mode COM01 Set the non-inverting mode for OC0 CS01 Set the prescaler to 8, starting the timer.
  TCCR0 |= (1 << WGM01) | (1 << WGM00) | (1 << COM01) | (1 << CS01);
  TCCR1A |= (1 << WGM10) | (1 << COM1A1) | (1 << COM1B1);
  TCCR1B |= (1 << WGM12) | (1 << CS11);
  TCCR2 |= (1 << WGM21) | (1 << WGM20) | (1 << COM21) | (1 << CS21);
  DDRB |= (1 << DDB3);
  DDRD |= (1 << DDD7) | (1 << DDD5) | (1 << DDD4);
}

void duty_percent(int ADC_val)
{
  if (ADC_val < 102)
    DutyCycle = 0;
  else if (ADC_val < 204)
    DutyCycle = 10;
  else if (ADC_val < 306)
    DutyCycle = 20;
  else if (ADC_val < 409)
    DutyCycle = 30;
  else if (ADC_val < 511)
    DutyCycle = 40;
  else if (ADC_val < 613)
    DutyCycle = 50;
  else if (ADC_val < 716)
    DutyCycle = 60;
  else if (ADC_val < 818)
    DutyCycle = 70;
  else if (ADC_val < 920)
    DutyCycle = 80;
  else if (ADC_val < 1023)
    DutyCycle = 90;
  else
    DutyCycle = 100;
}

void update_motor_status()
{

  if (motor1_status)
    motor1_duty = DutyCycle;
  if (motor2_status)
    motor2_duty = DutyCycle;
  if (motor3_status)
    motor3_duty = DutyCycle;

  if (!motor1_status && !motor2_status && !motor3_status)
  {
    NewState = 'R';
  }
  else if (!motor1_status && !motor2_status)
  {
    if (DutyCycle * 3 > 100)
    {
      motor3_duty = 100;
      NewState = 'R';
    }
    else
    {
      motor3_duty = DutyCycle * 3;
      NewState = 'Y'; // two motors not working
    }
  }
  else if (!motor1_status && !motor3_status)
  {
    if (DutyCycle * 3 > 100)
    {
      motor2_duty = 100;
      NewState = 'R';
    }
    else
    {
      motor2_duty = DutyCycle * 3;
      NewState = 'Y'; // two motors not working
    }
  }
  else if (!motor2_status && !motor3_status)
  {
    if (DutyCycle * 3 > 100)
    {
      motor1_duty = 100;
      NewState = 'R';
    }
    else
    {
      motor1_duty = DutyCycle * 3;
      NewState = 'Y'; // two motors not working
    }
  }
  else if (!motor1_status)
  {
    if ((DutyCycle * 3) / 2 > 100)
    {
      motor2_duty = 100;
      motor3_duty = 100;
      NewState = 'R';
    }
    else
    {
      motor2_duty = DutyCycle * 1.5;
      motor3_duty = DutyCycle * 1.5;
      NewState = 'Y'; // two motors not working
    }
  }
  else if (!motor2_status)
  {
    if ((DutyCycle * 3) / 2 > 100)
    {
      motor1_duty = 100;
      motor3_duty = 100;
      NewState = 'R';
    }
    else
    {
      motor1_duty = DutyCycle * 1.5;
      motor3_duty = DutyCycle * 1.5;
      NewState = 'Y'; // two motors not working
    }
  }
  else if (!motor3_status)
  {
    if ((DutyCycle * 3) / 2 > 100)
    {
      motor2_duty = 100;
      motor1_duty = 100;
      NewState = 'R';
    }
    else
    {
      motor2_duty = DutyCycle * 1.5;
      motor1_duty = DutyCycle * 1.5;
      NewState = 'Y'; // two motors not working
    }
  }

  if ((NewState == 'Y' && PrevState == 'G') || (NewState == 'Y' && PrevState == 'R'))
  {
    USART_Transmit('Y');
    PrevState = NewState;
  }
  else if (NewState == 'R' && PrevState == 'Y')
  {
    USART_Transmit('R');
    PrevState = NewState;
  }
  else if (NewState == 'G' && PrevState == 'Y')
  {
    USART_Transmit('G');
    PrevState = NewState;
  }

  OCR0 = (motor1_duty * 255) / 100;
  OCR1A = (motor2_duty * 255) / 100;
  OCR2 = (motor3_duty * 255) / 100;
}

void convert_to_temperature()
{
  temperature = (adc_value >> 2) / 255.0; // Convert ADC value to voltage (assuming Vref = 1V)
  temperature = temperature * 100;        // Convert voltage to temperature in degrees Celsius
}

ISR(USART_RXC_vect)
{
  received_char = UDR;
  if (logedIn)
  {
    if (received_char == 'o' || received_char == 'm' || received_char == 'n' || received_char == 't')
    {
      switch (received_char)
      {
      case 'o':
        snprintf(dataToSend, sizeof(dataToSend), "%d", motor1_duty);
        break;
      case 'm':
        snprintf(dataToSend, sizeof(dataToSend), "%d", motor2_duty);
        break;
      case 'n':
        snprintf(dataToSend, sizeof(dataToSend), "%d", motor3_duty);
        break;
      case 't':
        adc_value = read_ADC(7);
        convert_to_temperature();
        snprintf(dataToSend, sizeof(dataToSend), "%d", (int)temperature);
        break;
      default:
        break;
      }
      for (uint8_t i = 0; i < strlen(dataToSend); i++)
      {
        USART_Transmit(dataToSend[i]);
      }
      USART_Transmit('*');
    }
  }
  else
  {
    data_ready = 1;
  }
}

ISR(USART_TXC_vect)
{
}

int main(void)
{
  uint8_t index = 0;
  char password[20] = "1234";
  char received_password[20]; // Define the correct password
  DDRA |= (0 << DDA7);        // input for adc
  ADMUX |= (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (0 << MUX4) | 
  (0 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0);
  sei();

  init_ADC();
  init_PWM();
  USART_Init(MYUBRR); // Initialize UART

  while (1)
  {
    // Check if data is available
    if (data_ready && !logedIn)
    {
      data_ready = 0;
      if (received_char == '*')
      {
        received_password[index] = '\0';
        index = 0;
        // Validate password and send response via USART
        if (strcmp(password, received_password) == 0)
        {
          logedIn = 1;
          USART_Transmit('U'); // Send 'U' for unlocked
          DDRA |= (1 << DDA0);
          _delay_ms(3000);
          while (1)
          {
            adc_value = read_ADC(7);
            duty_percent(adc_value);
            update_motor_status();
          }
        }
        else
        {
          USART_Transmit('L'); // Send 'L' for locked
        }
      }
      else
      {
        received_password[index++] = received_char;
      }
    }
  }
}
