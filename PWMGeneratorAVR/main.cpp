#define F_CPU 16000000L//System clock frequency is 16MHz
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
//#define T_LOW 1162//Calculate timer value: Clk_sys/(scale*Compare value) = ISR_freq. T_LOW = time_des x ISR_freq where time_des = 18500us
//#define T_HIGH 94//Calculate timer value: Clk_sys/(scale*Compare value) = ISR_freq. T_HIGH = time_des x ISR_freq where time_des = 1500us

char value;
volatile unsigned int t_high = 0, t_low = 0, T_HIGH = 100, T_LOW = 1000;

void USART_Transmit(unsigned char data)
{
    /* Wait for empty transmit buffer */
    if (!(UCSR0A & (1 << UDRE0)))
        ;
    UDR0 = data;
    /* Put data into buffer, sends the data */

}

int main(void)
{
    DDRD |= (1 << DDD2);// configuring pin 2 as output
    DDRB |= (1 << DDB5);// configuring pin 13 as output
    UBRR0 = 103; // for configuring baud rate of 9600bps
    UCSR0C = 0x05;//8bit char length, no parity, one stop bit

    UCSR0B |= (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);// Turn on the transmission, reception, and Receive interrupt
    SREG |= (1 << 7);// enable global interrupt

    TCCR0A = (1 << WGM01);//Waveform Generation Mode:Clear Timer on Compare Match (CTC)
    OCR0A = 255;//Compare register
    TIMSK0 = (1 << OCIE0A);//Compare match interrupt enabled
    sei();//Enable global interrupt
    //TCCR0B = (1 << CS02) | (1 << CS00);//Prescaler setting:clk_i/o/1024 --> 15625 divided by 155 (compare register) --> 100Hz
    TCCR0B = 0x01;
    //Calculate timer value: Clk_sys/(scale*Compare value) = ISR_freq. timer = time_des x ISR_freq

    while (1)
    {
        PORTB &= ~(1 << PORTB5);
        PORTD &= ~(1 << PORTD2);
        t_low = 0;
        while (t_low != T_LOW);
        PORTB |= (1 << PORTB5);
        PORTD |= (1 << PORTD2);
        t_high = 0;
        while (t_high != T_HIGH);

        if (value == 119) {
            T_LOW = 1162;
            T_HIGH = 94;
        }
        if (value == 97) {
            T_LOW = 1193;
            T_HIGH = 63;
        }
        if (value == 100) {
            T_LOW = 1130;
            T_HIGH = 126;
        }
    }
}
//interrupt routine for serial communication
ISR(USART_RX_vect)
{
    value = UDR0; // read the received data byte in temp
}

ISR(TIMER0_COMPA_vect)
{
    t_low++;
    t_high++;
}
