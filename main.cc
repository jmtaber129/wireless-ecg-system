#include <msp430g2553.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


#include "configuration.h"
#include "uart-queue.h"


const char* kEot = "\004\n";

enum UartCommand {
  kStart = 'a',
  kStop = 'b'
};


volatile UartCommand current_command = kStop;
int adc_sample = 0;
int timer_count = 0;
int timestamp = 0;
char rx_buffer[15];
int rx_index = 0;

UartQueue uart_queue;

int main(void) {
  Configuration::Init(&adc_sample);

  // Serial data-streaming loop.
  while (1) {
    if (!uart_queue.Empty()) {
      // Get the next char to send.
      char current_char = uart_queue.Front();

      // Wait for the transmit buffer to be ready.
      while (!(IFG2 & UCA0TXIFG));

      // Load the char into the buffer.
      UCA0TXBUF = current_char;

      // Remove the string we just sent from the queue.
      uart_queue.Pop();
    } else {
      // If there's nothing left to process, enter low-power mode.
      __bis_SR_register(LPM0_bits);
    }

  }
}


#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void) {
  if (UCA0RXBUF != '\n' && UCA0RXBUF != '\r') {
    rx_buffer[rx_index] = UCA0RXBUF;
    ++rx_index;
    return;
  }

  rx_buffer[rx_index] = '\000';
  rx_index = 0;


  if (rx_buffer[0] == kStart) {
    current_command = kStart;
    char* tmp_command = strtok(rx_buffer, ",");
    char* str_timestamp = strtok(NULL, ",");
    char* str_timer_count = strtok(NULL, ",");

    timestamp = atoi(str_timestamp);
    timer_count = atoi(str_timer_count);
  } else if (rx_buffer[0] == kStop) {
    current_command = kStop;

    // Add end of transmission (EOT) character to the queue.
    uart_queue.Push(kEot);

    // Wake up from LPM so the EOT char can be transmitted.
    LPM0_EXIT;
  } else {
    // Received character isn't a valid command.
    // TODO(jmtaber129): Add invalid command error handling.
  }
}

#pragma vector=TIMER1_A0_VECTOR
__interrupt void Timer_A (void) {
  if (++timer_count <= 3) {
    return;
  }

  timer_count = 0;

  if (current_command == kStop) {
    // If the current command is STOP, don't take any measurements, and just
    // return.
    return;
  }

  ++timestamp;

  // Toggle P1.0 for debugging purposes.
  P1OUT ^= BIT0;

  char buffer[10];
  ADC10CTL0 |= ENC + ADC10SC;      //enable conversion and start conversion
  while(ADC10CTL1 & BUSY);
  adc_sample = ADC10MEM;
  sprintf(buffer, "%u,%d\n", timestamp, adc_sample);
  uart_queue.Push(buffer);

  // The string that was just pushed to the queue needs to be sent by the main
  // loop, so exit LPM.
  LPM0_EXIT;

  return;
}
