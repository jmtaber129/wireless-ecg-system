#include <msp430g2553.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


#include "configuration.h"
#include "uart-queue.h"


const char* kEot = "\004\n";

enum UartCommand {
  kStart = 'a',
  kStop = 'b',
  kSync = 's'
};


volatile UartCommand current_command = kStop;
int adc_sample = 0;
volatile int timer_count = 0;
volatile int timestamp = 0;
char rx_buffer[15];
int rx_index = 0;

// True if the next timer interrupt should put the sync output high.  False if
// the next timer interrupt should put the sync output high.
bool sync = false;

bool received_sync = false;

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

  if (rx_buffer[0] == kStop) {
    current_command = kStop;

    // Add end of transmission (EOT) character to the queue.
    uart_queue.Push(kEot);

    // Wake up from LPM so the EOT char can be transmitted.
    LPM0_EXIT;
  } else if (rx_buffer[0] == kSync) {
    sync = true;
  } else {
    current_command = kStart;

    if (rx_buffer[1] != '\000') {
      char* tmp_command = strtok(rx_buffer, ",");
      char* str_timestamp = strtok(NULL, ",");
      char* str_timer_count = strtok(NULL, ",");

      timestamp = atoi(str_timestamp);
      timer_count = atoi(str_timer_count);
    }
  }
}

#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void) {
  if ((SYNC_INPUT & P1IFG) && (SYNC_INPUT & P1IN)) {
    timestamp = 0;
    timer_count = 0;
  }
  P1IFG = 0;
}

#pragma vector=TIMER1_A0_VECTOR
__interrupt void Timer_A (void) {
  if (SYNC_INPUT & P1IN) {
    if (!received_sync) {
      timestamp = 1;
      timer_count = 0;
      received_sync = true;
    }
  } else {
    received_sync = false;
  }

  if (++timer_count < 10) {
    return;
  }

  // Handle sync pulse.
  if (sync) {
    P1OUT |= SYNC_OUTPUT;
    timestamp = 0;
    timer_count = 0;
  } else {
    P1OUT &= ~SYNC_OUTPUT;
  }

  sync = false;

  timer_count = 1;
  ++timestamp;

  if (timestamp > 32000) {
    timestamp = 0;
  }

  if (current_command == kStop) {
    // If the current command is STOP, don't take any measurements, and just
    // return.
    return;
  }

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
