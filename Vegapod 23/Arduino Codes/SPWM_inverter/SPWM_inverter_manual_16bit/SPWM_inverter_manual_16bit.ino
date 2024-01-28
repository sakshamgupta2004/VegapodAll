#include <avr/io.h>
#include <avr/interrupt.h>
#define fs 10
int* lookUp;
int lookUp2[] = { 800.00, 850.23, 900.27, 949.91, 998.95, 1047.21, 1094.50, 1140.62, 1185.40, 1228.66, 1270.23, 1309.94, 1347.64, 1383.17, 1416.41, 1447.21, 1475.46, 1501.05, 1523.86, 1543.82, 1560.85, 1574.87, 1585.83, 1593.69, 1598.42, 1600.00, 1598.42, 1593.69, 1585.83, 1574.87, 1560.85, 1543.82, 1523.86, 1501.05, 1475.46, 1447.21, 1416.41, 1383.17, 1347.64, 1309.94, 1270.23, 1228.66, 1185.40, 1140.62, 1094.50, 1047.21, 998.95, 949.91, 900.27, 850.23, 800.00, 749.77, 699.73, 650.10, 601.05, 552.79, 505.50, 459.38, 414.60, 371.34, 329.77, 290.06, 252.36, 216.83, 183.59, 152.79, 124.54, 98.95, 76.14, 56.18, 39.15, 25.13, 14.17, 6.31, 1.58, 0.00, 1.58, 6.31, 14.17, 25.13, 39.15, 56.18, 76.14, 98.95, 124.54, 152.79, 183.59, 216.83, 252.36, 290.06, 329.77, 371.34, 414.60, 459.38, 505.50, 552.79, 601.05, 650.10, 699.73, 749.77, 800.00 };
int * runsPerPhase;
int samples;
int maxCount;
int maxCountQuo;
volatile int num, num2, num3;
volatile bool t2setup, t2_setup, t3setup, t3_setup;
volatile int numRunsT1, numRunsT2, numRunsT3;
void changeFreq(float fs1) {
  PORTB = 0;
  PORTD = 0;
  cli();

  num = 0;
  num2 = 0;
  num3 = 0;
  numRunsT1 = 0;
  numRunsT2 = 0;
  numRunsT3 = 0;
  t2setup = false;
  t2_setup = false;
  t3setup = false;
  t3_setup = false;


  maxCount = (160000.0 / fs1) / 64.0;  //change this multiplier
  maxCountQuo = maxCount / 255;
  for (int i = 0; i <= samples; i++) {
    lookUp[i] = (lookUp2[i] / 1600.0) * maxCount;
    runsPerPhase[i] = lookUp[i] / 255;
  }

  TCCR2A = 0;
  TCCR2A |= (1 << 0) | (1 << 1);
  TCCR2B = 0;
  TCCR2B |= (1 << 3) | (1 << 2);
  OCR2A = 255;  // Set max countedr value
  TIMSK2 |= (1 << OCIE2B) | (1 << TOIE2);

  TCCR1A = 0b00000011;
  TCCR1B = 0b00011011;
  OCR1A = 255;  // Set max countedr value
  TIMSK1 |= (1 << OCIE1B) | (1 << TOIE1);

  TCCR0A = 0;
  TCCR0A |= (1 << WGM01) | (1 << WGM00);  //wgm sets timer mode. in this case fast pwm overflow at ocrna
  TCCR0B = 0;
  TCCR0B |= (1 << WGM02) | (1 << CS01) | (1 << CS00);    //CS bits for frequency scaling
  OCR0A = 255;                             // Set max countedr value
  TIMSK0 |= (1 << OCIE0B) | (1 << TOIE0);  //for calling comparison and overflow function/interrupts

  sei();

  PORTB = 0;
  PORTB |= (1 << 2);
}
int main() {
  DDRB |= (1 << 2);
  DDRD |= (1 << 6) | (1 << 5) | (1 << 4) | (1 << 3) | (1 << 2) | (1 << 1);
  PORTB = 0;
  PORTD = 0;

  samples = sizeof(lookUp2) / sizeof(lookUp2[0]);
  lookUp = (int*)malloc(sizeof(lookUp2));
  runsPerPhase = (int*)malloc(sizeof(lookUp2));
  changeFreq(fs);

  while (1) {
    for (float i = fs; i < 100; i += 5) {
      changeFreq(i);
      _delay_ms(500);
    }
    for (float i = 100; i > fs; i -= 5) {
      changeFreq(i);
      _delay_ms(500);
    }
  }
}


ISR(TIMER1_OVF_vect) {
  numRunsT1++;
  if (numRunsT1 == maxCountQuo + 1) {
    if (lookUp[num] > 0) {
      PORTD &= ~(1 << 5);
    }
    OCR1B = (lookUp[num] % 255);
    OCR1A = 255;
    if (num < samples) {
      num += 1;
    } else {
      num = 0;
    }
    if (!t2setup) {
      num2 = num + (samples / 3);
      num2 %= samples;
      t2_setup = true;
    }
    if (!t3setup) {
      num3 = num + ((2 * samples) / 3);
      num3 %= samples;
      t3_setup = true;
    }
    numRunsT1 = 0;
    if (lookUp[num] > 0) {
      PORTD |= (1 << 6);
    }
  }
  else if (numRunsT1 == maxCountQuo) {
    OCR1A = lookUp[num] % 255;
  }
}
ISR(TIMER1_COMPB_vect) {
  if (numRunsT1 == runsPerPhase[num]) {
    PORTD &= ~(1 << 6);
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    PORTD |= (1 << 5);
  }
}
ISR(TIMER2_OVF_vect) {
  numRunsT2++;
  if (numRunsT2 == maxCountQuo + 1) {
    if (lookUp[num2] > 0) {
      PORTD &= ~(1 << 3);
    }
    OCR2B = (lookUp[num2] % 255);
    OCR2A = 255;
    if (num2 < samples) {
      num2 += 1;
    } else {
      num2 = 0;
    }
    if (t2_setup) {
      t2setup = true;
    }
    numRunsT2 = 0;
    if (lookUp[num2] > 0) {
      PORTD |= (1 << 4);
    }
  }
  else if (numRunsT2 == maxCountQuo) {
    OCR2A = lookUp[num2] % 255;
  }
}
ISR(TIMER2_COMPB_vect) {
  if (numRunsT2 == runsPerPhase[num2]) {
    PORTD &= ~(1 << 4);
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    PORTD |= (1 << 3);
  }
}
ISR(TIMER0_OVF_vect) {
  numRunsT3++;
  if (numRunsT3 == maxCountQuo + 1) {
    if (lookUp[num3] > 0) {
      PORTD &= ~(1 << 1);
    }
    OCR0B = (lookUp[num3] % 255);
    OCR0A = 255;
    if (num3 < samples) {
      num3 += 1;
    } else {
      num3 = 0;
    }
    if (t3_setup) {
      t3setup = true;
    }
    numRunsT3 = 0;
    if (lookUp[num3] > 0) {
      PORTD |= (1 << 2);
    }
  }
  else if (numRunsT3 == maxCountQuo) {
    OCR0A = lookUp[num3] % 255;
  }
}
ISR(TIMER0_COMPB_vect) {
  if (numRunsT3 == runsPerPhase[num3]) {
    PORTD &= ~(1 << 2);
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    PORTD |= (1 << 1);
  }
}
