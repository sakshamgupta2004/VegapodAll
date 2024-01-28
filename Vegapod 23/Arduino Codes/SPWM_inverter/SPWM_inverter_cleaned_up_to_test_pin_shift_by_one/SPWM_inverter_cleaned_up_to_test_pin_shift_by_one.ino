#include <avr/io.h>
#include <avr/interrupt.h>
#define fs 100
int* lookUp;
int lookUp2[] = { 800.00, 850.23, 900.27, 949.91, 998.95, 1047.21, 1094.50, 1140.62, 1185.40, 1228.66, 1270.23, 1309.94, 1347.64, 1383.17, 1416.41, 1447.21, 1475.46, 1501.05, 1523.86, 1543.82, 1560.85, 1574.87, 1585.83, 1593.69, 1598.42, 1600.00, 1598.42, 1593.69, 1585.83, 1574.87, 1560.85, 1543.82, 1523.86, 1501.05, 1475.46, 1447.21, 1416.41, 1383.17, 1347.64, 1309.94, 1270.23, 1228.66, 1185.40, 1140.62, 1094.50, 1047.21, 998.95, 949.91, 900.27, 850.23, 800.00, 749.77, 699.73, 650.10, 601.05, 552.79, 505.50, 459.38, 414.60, 371.34, 329.77, 290.06, 252.36, 216.83, 183.59, 152.79, 124.54, 98.95, 76.14, 56.18, 39.15, 25.13, 14.17, 6.31, 1.58, 0.00, 1.58, 6.31, 14.17, 25.13, 39.15, 56.18, 76.14, 98.95, 124.54, 152.79, 183.59, 216.83, 252.36, 290.06, 329.77, 371.34, 414.60, 459.38, 505.50, 552.79, 601.05, 650.10, 699.73, 749.77, 800.00 };
int samples;
float TP;
int num, num2, num3;
bool t2setup, t2_setup, t3setup, t3_setup;
void changeFreq(float fs1) {
  PORTB = 0;
  PORTD = 0;
  cli();

  num = 0;
  num2 = 0;
  num3 = 0;
  t2setup = false;
  t2_setup = false;
  t3setup = false;
  t3_setup = false;

  if (fs1 >= 80) {
    TP = (1 / fs1) * 9.661835 * 8;
  } else if (fs1 >= 20) {
    TP = (1 / fs1) * 9.661835;
  } else {
    TP = (1 / fs1) * 9.661835 * 0.25;
  }
  for (int i = 0; i <= samples; i++) {
    lookUp[i] = (lookUp2[i] / 1600.0) * 255.0 * TP;
  }

  TCCR2A = 0;
  TCCR2A |= (1 << 0) | (1 << 1);
  TCCR2B = 0;
  TCCR2B |= (1 << 3);
  if (fs1 >= 80) {
    TCCR2B |= (1 << 1);
  } else if (fs1 >= 20) {
    TCCR2B |= (1 << 2);
  } else {
    TCCR2B |= (1 << 2) | (1 << 1);
  }
  OCR2A = 255 * TP;  // Set max countedr value
  TIMSK2 |= (1 << OCIE2B) | (1 << TOIE2);

  TCCR1A = 0b00000011;
  TCCR1B = 0b00011000;
  if (fs1 >= 80) {
    TCCR1B |= (1 << 1);
  } else if (fs1 >= 20) {
    TCCR1B |= (1 << 1) | (1 << 0);
  } else {
    TCCR1B |= (1 << 2);
  }
  OCR1A = 255 * TP;  // Set max countedr value
  TIMSK1 |= (1 << OCIE1B) | (1 << TOIE1);

  TCCR0A = 0;
  TCCR0A |= (1 << WGM01) | (1 << WGM00);  //wgm sets timer mode. in this case fast pwm overflow at ocrna
  TCCR0B = 0;
  TCCR0B |= (1 << WGM02);
  if (fs1 >= 80) {
    TCCR0B |= (1 << 1);
  } else if (fs1 >= 20) {
    TCCR0B |= (1 << 1) | (1 << 0);
  } else {
    TCCR0B |= (1 << 2);
  }
  OCR0A = 255 * TP;                        // Set max countedr value
  TIMSK0 |= (1 << OCIE0B) | (1 << TOIE0);  //for calling comparison and overflow function/interrupts

  sei();

  PORTB = 0;
  PORTB |= (1 << 2);
}

bool stringComplete = false;
String inputString = "";

int main() {
  DDRB |= (1 << 2);
  DDRD |= (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | (1 << 3) | (1 << 2);
  PORTB = 0;
  PORTD = 0;

  samples = sizeof(lookUp2) / sizeof(lookUp2[0]);
  lookUp = malloc(sizeof(lookUp2));
  changeFreq(fs);
  // Serial.begin(115200);

  while (1) {
    /*for (float i = fs; i < 150; i += 5) {
      changeFreq(i);
      _delay_ms(200);
    }
    for (float i = 150; i > fs; i -= 5) {
      changeFreq(i);
      _delay_ms(200);
    }*/
    /* if (Serial.available() > 0) {
      while (Serial.available()) {
        // get the new byte:
        char inChar = (char)Serial.read();
        // add it to the inputString:
        inputString += inChar;
        // if the incoming character is a newline, set a flag so the main loop can
        // do something about it:
        if (inChar == '\n') {
          stringComplete = true;
        }
      }
    }
    if (stringComplete) {
      stringComplete = true;
      
    }*/
  }
}


ISR(TIMER1_OVF_vect) {
  OCR1B = (lookUp[num]);
  if (lookUp[num] > 0) {
    PORTD &= ~(1 << 6);
  }
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
  if (lookUp[num] > 0) {
    PORTD |= (1 << 7);
  }
}
ISR(TIMER1_COMPB_vect) {
  PORTD &= ~(1 << 7);
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
  PORTD |= (1 << 6);
}
ISR(TIMER2_OVF_vect) {
  OCR2B = (lookUp[num2]);
  if (lookUp[num2] > 0) {
    PORTD &= ~(1 << 4);
  }
  if (num2 < samples) {
    num2 += 1;
  } else {
    num2 = 0;
  }
  if (t2_setup) {
    t2setup = true;
  }
  if (lookUp[num2] > 0) {
    PORTD |= (1 << 5);
  }
}
ISR(TIMER2_COMPB_vect) {
  PORTD &= ~(1 << 5);
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
  PORTD |= (1 << 4);
}
ISR(TIMER0_OVF_vect) {
  OCR0B = (lookUp[num3]);
  if (lookUp[num3] > 0) {
    PORTD &= ~(1 << 2);
  }
  if (num3 < samples) {
    num3 += 1;
  } else {
    num3 = 0;
  }
  if (t3_setup) {
    t3setup = true;
  }
  if (lookUp[num3] > 0) {
    PORTD |= (1 << 3);
  }
}
ISR(TIMER0_COMPB_vect) {
  PORTD &= ~(1 << 3);
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
  PORTD |= (1 << 2);
}