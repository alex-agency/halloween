#include <SPI.h>

#define DEBUG

#define __spi_latch 9
#define __pir_pin 7
#define __trig_pin 6
#define __echo_pin 5

#define __TIMER1_MAX 0xFFFF // 16 bit CTR
#define __TIMER1_CNT 0x130 // 32 levels --> 0x0130; 38 --> 0x0157 (flicker)

#define __rows 8
#define __max_row __rows-1
#define __leds_per_row 8
#define __max_led __leds_per_row-1
#define __brightness_levels 30 //50 for rgb only
#define __max_brightness __brightness_levels-1

byte brightness_red[__leds_per_row][__rows]; 
byte brightness_green[__leds_per_row][__rows];
byte brightness_blue[__leds_per_row][__rows];

#define NICE_EYE 0
#define EVIL_EYE 40 // 5*8
#define BORED_EYE 80 // 10*8

uint8_t blinkIndex[] = { 1, 2, 3, 4, 3, 2, 1 }; // Blink bitmap sequence
uint8_t blinkCountdown = 100; // Countdown to next blink (in frames)
uint8_t gazeCountdown  =  50; // Countdown to next eye movement
uint8_t gazeFrames     =  20; // Duration of eye movement (smaller = faster)
uint8_t eyeOffset = BORED_EYE;
uint8_t pupilX = 3, pupilY = 3;   // Current pupil position
uint8_t newX = 3, newY = 3;   // Next pupil position
int8_t dX   = 0, dY   = 0;   // Distance from prior to new position

extern volatile unsigned long timer0_overflow_count;
unsigned long hpticks ()
{
    return (timer0_overflow_count << 8) + TCNT0;
}
unsigned long last_micros = hpticks()*4;
unsigned long distance = 500;
bool wait = false;

static const uint8_t PROGMEM
blinkImg[][8] = {    // Eye animation frames
    // The NICE eye
    { B00111100,         // Fully open nice eye
        B01111110,
        B11111111,
        B11111111,
        B11111111,
        B11111111,
        B01111110,
        B00111100 },
    { B00000000,
        B01111110,
        B11111111,
        B11111111,
        B11111111,
        B11111111,
        B01111110,
        B00111100 },
    { B00000000,
        B00000000,
        B00111100,
        B11111111,
        B11111111,
        B11111111,
        B00111100,
        B00000000 },
    { B00000000,
        B00000000,
        B00000000,
        B00111100,
        B11111111,
        B01111110,
        B00011000,
        B00000000 },
    { B00000000,         // Fully closed nice eye
        B00000000,
        B00000000,
        B00000000,
        B10000001,
        B01111110,
        B00000000,
        B00000000 },

    // The EVIL eye
    { B00000000,         // Fully open evil eye
        B11000000,
        B11111000,
        B01111111,
        B01111111,
        B00111111,
        B00111110,
        B00011100 },
    { B00000000,
        B00000000,
        B11111000,
        B01111111,
        B01111111,
        B00111111,
        B00111110,
        B00011100 },
    { B00000000,
        B00000000,
        B11111000,
        B01111111,
        B01111111,
        B00111111,
        B00111110,
        B00000000 },
    { B00000000,
        B00000000,
        B00000000,
        B01111111,
        B01111111,
        B00111111,
        B00000000,
        B00000000 },
    { B00000000,         // Fully closed evil eye
        B00000000,
        B00000000,
        B00000000,
        B01111111,
        B00000000,
        B00000000,
        B00000000 },
    // The BORED eye
    { B00000000,         // Fully open bored eye
        B00000000,
        B11111110,
        B11111110,
        B11000110,
        B01111100,
        B00000000,
        B00000000 },
    { B00000000,
        B00000000,
        B00000000,
        B11111110,
        B11000110,
        B01111100,
        B00000000,
        B00000000 },
    { B00000000,
        B00000000,
        B00000000,
        B00000000,
        B11000110,
        B01111100,
        B00000000,
        B00000000 },
    { B00000000,
        B00000000,
        B00000000,
        B00000000,
        B10000010,
        B01111100,
        B00000000,
        B00000000 },
    { B00000000,         // Fully closed bored eye
        B00000000,
        B00000000,
        B00000000,
        B00000000,
        B01111100,
        B00000000,
        B00000000 }
};

// Declare output
static int serial_putchar(char c, FILE *) {
  Serial.write(c);
  return 0;
};
FILE serial_out = {0};

void setup() {
  // Configure output
  Serial.begin(9600);
  fdev_setup_stream(&serial_out, serial_putchar, NULL, _FDEV_SETUP_WRITE);
  stdout = stderr = &serial_out;
  // Initialize SPI
  SPI.begin();
  pinMode(__spi_latch,OUTPUT);
  delay(10);

  set_matrix_rgb(0,0,0);
  setup_timer1_ovf();
  matrix_test(25);

  pinMode(__pir_pin,INPUT);

  pinMode(__trig_pin, OUTPUT);
  pinMode(__echo_pin, INPUT);
}

ISR(TIMER1_OVF_vect) {
  TCNT1 = __TIMER1_MAX - __TIMER1_CNT;
  byte cycle;

  for(cycle = 0; cycle < __max_brightness; cycle++) {
    byte led;
    byte row = B00000000;    // row: current source. on when (1)
    byte red;    // current sinker when on (0)
    byte green;  // current sinker when on (0)
    byte blue;   // current sinker when on (0)

    for(row = 0; row <= __max_row; row++) {
      red = B11111111;    // off
      green = B11111111;  // off
      blue = B11111111;   // off
      
      for(led = 0; led <= __max_led; led++) {   
        if(row == __max_row && cycle >= __max_brightness-1)
          break;
        
        if(cycle < brightness_red[row][led]) {
          red &= ~(1<<led);
        }
        if(cycle < brightness_green[row][led]) {
          green &= ~(1<<led);
        }
        if(cycle < brightness_blue[row][led]) {
          blue &= ~(1<<led);
        }
      }

      digitalWrite(__spi_latch,LOW);
      SPI.transfer(B00000001<<row);
      SPI.transfer(blue);
      SPI.transfer(green);
      SPI.transfer(red);
      digitalWrite(__spi_latch,HIGH);
    }
  }

  // measure distance from ultrasonic
  if(last_micros+100000 < hpticks() && wait == false) {
    digitalWrite(__trig_pin,LOW);   
    last_micros = hpticks();
    if(last_micros < hpticks()) { // 2 microsec delay
      digitalWrite(__trig_pin,HIGH);      
      wait = true;
      last_micros = hpticks();
    }
  }
  if(wait && last_micros+3 < hpticks()) { // 10 microsec delay
    digitalWrite(__trig_pin,LOW);   
    distance = pulseIn(__echo_pin,HIGH,15000);         
    distance = distance / 58;
    wait = false;
  }
}

void loop() 
{  
  #ifdef DEBUG
    printf_P(PSTR("cm: %d\n\r"), distance);
  #endif
  
  if(digitalRead(__pir_pin) == LOW)
    eyeOffset = BORED_EYE;
  else
    eyeOffset = NICE_EYE;

  const uint8_t* eye = 
            &blinkImg[
            (blinkCountdown < sizeof(blinkIndex)) ? // Currently blinking?
            blinkIndex[blinkCountdown] :            // Yes, look up bitmap #
            0                                       // No, show bitmap 0
            ][0] + eyeOffset;
  
  blinkCountdown--;
    if (blinkCountdown == 0) 
        blinkCountdown = random(5, 180);

  if(--gazeCountdown <= gazeFrames && eyeOffset != BORED_EYE) {
    // pupil in motion - draw pupil at interim position
    drawEyes(eye, 
      newX - (dX * gazeCountdown / gazeFrames),
      newY - (dY * gazeCountdown / gazeFrames));
    
    if(gazeCountdown == 0) {    // Last frame?
      pupilX = newX; pupilY = newY;
      
      if(distance < 100 && distance != 0) {
        eyeOffset = EVIL_EYE;
      }  
      // Pick random positions
      if(eyeOffset == NICE_EYE) {
        newX = random(1,6); newY = random(2,5);
      }
      else if(eyeOffset == EVIL_EYE) {
        newX = random(2,6); newY = 4;
      }
        
      dX            = newX - pupilX;           // Horizontal distance to move
      dY            = newY - pupilY;           // Vertical distance to move
      gazeFrames    = random(3, 15);           // Duration of pupil movement
      gazeCountdown = random(gazeFrames, 120); // Count to end of next movement
    }
  } else if(eyeOffset == BORED_EYE) {
      drawEyes(eye, 8, 8);
  } else {
      // Not in motion yet -- draw pupil at current static position
      drawEyes(eye, pupilX, pupilY);
  }
}

void drawEyes(const uint8_t* eye, uint8_t pupilX, uint8_t pupilY) {
  for(byte ctr1 = 0; ctr1 <= __max_row; ctr1++) {
      byte image = bit_reverse(pgm_read_byte_near(eye+ctr1));
      
      if(ctr1==pupilY || ctr1==pupilY+1) {
        image = image-(3<<pupilX);
      }
      
      if(eyeOffset == EVIL_EYE) {
        set_row_byte_rgb(ctr1,image,__max_brightness,0,0);
        //set_row_byte_hue(ctr1,image,300);
      } else if(eyeOffset == NICE_EYE) {
        set_row_byte_rgb(ctr1,image,0,__max_brightness,0);
        //set_row_byte_hue(ctr1,image,100);
      } else {
        set_row_byte_rgb(ctr1,image,0,0,__max_brightness);
        //set_row_byte_hue(ctr1,image,240);
      }
  }
  #ifdef DEBUG
    //printf_P(PSTR("pupil: x=%d, y=%d\n\r"), pupilX, pupilY);
  #endif
  delay(5);
}

byte bit_reverse( byte x ) { 
  x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa); 
  x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc); 
  x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0); 
  return x;    
}

void setup_timer1_ovf() {
  // Arduino runs at 16 Mhz...
  // Timer1 (16bit) Settings:
  // prescaler (frequency divider) values:   CS12    CS11   CS10
  //                                           0       0      0    stopped
  //                                           0       0      1      /1  
  //                                           0       1      0      /8  
  //                                           0       1      1      /64
  //                                           1       0      0      /256 
  //                                           1       0      1      /1024
  //                                           1       1      0      external clock on T1 pin, falling edge
  //                                           1       1      1      external clock on T1 pin, rising edge
  //
  TCCR1B &= ~ ( (1<<CS11) );
  TCCR1B |= ( (1<<CS12) | (1<<CS10) );      
  // normal mode
  TCCR1B &= ~ ( (1<<WGM13) | (1<<WGM12) );
  TCCR1A &= ~ ( (1<<WGM11) | (1<<WGM10) );
  // Timer1 Overflow Interrupt Enable  
  TIMSK1 |= (1<<TOIE1);
  TCNT1 = __TIMER1_MAX - __TIMER1_CNT;
  // enable all interrupts
  sei(); 
}

void set_led_red(byte row, byte led, byte red) {
  brightness_red[row][led] = red;
}


void set_led_green(byte row, byte led, byte green) {
  brightness_green[row][led] = green;
}


void set_led_blue(byte row, byte led, byte blue) {
  brightness_blue[row][led] = blue;
}

void set_led_rgb(byte row, byte led, byte red, byte green, byte blue) {
  set_led_red(row,led,red);
  set_led_green(row,led,green);
  set_led_blue(row,led,blue);
}

void set_row_rgb(byte row, byte red, byte green, byte blue) {
  byte ctr1;
  for(ctr1 = 0; ctr1 <= __max_led; ctr1++) {
      set_led_rgb(row,ctr1,red,green,blue);
  }
}

void set_column_rgb(byte column, byte red, byte green, byte blue) {
  byte ctr1;
  for(ctr1 = 0; ctr1 <= __max_row; ctr1++) {
      set_led_rgb(ctr1,column,red,green,blue);
  }
}

void set_row_byte_rgb(byte row, byte data_byte, byte red, byte green, byte blue) {
  byte led;
  for(led = 0; led <= __max_led; led++) {
    if( (data_byte>>led)&(B00000001) ) {
      set_led_rgb(row,led,red,green,blue);
    }
    else {
      set_led_rgb(row,led,0,0,0);
    }
  }
}

void set_row_byte_red(byte row, byte data_byte, byte red) {
  byte led;
  for(led = 0; led <= __max_led; led++) {
    if( (data_byte>>led)&(B00000001) ) {
      set_led_red(row,led,red);
    }
    else {
      set_led_red(row,led,0);
    }
  }
}

void set_row_byte_green(byte row, byte data_byte, byte green) {
  byte led;
  for(led = 0; led <= __max_led; led++) {
    if( (data_byte>>led)&(B00000001) ) {
      set_led_green(row,led,green);
    }
    else {
      set_led_green(row,led,0);
    }
  }
}

void set_row_byte_blue(byte row, byte data_byte, byte blue) {
  byte led;
  for(led = 0; led <= __max_led; led++) {
    if( (data_byte>>led)&(B00000001) ) {
      set_led_blue(row,led,blue);
    }
    else {
      set_led_blue(row,led,0);
    }
  }
}

void set_matrix_rgb(byte red, byte green, byte blue) {
  byte ctr1;
  byte ctr2;
  for(ctr2 = 0; ctr2 <= __max_row; ctr2++) {
    for(ctr1 = 0; ctr1 <= __max_led; ctr1++) {
      set_led_rgb(ctr2,ctr1,red,green,blue);
    }
  }
}

void set_led_hue(byte row, byte led, int hue) {
  // see wikipeda: HSV
  float S=100.0,V=100.0,s=S/100.0,v=V/100.0,h_i,f,p,q,t,R,G,B;
    
    hue = hue%360;
    h_i = hue/60;            
    f = (float)(hue)/60.0 - h_i;
    p = v*(1-s);
    q = v*(1-s*f);
    t = v*(1-s*(1-f));
    
    if      ( h_i == 0 ) { 
      R = v; 
      G = t; 
      B = p;
    }
    else if ( h_i == 1 ) { 
      R = q; 
      G = v; 
      B = p;
    }
    else if ( h_i == 2 ) { 
      R = p; 
      G = v; 
      B = t;
    }
    else if ( h_i == 3 ) { 
      R = p; 
      G = q; 
      B = v;
    }
    else if ( h_i == 4 ) { 
      R = t; 
      G = p; 
      B = v;
    }
    else                   { 
      R = v; 
      G = p; 
      B = q;
    }

    set_led_rgb(row,led,byte(R*(float)(__max_brightness)),byte(G*(float)(__max_brightness)),byte(B*(float)(__max_brightness)));   
}

void set_row_byte_hue(byte row, byte data_byte, int hue) {
  byte led;
  for(led = 0; led <= __max_led; led++) {
    if( (data_byte>>led)&(B00000001) ) {
      set_led_hue(row,led,hue);
    }
    else {
      set_led_rgb(row,led,0,0,0);
    }
  }
}

void matrix_test(int speed) {
  set_row_byte_rgb(0,1,__max_brightness,__max_brightness,__max_brightness);
  delay(speed*10);
  set_matrix_rgb(0,0,0);

  byte ctr1;
  byte ctr2;
  for(ctr1 = 0; ctr1 <= __max_brightness; ctr1++) {
    set_matrix_rgb(ctr1,0,0);
    delay(speed/3);
  }
  delay(speed);
  for(ctr1 = 0; ctr1 <= __max_brightness; ctr1++) {
    set_matrix_rgb(0,ctr1,0);
    delay(speed/3);
  }
  delay(speed);
  for(ctr1 = 0; ctr1 <= __max_brightness; ctr1++) {
    set_matrix_rgb(0,0,ctr1);
    delay(speed/3);
  }
  delay(speed);
  set_matrix_rgb(0,0,0);
  
  for(ctr1 = 0; ctr1 <= __max_row; ctr1++) {
    set_row_rgb(ctr1,__max_brightness,0,0);
    delay(speed);
    set_row_rgb(ctr1,0,__max_brightness,0);
    delay(speed);
    set_row_rgb(ctr1,0,0,__max_brightness);
    delay(speed);
    set_matrix_rgb(0,0,0);
  }

  for(ctr1 = 0; ctr1 <= __max_row; ctr1++) {
    set_column_rgb(ctr1,__max_brightness,0,0);
    delay(speed);
    set_column_rgb(ctr1,0,__max_brightness,0);
    delay(speed);
    set_column_rgb(ctr1,0,0,__max_brightness);
    delay(speed);
    set_matrix_rgb(0,0,0);
  }
  
  for(ctr1 = 0; ctr1 < speed*10; ctr1++) { 
    set_led_hue((byte)(random(__rows)),(byte)(random(__leds_per_row)),(int)(random(360)));
  }
  delay(speed*2);
  set_matrix_rgb(0,0,0);
}
