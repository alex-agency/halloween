#include <SPI.h>

#define DEBUG

#define SPI_LATCH_PIN  9
#define PIR_PIN  7
#define TRIG_PIN  6
#define ECHO_PIN  5
#define RELAY1_PIN  4
#define RELAY2_PIN  3

#define TIMER1_MAX  0xFFFF // 16 bit CTR
#define TIMER1_CNT  0x130 // 32 levels --> 0x0130; 38 --> 0x0157 (flicker)

#define MATRIX_COUNT  2
#define MATRIX_ROWS  8
#define ROW_LEDS  8
#define MATRIX_BRIGHTNESS  21

byte RED_MATRIX[MATRIX_COUNT][ROW_LEDS][MATRIX_ROWS]; 
byte GREEN_MATRIX[MATRIX_COUNT][ROW_LEDS][MATRIX_ROWS];
byte BLUE_MATRIX[MATRIX_COUNT][ROW_LEDS][MATRIX_ROWS];

#define NICE_EYE  0
#define EVIL_EYE  40 // 5*8
#define BORED_EYE  80 // 10*8

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
        B00111111,
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

uint8_t blinkIndex[] = { 1, 2, 3, 4, 3, 2, 1 }; // Blink bitmap sequence
uint8_t blinkCountdown = 100; // Countdown to next blink (in frames)
uint8_t gazeCountdown  =  50; // Countdown to next eye movement
uint8_t gazeFrames     =  20; // Duration of eye movement (smaller = faster)
uint8_t eyeOffset = BORED_EYE;
uint8_t pupilX = 3, pupilY = 3; // Current pupil position
uint8_t newX = 3, newY = 3; // Next pupil position
int8_t  dX   = 0, dY   = 0; // Distance from prior to new position
unsigned long sonar = 200; // Distance to sonar in cm
uint8_t flashCountdown = 0; // flash duration
bool pir = false;

// Declare serial output
static int serial_putchar(char c, FILE *) {
  Serial.write(c);
  return 0;
};
FILE serial_out = {0};

void setup() {
  // Configure serial output
  Serial.begin(9600);
  fdev_setup_stream(&serial_out, serial_putchar, NULL, _FDEV_SETUP_WRITE);
  stdout = stderr = &serial_out;
  // Initialize SPI
  SPI.begin();
  pinMode(SPI_LATCH_PIN,OUTPUT);
  delay(10);
  matrix_clear(); 
  // Initialize timer1
  setup_timer1_ovf();
  matrix_test(20);
  // Initialize PIR sensor
  pinMode(PIR_PIN,INPUT_PULLUP);
  // Initialize sonar
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  // Initialize relays
  pinMode(RELAY1_PIN, OUTPUT);
  digitalWrite(RELAY1_PIN, LOW);
  pinMode(RELAY2_PIN, OUTPUT);
  digitalWrite(RELAY2_PIN, LOW);
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
  TCNT1 = TIMER1_MAX - TIMER1_CNT;
  // enable all interrupts
  sei(); 
}

ISR(TIMER1_OVF_vect) {
  TCNT1 = TIMER1_MAX-TIMER1_CNT;

  byte row;
  byte led;
  byte red1;    // current sinker when on (0)
  byte green1;  // current sinker when on (0)
  byte blue1;   // current sinker when on (0)
  byte red2;    // current sinker when on (0)
  byte green2;  // current sinker when on (0)
  byte blue2;   // current sinker when on (0)

  for(byte cycle = 0; cycle < MATRIX_BRIGHTNESS; cycle++) {
    row = B00000000;    // row: current source. on when (1)
      
    for(row = 0; row < MATRIX_ROWS; row++) {
      red1 = B11111111;    // off
      green1 = B11111111;  // off
      blue1 = B11111111;   // off
      red2 = B11111111;    // off
      green2 = B11111111;  // off
      blue2 = B11111111;   // off
      
      for(led = 0; led < ROW_LEDS; led++) {   
        if(cycle >= MATRIX_BRIGHTNESS-1 && row == MATRIX_ROWS-1)
          break;
        
        if(cycle < RED_MATRIX[0][row][led]) {
          red1 &= ~(1<<led);
        }
        if(cycle < GREEN_MATRIX[0][row][led]) {
          green1 &= ~(1<<led);
        }
        if(cycle < BLUE_MATRIX[0][row][led]) {
          blue1 &= ~(1<<led);
        }
        if(cycle < RED_MATRIX[1][row][led]) {
          red2 &= ~(1<<led);
        }
        if(cycle < GREEN_MATRIX[1][row][led]) {
          green2 &= ~(1<<led);
        }
        if(cycle < BLUE_MATRIX[1][row][led]) {
          blue2 &= ~(1<<led);
        }
     
      }

      digitalWrite(SPI_LATCH_PIN,LOW);
      SPI.transfer(B00000001<<row);  
      SPI.transfer(blue2);
      SPI.transfer(green2);
      SPI.transfer(red2);
      SPI.transfer(B00000001<<row);  
      SPI.transfer(blue1);
      SPI.transfer(green1);
      SPI.transfer(red1);
      digitalWrite(SPI_LATCH_PIN,HIGH);
    }
  }
}

void loop() 
{  
  if(sonar < 100) {
    eyeOffset = EVIL_EYE;
    if(pupilY != 4) {
      newY = 4; pupilY = 4;
      newX = 3; pupilX = 3;
    }
    if(pir) {
      flashCountdown = 5;
    }
    if(flashCountdown == 0) {
      digitalWrite(RELAY1_PIN, LOW);
      digitalWrite(RELAY2_PIN, LOW);
      delay(1000);
      pir = digitalRead(PIR_PIN);
    }

  } else if(digitalRead(PIR_PIN) == LOW) {
    eyeOffset = BORED_EYE;
  } else {
    eyeOffset = NICE_EYE;
  }

  flash();

  blinkCountdown--;
  if (blinkCountdown == 0) { 
    blinkCountdown = random(5, 150);
    
    sonar = measurement();
    #ifdef DEBUG
      //printf_P(PSTR("cm: %d\n\r"), sonar);
    #endif
  }
  
  const uint8_t* eye = 
            &blinkImg[
            (blinkCountdown < sizeof(blinkIndex)) ? // Currently blinking?
            blinkIndex[blinkCountdown] :            // Yes, look up bitmap #
            0                                       // No, show bitmap 0
            ][0] + eyeOffset;

  if(--gazeCountdown <= gazeFrames && eyeOffset != BORED_EYE) {
    // pupil in motion - draw pupil at interim position
    drawEyes(eye, 
      newX - (dX * gazeCountdown / gazeFrames),
      newY - (dY * gazeCountdown / gazeFrames));
    
    if(gazeCountdown == 0) {    // Last frame?
      pupilX = newX; pupilY = newY; 
      // Pick random positions
      if(eyeOffset == NICE_EYE) {
        newX = random(1,6); newY = random(2,5);
      }
      else if(eyeOffset == EVIL_EYE) {
        newX = random(2,5); newY = 4;
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

void matrix_clear() {
  for(byte matrix = 0; matrix <= MATRIX_COUNT-1; matrix++) {
    for(byte row = 0; row <= MATRIX_ROWS-1; row++) {
      for(byte led = 0; led <= ROW_LEDS-1; led++) {
        set_led_rgb(matrix,row,led,0,0,0);
      }
    }
  }
}

void set_led_rgb(uint8_t matrix, byte row, byte led, byte red, byte green, byte blue) {
  RED_MATRIX[matrix][row][led] = red;
  GREEN_MATRIX[matrix][row][led] = green;
  BLUE_MATRIX[matrix][row][led] = blue;
}

void set_row_rgb(uint8_t matrix, byte row, byte data_byte, byte red, byte green, byte blue) {
  for(byte led = 0; led <= ROW_LEDS-1; led++) {
    if( (data_byte>>led)&(B00000001) ) {
      set_led_rgb(matrix,row,led,red,green,blue);
    }
    else {
      set_led_rgb(matrix,row,led,0,0,0);
    }
  }
}

void set_row_hue(uint8_t matrix, byte row, byte data_byte, uint8_t hue) {
  for(byte led = 0; led <= ROW_LEDS-1; led++) {
    if( (data_byte>>led)&(B00000001) ) {
      set_led_hue(matrix,row,led,hue);
    }
    else {
      set_led_rgb(matrix,row,led,0,0,0);
    }
  }
}

void set_led_hue(uint8_t matrix, byte row, byte led, uint8_t hue) {
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

    set_led_rgb(matrix,row,led,
      byte(R*(float)(MATRIX_BRIGHTNESS)),
      byte(G*(float)(MATRIX_BRIGHTNESS)),
      byte(B*(float)(MATRIX_BRIGHTNESS)));   
}

void matrix_test(int speed) {
  // left top white point
  for(byte matrix = 0; matrix <= MATRIX_COUNT-1; matrix++) {
    set_led_rgb(matrix,0,0,
      MATRIX_BRIGHTNESS,MATRIX_BRIGHTNESS,MATRIX_BRIGHTNESS);
  }
  delay(speed*5);
  // red matrix brightness
  for(byte ctr1 = 0; ctr1 <= MATRIX_BRIGHTNESS; ctr1++) {
    for(byte matrix = 0; matrix <= MATRIX_COUNT-1; matrix++) {
      for(byte row = 0; row <= MATRIX_ROWS-1; row++) {
        set_row_rgb(matrix,row,B11111111,ctr1,0,0);
      }
    }
    delay(speed/4);
  }
  delay(speed);
  // green matrix brightness
  for(byte ctr1 = 0; ctr1 <= MATRIX_BRIGHTNESS; ctr1++) {
    for(byte matrix = 0; matrix <= MATRIX_COUNT-1; matrix++) {
      for(byte row = 0; row <= MATRIX_ROWS-1; row++) {
        set_row_rgb(matrix,row,B11111111,0,ctr1,0);
      }
    }
    delay(speed/4);
  }
  delay(speed);
  // blue matrix brightness
  for(byte ctr1 = 0; ctr1 <= MATRIX_BRIGHTNESS; ctr1++) {
    for(byte matrix = 0; matrix <= MATRIX_COUNT-1; matrix++) {
      for(byte row = 0; row <= MATRIX_ROWS-1; row++) {
        set_row_rgb(matrix,row,B11111111,0,0,ctr1);
      }
    }
    delay(speed/4);
  }
  delay(speed);
  matrix_clear();
  // rgb column shift
  for(byte matrix = 0; matrix <= MATRIX_COUNT-1; matrix++) {
    for(byte led = 0; led <= ROW_LEDS-1; led++) {
      for(byte row = 0; row <= MATRIX_ROWS-1; row++) {
        set_led_rgb(matrix,row,led,MATRIX_BRIGHTNESS,0,0);
      }
      delay(speed);
      for(byte row = 0; row <= MATRIX_ROWS-1; row++) {
        set_led_rgb(matrix,row,led,0,MATRIX_BRIGHTNESS,0);
      }
      delay(speed);
      for(byte row = 0; row <= MATRIX_ROWS-1; row++) {
        set_led_rgb(matrix,row,led,0,0,MATRIX_BRIGHTNESS);
      }
      delay(speed);
      for(byte row = 0; row <= MATRIX_ROWS-1; row++) {
        set_led_rgb(matrix,row,led,0,0,0);
      }
    }
  }
  matrix_clear();
  // rgb row shift
  for(byte row = 0; row <= MATRIX_ROWS-1; row++) {
    for(byte matrix = 0; matrix <= MATRIX_COUNT-1; matrix++) {
      set_row_rgb(matrix,row,B11111111,MATRIX_BRIGHTNESS,0,0);
    }
    delay(speed);
    for(byte matrix = 0; matrix <= MATRIX_COUNT-1; matrix++) {
      set_row_rgb(matrix,row,B11111111,0,MATRIX_BRIGHTNESS,0);
    }
    delay(speed);
    for(byte matrix = 0; matrix <= MATRIX_COUNT-1; matrix++) {
      set_row_rgb(matrix,row,B11111111,0,0,MATRIX_BRIGHTNESS);
    }
    delay(speed);
    matrix_clear();
  }
  matrix_clear();
  // random led
  for(byte ctr1 = 0; ctr1 < speed*12; ctr1++) { 
    for(byte matrix = 0; matrix <= MATRIX_COUNT-1; matrix++) {
      set_led_hue(matrix,
        (byte)(random(MATRIX_ROWS)),
        (byte)(random(ROW_LEDS)),
        (int)(random(360)));
    }
  }
  delay(speed*10);
  matrix_clear();
}

void drawEyes(const uint8_t* eyePoint, uint8_t pupilX, uint8_t pupilY) {
  
  for(byte row = 0; row < MATRIX_ROWS; row++) {
      for(byte matrix = 0; matrix < MATRIX_COUNT; matrix++) {
      
      // right eye
      byte eye = pgm_read_byte_near(eyePoint+row);
      if(matrix == 0) {
        // left eye
        eye = bit_reverse(eye);
      }

      // insert pupil
      if(row==pupilY || row==pupilY+1)  {
        eye = eye-(3<<pupilX);
      }

      if(eyeOffset == EVIL_EYE) {
        set_row_hue(matrix,row,eye,0);
      } else if(eyeOffset == NICE_EYE) {
        set_row_hue(matrix,row,eye,120);
      } else {
        set_row_hue(matrix,row,eye,200);
      }
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

unsigned long measurement() {
  unsigned long cm;
  noInterrupts();
  digitalWrite(TRIG_PIN,LOW);
  delayMicroseconds(2);  
  digitalWrite(TRIG_PIN,HIGH);
  delayMicroseconds(10);      
  digitalWrite(TRIG_PIN,LOW);   
  cm = pulseIn(ECHO_PIN,HIGH,12000);
  interrupts();
  if(cm == 0)
    return 200;    
  return cm / 58;
}

void flash() {
  if(flashCountdown == 0) {  
    return;
  }
  
  flashCountdown--;

  if(flashCountdown > random(1,100)) {
    digitalWrite(RELAY1_PIN, HIGH);
    return;
  }

  if(flashCountdown > random(1,100)) {
    digitalWrite(RELAY2_PIN, HIGH);
    return;
  }

  if(flashCountdown > random(1,100)) {
    digitalWrite(RELAY1_PIN, LOW);
    return;
  }

  if(flashCountdown > random(1,100)) {
    digitalWrite(RELAY2_PIN, LOW);
    return;
  }
}

