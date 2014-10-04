
// Debug info
#define DEBUG

#define __spi_clock 13   // SCK - hardware SPI
#define __spi_data 11    // MOSI - hardware SPI
#define __spi_data_in 12 // MISO - hardware SPI (unused)
#define __spi_latch 10   // SS - hardware SPI

#define __TIMER1_MAX 0xFFFF // 16 bit CTR
#define __TIMER1_CNT 0x0130 // 32 levels --> 0x0130; 38 --> 0x0157 (flicker)

#define __rows 8
#define __max_row __rows-1
#define __leds_per_row 8
#define __max_led __leds_per_row-1
#define __brightness_levels 20 // 0...15 above 28 is bad for ISR
#define __max_brightness __brightness_levels-1

byte brightness_red[__leds_per_row][__rows]; 
byte brightness_green[__leds_per_row][__rows];
byte brightness_blue[__leds_per_row][__rows]; 

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
  pinMode(__spi_clock,OUTPUT);
  pinMode(__spi_latch,OUTPUT);
  pinMode(__spi_data,OUTPUT);
  //pinMode(__spi_data_in,INPUT);
  digitalWrite(__spi_latch,LOW);
  digitalWrite(__spi_data,LOW);
  digitalWrite(__spi_clock,LOW);
  setup_hardware_spi();
  delay(10);

  set_matrix_rgb(0,0,0);
  setup_timer1_ovf();
  matrix_test();
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
      spi_transfer(blue);
      spi_transfer(green);
      spi_transfer(red);
      spi_transfer(B00000001<<row);
      digitalWrite(__spi_latch,HIGH);
      digitalWrite(__spi_latch,LOW);
    }
  }
}

void loop() {
  set_matrix_rgb(0,0,0);

  set_row_byte_blue(0, B00111100, __max_brightness);
  set_row_byte_blue(1, B01111110, __max_brightness);
  set_row_byte_blue(2, B11111111, __max_brightness);
  set_row_byte_blue(3, B11111001, __max_brightness);
  set_row_byte_blue(4, B11111001, __max_brightness);
  set_row_byte_blue(5, B11111111, __max_brightness);
  set_row_byte_blue(6, B01111110, __max_brightness);
  set_row_byte_blue(7, B00111100, __max_brightness);
  delay(1000);
  set_row_byte_blue(3, B11111111, __max_brightness);
  set_row_byte_blue(4, B11110011, __max_brightness);
  set_row_byte_blue(5, B11110011, __max_brightness);
  delay(500);
  set_row_byte_blue(4, B11111111, __max_brightness);
  set_row_byte_blue(5, B11100111, __max_brightness);
  set_row_byte_blue(6, B01100110, __max_brightness);
  delay(1000);

  set_row_byte_blue(0, B00000000, __max_brightness);
  delay(100);
  set_row_byte_blue(1, B00000000, __max_brightness);
  set_row_byte_blue(2, B00111100, __max_brightness);
  set_row_byte_blue(4, B11111111, __max_brightness);
  set_row_byte_blue(5, B11111111, __max_brightness);
  set_row_byte_blue(6, B00111100, __max_brightness);
  set_row_byte_blue(7, B00000000, __max_brightness);
  delay(100);
  set_row_byte_blue(2, B00000000, __max_brightness);
  set_row_byte_blue(3, B00111100, __max_brightness);
  set_row_byte_blue(5, B01111110, __max_brightness);
  set_row_byte_blue(6, B00011000, __max_brightness);
  delay(100);
  set_row_byte_blue(3, B00000000, __max_brightness);
  set_row_byte_blue(4, B10000001, __max_brightness);
  set_row_byte_blue(6, B00000000, __max_brightness);
  delay(100);
  set_row_byte_blue(3, B00111100, __max_brightness);
  set_row_byte_blue(4, B11111111, __max_brightness);  
  set_row_byte_blue(6, B00011000, __max_brightness);
  delay(100);
  set_row_byte_blue(2, B00111100, __max_brightness);
  set_row_byte_blue(3, B11111111, __max_brightness);
  set_row_byte_blue(5, B11111111, __max_brightness);
  set_row_byte_blue(6, B00111100, __max_brightness);
  delay(100);
  set_row_byte_blue(0, B00111100, __max_brightness);
  set_row_byte_blue(1, B01111110, __max_brightness);
  set_row_byte_blue(2, B11111111, __max_brightness);
  set_row_byte_blue(6, B01111110, __max_brightness);
  set_row_byte_blue(7, B00111100, __max_brightness);
  delay(1000);

  set_row_byte_blue(3, B11111001, __max_brightness);
  set_row_byte_blue(4, B11111001, __max_brightness);
  set_row_byte_red(3, B00000110, __max_brightness);
  set_row_byte_red(4, B00000110, __max_brightness);
  delay(1000);
  set_row_byte_blue(3, B11111111, __max_brightness);
  set_row_byte_blue(4, B11110011, __max_brightness);
  set_row_byte_blue(5, B11110011, __max_brightness);
  set_row_byte_red(3, B00000000, __max_brightness);
  set_row_byte_red(4, B00001100, __max_brightness);
  set_row_byte_red(5, B00001100, __max_brightness);
  delay(500);
  set_row_byte_blue(4, B11111111, __max_brightness);
  set_row_byte_blue(5, B11100111, __max_brightness);
  set_row_byte_blue(6, B01100110, __max_brightness);
  set_row_byte_red(4, B00000000, __max_brightness);
  set_row_byte_red(5, B00011000, __max_brightness);
  set_row_byte_red(6, B00011000, __max_brightness);
  delay(1000);
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

void setup_hardware_spi(void) {
  byte clr;
  // spi prescaler: 
  // SPI2X SPR1 SPR0
  //   0     0     0    fosc/4
  //   0     0     1    fosc/16
  //   0     1     0    fosc/64
  //   0     1     1    fosc/128
  //   1     0     0    fosc/2
  //   1     0     1    fosc/8
  //   1     1     0    fosc/32
  //   1     1     1    fosc/64
  SPCR |= ( (1<<SPE) | (1<<MSTR) ); // enable SPI as master
  //SPCR |= ( (1<<SPR1) ); // set prescaler bits
  SPCR &= ~ ( (1<<SPR1) | (1<<SPR0) ); // clear prescaler bits
  clr=SPSR; // clear SPI status reg
  clr=SPDR; // clear SPI data reg
  SPSR |= (1<<SPI2X); // set prescaler bits
  //SPSR &= ~(1<<SPI2X); // clear prescaler bits
}

byte spi_transfer(byte data)
{
  SPDR = data;                    // Start the transmission
  while (!(SPSR & (1<<SPIF)))     // Wait the end of the transmission
  {
  };
  return SPDR;                    // return the received byte, we don't need that
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
  set_row_byte_rgb(row,data_byte,red,0,0);
}

void set_row_byte_green(byte row, byte data_byte, byte green) {
  set_row_byte_rgb(row,data_byte,0,green,0);
}

void set_row_byte_blue(byte row, byte data_byte, byte blue) {
  set_row_byte_rgb(row,data_byte,0,0,blue);
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

void matrix_test() {
  set_matrix_rgb(__max_brightness,0,0);
  delay(1000);
  set_matrix_rgb(0,__max_brightness,0);
  delay(1000);
  set_matrix_rgb(0,0,__max_brightness);
  delay(1000);
  set_matrix_rgb(0,0,0);
  byte ctr1;
  byte ctr2;
  for(ctr2 = 0; ctr2 <= __max_row; ctr2++) {
    for(ctr1 = 0; ctr1 <= __max_led; ctr1++) {
      set_led_rgb(ctr2,ctr1,__max_brightness,0,0);
      delay(200);
      set_led_rgb(ctr2,ctr1,0,__max_brightness,0);
      delay(200);
      set_led_rgb(ctr2,ctr1,0,0,__max_brightness);
      delay(200);
    }
  }
  set_matrix_rgb(0,0,0);
  delay(1000);
}
