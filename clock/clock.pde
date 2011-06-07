// Originally from http://www.nearfuturelaboratory.com/2006/12/14/arduino-and-ds1306-real-time-clock/
// ATMega168 Code
#include <avr/interrupt.h>
#include <avr/io.h>
 
#define DATAOUT 10 //MOSI
#define DATAIN  11 //MISO
#define SPICLOCK  12 //sck
#define RTC_CHIPSELECT 9 // chip select (ss/ce) for RTC, active high
#define LED 13

 
byte clr;
char spi_transfer(volatile char data)
{
  /*
  Writing to the SPDR register begins an SPI transaction
   */
  SPDR = data;
  /*
  Loop right here until the transaction is complete. the SPIF bit is
   the SPI Interrupt Flag. When interrupts are enabled, and the
   SPIE bit is set enabling SPI interrupts, this bit will set when
   the transaction is finished.
   */
  while (!(SPSR & (1<<SPIF)))
  {
  };
  // received data appears in the SPDR register
  return SPDR;
}
 
void setup()
{
  char in_byte;
  clr = 0;
  in_byte = clr;
  Serial.begin(9600);
  // set direction of pins
  pinMode(LED, OUTPUT);
  pinMode(DATAOUT, OUTPUT);
  pinMode(DATAIN, INPUT);
  pinMode(SPICLOCK,OUTPUT);
  pinMode(RTC_CHIPSELECT,OUTPUT);
  digitalWrite(RTC_CHIPSELECT,LOW); //disable RTC
 
  // set up the RTC by enabling the oscillator, disabling the write protect in the control register,
  // enabling AIE0 and AIE1 and the 1HZ Output
  // 0x8F to 00000111 = 0x03
  // EOSC Active Low
  // WP Active High, so turn it off
  write_rtc_register(0x8F, 0x01|0x02|0x04);
 
  // little sanity checks
  in_byte = read_rtc_register(0x0F);
  Serial.print("CTRL REG [");
  Serial.print(in_byte, HEX);
  Serial.println("]");
  delay(10);
 
  in_byte = read_rtc_register(0x10);
  Serial.print("STATUS REG [");
  Serial.print(in_byte, BIN);
  Serial.println("]");
 
  // set up both alarms at 00 seconds?
  write_rtc_register(0x87,0x00);
  // mask all the other registers
  write_rtc_register(0x88,0x80);
  write_rtc_register(0x89,0x80);
  write_rtc_register(0x8A,0x80);
 
  write_rtc_register(0x8B,0x00);
  write_rtc_register(0x8C,0x80);
  write_rtc_register(0x8D,0x80);
  write_rtc_register(0x8E,0x80);
 
  in_byte = read_rtc_register(0x06);
  Serial.print("YEAR [");
  Serial.print(in_byte, HEX);
  Serial.println("]");
 
  in_byte = read_rtc_register(0x05);
  Serial.print("MONTH [");
  Serial.print(in_byte, HEX);
  Serial.println("]");
 
  // enable INT0, PORTD Bit 2 on the Atmega 8
  // we'll attach the 1HZ signal from the 1306, pin 7
  // or we can attach the active low interrupt output (pin 5 on the DS1306 DIP package)
  // to digital pin 3 on the Arduino (INT1 on the ATmega8)
  // I just picked INT1 arbitrarily — I think you could use any of the interrupts, so long as it
  // wasn't already being used by some other part of the Arduino.
  // cf. http://www.arduino.cc/en/Hacking/PinMapping?from=Main.PinMapping
  // enable INT1 in the global interrupt control register
 
  // Atmega8 - uncomment these two lines
  // Atmega168 - comment these two lines
  //GICR = (1<<INT1);
 //Serial.println(GICR, HEX);
 
  // Atmega168 - uncomment these four lines
  // Atmega8 - comment these four lines
  EIMSK = (1<<INT1); // activate the external interrupt INT1 mask
  EICRA = (0<<ISC11|1<<ISC10); // set the interrupt sensing so that the falling edge of INT1 generates an interrupt request.
  SREG = (1<<7); // put a 1 in the 7th bit of the microcontroller's status register to globally turn on interrupts
  Serial.println(EIMSK, HEX); // print out the EIMSK register just so we can see for ourselves..
 
  digitalWrite(LED, HIGH);
  delay(1000);
 
}
 
// can't really share variables unless they're declared
// "volatile", otherwise, they'll be set here, then popped
// back to their values before the interrupt handler was called
ISR(INT1_vect) {
  //signal that we have an interrupt
  //turn on the LED
  byte a;
 
  digitalWrite(LED, HIGH);
  // writing or reading from the DS1306 registers resets the alarm
  // so as to cause an alarm every 15 seconds.
  a = read_rtc_register(0x07);
  if(a == 0x00) {
    write_rtc_register(0x87,0x15);
  }
  if(a == 0x15) write_rtc_register(0x87,0x30);
  if(a == 0x30) write_rtc_register(0x87,0x45);
  if(a == 0x45) write_rtc_register(0x87,0x00);
 
  // every minute — set bit 7, the mask bit, to 1
  write_rtc_register(0x88,0x80);
  // every hour
  write_rtc_register(0x89,0x80);
  // every day of the week
  write_rtc_register(0x8A,0x80);
 
}
 
void write_rtc_register(char register_name, byte data) {
  write_register(register_name, data, RTC_CHIPSELECT, HIGH, true, true);
}
 
char read_rtc_register(char register_name) {
  return read_register(register_name, RTC_CHIPSELECT, HIGH, false, true);
}
 
// reads a register
char read_register(char register_name, byte cs_pin, byte cs_active_level, boolean read_high, boolean cpha_trailing)
{
  char in_byte;
  if(cpha_trailing) {
    SPCR = (1<<SPE)|(1<<MSTR)|(1<<CPHA)|(0<<SPR1)|(0<<SPR0);
  }
  else {
    SPCR = (1<<SPE)|(1<<MSTR)|(0<<CPHA)|(0<<SPR1)|(0<<SPR0);
  }
  clr = SPCR;
  clr = SPDR;
  if(read_high) {
    // need to set bit 7 to indicate a read for the slave device
    register_name |= 128;
  }
  else {
    // if read low, means A7 bit should be cleared when reading for the slave device
    register_name &= 127;
  }
  // SS is active low
  digitalWrite(cs_pin, cs_active_level);
  // send the address of the register we want to read first
  spi_transfer(register_name);
  // send nothing, but here's when the device sends back the register's value as an 8 bit byte
  in_byte = spi_transfer(0);
  // deselect the device..
  if(cs_active_level == HIGH) {
    digitalWrite(cs_pin, LOW);
  }
  else {
    digitalWrite(cs_pin, HIGH);
  }
  return in_byte;
}
 
// write to a register
// write_high if true indicates set A7 bit to 1 during a write
void write_register(char register_name, byte data, byte cs_pin, byte cs_active_level, boolean write_high, boolean cpha_trailing)
{
  if(cpha_trailing) {
    SPCR = (1<<SPE)|(1<<MSTR)|(1<<CPHA)|(0<<SPR1)|(0<<SPR0);
  }
  else {
    SPCR = (1<<SPE)|(1<<MSTR)|(0<<CPHA)|(0<<SPR1)|(0<<SPR0);
  }
  clr=SPCR;
  clr=SPDR;
  // char in_byte;
  if(write_high) {
    // set A7 bit to 1 during a write for this device
    register_name |= 128;
  }
  else {
    // clear bit 7 to indicate we're doing a write for this device
    register_name &= 127;
  }
  // SS is active low
  digitalWrite(cs_pin, cs_active_level);
  // send the address of the register we want to write
  spi_transfer(register_name);
  // send the data we're writing
  spi_transfer(data);
  if(cs_active_level == HIGH) {
    digitalWrite(cs_pin, LOW);
  }
  else {
    digitalWrite(cs_pin, HIGH);
  }
  //return in_byte;
}
 
void loop()
{
  byte in_byte;
  // keep track of what our seconds alarm register is..
  // we use this in the ISR to make sure we alarm every 15 seconds
  in_byte = read_rtc_register(0x07);
  Serial.print("sec alarm is ");
  Serial.print(in_byte, HEX);
 
  in_byte = read_rtc_register(0x00);
  Serial.print(" SECS=");
  Serial.print(in_byte, HEX);
 
  in_byte = read_rtc_register(0x01);
  Serial.print(" MINS=");
  Serial.print(in_byte, HEX);
 
  in_byte = read_rtc_register(0x02);
  Serial.print(" HRS=");
  Serial.println(in_byte, HEX);
  digitalWrite(LED, LOW);
  delay(500);
}
