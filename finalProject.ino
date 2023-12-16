/*
Names: Farzana Tanni & Lorie Phan
Date: 12/15/23
Assignment: Final Project - Swamp Cooler
*/

//libaries
#include <LiquidCrystal.h>
#include <Stepper.h>
#include <DHT.h>
#include <RTClib.h>

DHT dht(6, DHT11);

const int RS = 11, EN = 12, D4 = 2, D5 = 3, D6 = 4, D7 = 5;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

#define RDA 0x80
#define TBE 0x20 

//UART
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

//ADC
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

//macros
#define WRITE_LOW(pin_num) *port_a &= ~(0x01 << pin_num);
#define WRITE_HIGH(pin_num) *port_a |= (0x01 << pin_num);
#define THRESHOLD 100
#define STEPPER 2038
#define BUTTON 4
#define SET_BIT(reg, bit) (reg |= (1 << bit))
#define CLEAR_BIT(reg, bit) (reg &= ~(1 << bit))

//define Port A register pointers
volatile unsigned char* port_a= (unsigned char*) 0x22;
volatile unsigned char* ddr_a= (unsigned char*) 0x21;
volatile unsigned char* pin_a= (unsigned char*) 0x20;

//define Port B register pointers
volatile unsigned char* port_b = (unsigned char*) 0x25; 
volatile unsigned char* ddr_b  = (unsigned char*) 0x24;
volatile unsigned char* pin_b  = (unsigned char*) 0x23; 

//define Port D register pointers
volatile unsigned char* port_d = (unsigned char*) 0x2B; 
volatile unsigned char* ddr_d  = (unsigned char*) 0x2A; 
volatile unsigned char* pin_d  = (unsigned char*) 0x29;

//define Port K register pointers
volatile unsigned char* port_k = (unsigned char*) 0x108; 
volatile unsigned char* ddr_k  = (unsigned char*) 0x107; 
volatile unsigned char* pin_k  = (unsigned char*) 0x106; 

volatile unsigned char* my_PCICR = (unsigned char*) 0x68;
volatile unsigned char* my_PCMSK0 = (unsigned char*) 0x6B;
volatile unsigned char* my_PCIFR = (unsigned char*) 0x3B;
volatile unsigned char* my_EIMSK = (unsigned char*) 0x3D;
volatile unsigned char* my_EICRA = (unsigned char*) 0x69;
volatile unsigned char* my_EICRB = (unsigned char*) 0x6A;
volatile unsigned char* my_EIFR = (unsigned char*) 0x3C;

//set up clock
RTC_DS1307 clock;

//variables
char currentState;
float conductivity; 
float waterLevel; 

const int stepsPerRev = 2048;
const int rotPerMin = 10;
const int steps = stepsPerRev / 360;
int buttonState = HIGH;
int prev = 0;
Stepper stepper(stepsPerRev, 8, 9, 10, 11);

//---SETUP FUNCTION
void setup() {
  U0init(9600);
  adc_init();
  stepper.setSpeed(15);

  *ddr_a &= 0b00001111;
  *port_a |= 0b11110000;
  *ddr_b |= 0b10000000;
  *port_b &= 0b01111111;
  *ddr_k &= 0b01111111;

  lcd.begin(16,2);
  dht.begin();
  currentState = 'i';
  ISR_config();
}

//---LOOP FUNCTION
void loop() {
  monitorAirHumidity();
  monitorWaterLevel();
}

//---OTHER FUNCTIONS FROM PREVIOUS LABS
void U0init(unsigned long U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}

unsigned char U0kbhit(){
  return (RDA & *myUCSR0A);
}

unsigned char U0getchar()
{
  return *myUDR0;
}

void U0putchar(unsigned char U0pdata)
{
  while(!(*myUCSR0A & TBE)){};
  *myUDR0 = U0pdata;
}

void adc_init() 
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit 7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}

unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

//--OTHER FUNCTIONS

//monitor water levels and print an alert when the level is too low
void monitorWaterLevel()
{
  int waterPin = A15;
  float conductivity = adc_read(pin_k);
  waterLevel = (conductivity - 0.5) * 100;

  //define the threshhold
  int threshhold = 275;

  //check if water is below threshhold
  if (waterLevel < threshhold) {
    //display on serial monitor
    Serial.println("Alert! Water level is low.");   
  } else if(waterLevel > threshhold) {
    Serial.println("Alert! Water level is high.");
  }
}

void monitorAirHumidity(){
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  lcd.clear();

  //temperature
  lcd.setCursor(0, 0);
  lcd.print("Temperature: ");
  lcd.print(temperature);
  lcd.write(223); // Degree symbol
  lcd.print("C");

  //humidity
  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.print(humidity);
  lcd.print("%");

  controlFanMotor(temperature);
}

void setFanHigh() {
    SET_BIT(*port_b, 2); 
}

void setFanLow() {
    CLEAR_BIT(*port_b, 2);
}

//start and stop fan motor when temperature falls out of specific range
void controlFanMotor(float temperature) {
  if (temperature > 25) {
    setFanHigh();
    recordTimeDate("Fan motor started!");
  } else {
    setFanLow();
    recordTimeDate("Fan motor stopped!");
  }
}

//record time and date every time motor is turned on and off
void recordTimeDate(String inputString) {
    DateTime currentTime = clock.now();
    
    inputString += "\n";

    printStringU0(inputString);

    printNumberU0(currentTime.year());
    U0putchar('-');
    printNumberU0(currentTime.month());
    U0putchar('-');
    printNumberU0(currentTime.day());
    U0putchar(' ');
    printNumberU0(currentTime.hour());
    U0putchar(':');
    printNumberU0(currentTime.minute());
    U0putchar(':');
    printNumberU0(currentTime.second());
}

void printStringU0(String str) {
    for (int i = 0; i < str.length(); i++) {
        U0putchar(str[i]);
    }
}

void printNumberU0(int num) {
    String sPrint = String(num);
    for (int i = 0; i < sPrint.length(); i++) {
        U0putchar(sPrint[i]);
    }
}

void ISR_config() {
  SET_BIT(*my_PCICR, 0);
  SET_BIT(*my_PCMSK0, BUTTON);
  SET_BIT(*my_EIMSK, 0);
  SET_BIT(*my_EICRA, 1);
}

ISR(PCINT0_vect) {
  int currentState = *pin_a & 0x0F;
  if (currentState == 0) {
    recordTimeDate("Button Pressed!");
    switchState();
  }
}

void switchState() {
  String state = "";
  if (currentState == 'i') {
    state = "idle";
  } else if (currentState == 'd') {
    state = "disabled";
  } else if (currentState == 'e') {
    state = "error";
  } else if (currentState == 'r') {
    state = "running";
  }
}