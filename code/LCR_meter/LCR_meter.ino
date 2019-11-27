/*
  Name : LCR_meter
  Title : Arduino NANO(Version3) based (LCR)Inductance  Capacitance and resistence meter.
  Description : An LCR meter is a type of electronic test equipment used to measure
  the inductance (L), capacitance (C), and resistance (R) of an electronic component.
  Full details at URL.
  Author: Gavin Lyons
  Platform: Arduino NANO version 3.0
  URL: https://github.com/gavinlyonsrepo/LCR_meter
*/

//****************** LIBS ********************
//V.1.1.2 https://github.com/marcoschwartz/LiquidCrystal_I2C
#include <LiquidCrystal_I2C.h> // For the LCD 

//***************** GLOBALS ********************

// Push Button pins
#define buttonTest 11
#define buttonMode 12

// Variables to debounce the two push buttons
int buttonTestState;             // the current reading from the input pin
int lastButtonTestState = HIGH;   // the previous reading from the input pin
// the following variable are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTestTime = 0;  // the last time the output pin was toggled

int buttonModeState;             // the current reading from the input pin
int lastButtonModeState = HIGH;   // the previous reading from the input pin
unsigned long lastDebounceModeTime = 0;  // the last time the output pin was toggled

unsigned long debounceDelay = 50;    // the debounce time;

// Var to hold menu mode
uint8_t mode = 0;

// resistor test pin numbers
#define analogResPin  14 //analog pin read for resistor test
#define ApplyResVoltage  10  //digital pin to unknown resistor,  apply 5V
// Known Resistors connected to these 4 digital pins
#define Res2K  2
#define Res20K  3
#define Res200K  8
#define Res1M  9

// Inductance test pin numbers
#define OutLtestPin  5 //digital pin input to circuit to "ring" LC circuit
#define PulseCap2InPinLtest  4 //digital pin to read in pulse

// Pins and vars for C test 1
#define Cap1analogPin    20
#define Cap1chargePin    13
#define Cap1dischargePin 21 //A7
#define resistorValue  9991.0F // 10K in theory User adjust

// Pins and vars for C test 2
#define Cap2OutPin  17
#define Cap2InPin  16
const float CapOne = 24.51; //user calibrate
const float Res_Pullup = 34.9;
const int MaxADC_Value = 1023;

//Pins and vars for C test 3
#define Cap3pulsePin  15
const unsigned long resistance = 9938; // (10K in theory) user adjust
volatile boolean triggered;
volatile boolean active;
volatile unsigned long startTime;
volatile unsigned long duration;

//Interupt service rountine used by Cap test 3
ISR (ANALOG_COMP_vect)
{
  unsigned long now = micros ();
  if (active)
  {
    duration = now - startTime;
    triggered = true;
    digitalWrite (Cap3pulsePin, LOW);
  }
}

// LCD , initialize the library with the numbers of the interface pins
LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

//**************** FUNCTION PROTOTYPES ***********
void DisplayInit();
void Serialinit();
void GPIOinit();

void LCDReady();
void DisplayHelpMsg();
void printMenuMsg();
void DisplayTime(unsigned long elaspedTime);

void ResScaleOne();
void ResScaleTwo();
void ResScaleThree();
void ResScaleFour();
void Ltest();
void CAPTestOne();
void CAPTestTwo();
void CAPTestThree();

float calcResult(float R1, int multi_factor);
void PrintResult(String unit, float R2);
void TestRun();

void ReadPushButtonMode();
void ReadPushButtonTest();

//*************** SETUP *************
void setup()
{
  Serialinit();
  GPIOinit();
  DisplayInit();
}

//******************* MAIN LOOP *****************
void loop()
{
  ReadPushButtonMode();
  ReadPushButtonTest();
}

// ********************* FUNCTIONS *************************

// Function to display help message related to PCB socket layout
// called when test button is pressed during Ready init message display
void DisplayHelpMsg()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("HELP");
  lcd.setCursor(0, 1);
  lcd.print("Connector pinout");
  delay(1000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("R1NCGDGDNCNCR2R2");
  lcd.setCursor(0, 1);
  lcd.print("LLC2NCC2GDGDC3C1");
  delay(5000);
  LCDReady();
}

//Function to init LCD and display LCD  welcome message
void DisplayInit()
{
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("LCR Meter V 1.0");
  lcd.setCursor(0, 1);
  lcd.print("By Gavin Lyons");
  delay(1000);
  LCDReady();
}

//Function to display Ready message on LCD and serial monitor
void LCDReady()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Ready: BTN1 MODE");
  lcd.setCursor(0, 1);
  lcd.print("       BTN2 TEST");
  Serial.println("LCR Meter Ready");
  mode = 0;
  delay(50);
}

//Function to setup serial called from setup
void Serialinit()
{
  Serial.begin(9600);
  delay(100);
  Serial.println("-------------LCR Meter Comms UP------------");
}

//Function to init GPIO pins on ATmega328p setup called from setup
void GPIOinit()
{
  // Setup pins for button enable internal pull-up resistors
  digitalWrite(buttonMode, HIGH);
  digitalWrite(buttonTest, HIGH);

  // Set resistor measurement pins
  pinMode(analogResPin, INPUT);
  pinMode(ApplyResVoltage, OUTPUT);
  // Set known resistor pins as inputs
  pinMode(Res2K, INPUT);
  pinMode(Res20K, INPUT);
  pinMode(Res200K, INPUT);
  pinMode(Res1M, INPUT);

  //Inductance test pins
  pinMode(PulseCap2InPinLtest, INPUT);
  pinMode(OutLtestPin, OUTPUT);

  // Setup pins for C test 1
  pinMode(Cap1chargePin, OUTPUT);
  digitalWrite(Cap1chargePin, LOW);

  // Setup pins for C test 2
  pinMode(Cap2OutPin, OUTPUT);
  pinMode(Cap2InPin, OUTPUT);

  // setup for C test3
  pinMode(Cap3pulsePin, OUTPUT);
  digitalWrite(Cap3pulsePin, LOW);

  // Set up Analog Comparator used by C test 3
  ADCSRB = 0; // clear ADCSRB registers
  // Analog Comparator Interrupt Flag: Clear Pending Interrupt
  // Analog Comparator Interrupt: Enabled
  // Analog Comparator Interrupt Mode: interrupt on the rising edge
  ACSR =  _BV (ACI)
          | _BV (ACIE)
          | _BV (ACIS0) | _BV (ACIS1);
}

//Function ResScaleOne :resistor 0 to 2k range test.
void ResScaleOne()
{
  digitalWrite(ApplyResVoltage, HIGH);
  pinMode(Res2K, OUTPUT);
  pinMode(Res20K, INPUT);
  pinMode(Res200K, INPUT);
  pinMode(Res1M, INPUT);
  digitalWrite(Res2K, LOW);
  float R2 = 0;
  float R1 = 2.005; // Set this value to the value of the used resistor in K ohms
  R2 = calcResult(R1, 1000);
  if (R2 > (R1 * 1000))
  {
    Serial.println("Increasing Scale");
    mode = 2; //increase scale
    printMenuMsg();
    //ResetResRange();
    ResScaleTwo();
  }
  if (R2 < (R1 * 1000))
  {
    PrintResult("ohms", R2);
  }
}

//Function ResScaleTwo: resistor 2K to 20k range test.
void ResScaleTwo()
{
  digitalWrite(ApplyResVoltage, HIGH);
  pinMode(Res2K, INPUT);
  pinMode(Res20K, OUTPUT);
  pinMode(Res200K, INPUT);
  pinMode(Res1M, INPUT);
  digitalWrite(Res20K, LOW);
  float R2 = 0;
  float R1 = 18.3; // Set this value to the value of the used resistor in K ohms
  R2 = calcResult(20.03, 1);
  if (R2 > R1)
  {
    Serial.println("Increasing Scale");
    mode = 3; //increase scale
    printMenuMsg();
    //ResetResRange();
    ResScaleThree();
  }
  if (R2 < R1)
  {
    PrintResult("k ohms", R2);
  }
}

//Function ResScaleThree : resistor test 20k to 200k range test.
void ResScaleThree()
{
  digitalWrite(ApplyResVoltage, HIGH);
  pinMode(Res2K, INPUT);
  pinMode(Res20K, INPUT);
  pinMode(Res200K, OUTPUT);
  pinMode(Res1M, INPUT);
  digitalWrite(Res200K, LOW);
  float R2 = 0;
  float R1 = 218; // Set this value to the value of the used resistor in K ohms
  R2 = calcResult(R1, 1);
  if (R2 > R1)
  {
    Serial.println("Increasing Scale");
    mode = 4; //increase scale
    printMenuMsg();
    //ResetResRange();
    ResScaleFour();
  }
  if (R2 < R1)
  {
    PrintResult("k ohms",  R2);
  }
}
//Function ResScaleFour : resistence test 200k to 1M range test.
void ResScaleFour()
{
  digitalWrite(ApplyResVoltage, HIGH);
  pinMode(Res2K, INPUT);
  pinMode(Res20K, INPUT);
  pinMode(Res200K, INPUT);
  pinMode(Res1M, OUTPUT);
  digitalWrite(Res1M, LOW);
  float R2 = 0;
  float R1 = 1006;// Set first value to the value of the used resistor in Kohms
  R2 = calcResult(R1, 1);
  if (R2 > 2000)
  {
    mode = 10; //Beyond Scale Too high
    printMenuMsg();
  }
  if (R2 < 2000)
  {
    if (R2 <= 10)
    {
      Serial.println("Decreasing Scale");
      mode = 1; //decrease scale
      printMenuMsg();
      ResScaleOne();
      return;
    }
    PrintResult("M ohms", (R2 / 1000));
  }
}

// Function: calcResult to calculate unknown resistor value
// Inputs: (2) 1, float R1 known resistor value for given scale
// 2, Integer multiple factor either 1 or 1000 depending on given scale.
// Outputs: returns 1, float R2 unknown resitor value
float calcResult(float R1, int multi_factor)
{
  float R2 = 0;
  float tmpbuffer = 0;
  int V_measured = 0;
  uint8_t Vin = 5;
  float Vout = 0;
  const uint8_t numReadings = 11; // number of analog samples
  int readings[numReadings];      // the readings from the analog input

  // Get 11(numreadings) values from ADC
  for (int thisReading = 0; thisReading < numReadings; thisReading++)
  {
    readings[thisReading] = analogRead(analogResPin); // ADC
    if (thisReading != 0) // ignore first reading as it is bad during auto-range.
    {
      V_measured  = V_measured + readings[thisReading]; //running total
    }
  }
  V_measured = (V_measured / (numReadings - 1)); // average

  Vout = (V_measured * Vin) / 1024.0; //Convert ADC to voltage
  tmpbuffer = (Vin / Vout) - 1; //voltage divider (VIN/VOUT -1)
  R2 = R1 * tmpbuffer * multi_factor;  // R2 = R1(Vin/Vout -1)
  return R2;
}

// Function to print various messages input based on Menu push button press
void printMenuMsg()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  switch (mode)
  {
    case (0): // Ready message
      __asm__("nop\n\t");
      break;
    case (1):
      lcd.print("R 0k-2k range");
      Serial.println("0k to 2k range");
      break;
    case (2):
      lcd.print("R 2k-20k range");
      Serial.println("2k to 20k range");
      break;
    case (3):
      lcd.print("R 20k-200k range");
      Serial.println("20k to 200k range");
      break;
    case (4):
      lcd.print("R 0.2M-1M range");
      Serial.println("200k to 1M range");
      break;
    case (5):
      lcd.print("Inductance test");
      Serial.println("Inductance test");
      break;
    case (6):
      lcd.print("Capacitance T1");
      lcd.setCursor(0, 1);
      lcd.print("1uF to 4F");
      Serial.println("Capacitance Test One");
      Serial.println("Range 1uF to 4F");
      break;
    case (7):
      lcd.print("Capacitance T2");
      lcd.setCursor(0, 1);
      lcd.print("18pF to 470uF");
      Serial.println("Capacitance Test Two");
      Serial.println("Range 18pF to 470uF");
      break;
    case (8):
      lcd.print("Capacitance T3");
      lcd.setCursor(0, 1);
      lcd.print("4.7 nF to 180 uF");
      Serial.println("Capacitance Test Three");
      Serial.println("4.7 nF to 180 uF");
      break;
    case (9):
      LCDReady();
      break;
    case (10):
      lcd.print("Out of Scale");
      Serial.println("Out of scale");
      delay(1500);
      LCDReady();
      break;
  }
}

//Function to run tests when test button pressed also
//Displays help message if in mode 0
void TestRun()
{
  switch (mode)
  {
    case (0):
      DisplayHelpMsg();
      break;
    case (1):
      ResScaleOne();
      break;
    case (2):
      ResScaleTwo();
      break;
    case (3):
      ResScaleThree();
      break;
    case (4):
      ResScaleFour();
      break;
    case (5):
      Ltest();
      break;
    case (6):
      CAPTestOne();
      break;
    case (7):
      CAPTestTwo();
      break;
    case (8):
      CAPTestThree();
      break;
  }
}

//Function PrintResult : Print calculated resistor value and unit to serial monitor
//Inputs (2) : 1 , String unit unit of resistance Kohms Mohms ,
// 2, float R2 , value of resistance calculated
void PrintResult(String unit, float R2)
{
  Serial.println("Resistance: ");
  Serial.print(R2);
  Serial.print(" ");
  Serial.println(unit);
  lcd.setCursor(0, 1);
  lcd.print(R2);
  lcd.print(unit);
  delay(2500);
  printMenuMsg();
}

//Function Ltest: Calculates Inductance
void Ltest()
{
  lcd.clear();
  double pulse, frequency, capacitance, inductance = 0;
  digitalWrite(OutLtestPin, HIGH);
  delay(5);//give some time to charge inductor.
  digitalWrite(OutLtestPin, LOW);
  delayMicroseconds(100); //make sure resonation is measured
  pulse = pulseIn(PulseCap2InPinLtest, HIGH, 5000); //returns 0 if timeout
  if (pulse > 0.1) { //pulse returns 0 if no complete pulse was received within the timeout

    capacitance = 2.E-6; // - insert Cap value here

    frequency = 1.E6 / (2 * pulse);
    inductance = 1. / (capacitance * frequency * frequency * 4.*3.14159 * 3.14159);
    inductance *= 1E6;
  }

  //Serial print
  Serial.print("High for uS:");
  Serial.println( pulse );
  Serial.print("frequency Hz:");
  Serial.println( frequency );
  Serial.print("inductance uH:");
  Serial.println( inductance );

  lcd.setCursor(0, 0);
  lcd.print(inductance);
  lcd.print("uH ");
  lcd.setCursor(0, 1);
  lcd.print(frequency);
  lcd.print("Hz ");
  lcd.print(pulse);
  lcd.print("uS ");

  delay(4000);
  printMenuMsg();
}

//Function to handle  C test1
void CAPTestOne()
{
  // carry out Test
  unsigned long startTime;
  unsigned long elapsedTime;
  float uF;
  float nF;
  digitalWrite(Cap1chargePin, HIGH);
  startTime = millis();
  // 648 is 63.2% of 1024 adc 10 bit = 1024 stop when get to 648
  while (analogRead(Cap1analogPin) < 648) {
  }
  elapsedTime = millis() - startTime;

  // Calculate and display value, c = TC/R
  uF = ((float)elapsedTime / resistorValue) * 1000;
  DisplayTime(elapsedTime);
  lcd.setCursor(0, 1);
  if (uF > 1) {
    Serial.print((long)uF);
    Serial.println(" microFarads");
    lcd.print((long)uF);
    lcd.print(" uF");
  }
  else {
    nF = uF * 1000.0;
    Serial.print((long)nF);
    Serial.println(" nanoFarads");
    lcd.print((long)nF);
    lcd.print(" nF");
    delay(500);
  }

  //discharge
  digitalWrite(Cap1chargePin, LOW);
  pinMode(Cap1dischargePin, OUTPUT);
  digitalWrite(Cap1dischargePin, LOW);
  while (analogRead(Cap1analogPin) > 0) {
  }
  pinMode(Cap1dischargePin, INPUT);
  delay(4000);
  printMenuMsg();
}

//Function to carry out Test2
void CAPTestTwo()
{
  lcd.clear();
  lcd.setCursor(0, 0);

  pinMode(Cap2InPin, INPUT);
  digitalWrite(Cap2OutPin, HIGH);
  int val = analogRead(Cap2InPin);
  digitalWrite(Cap2OutPin, LOW);

  if (val < 1000)
  {
    pinMode(Cap2InPin, OUTPUT);
    // Cu = VA2 * C1 / (VA3 - VA2)
    float capacitance = (float)val * CapOne / (float)(MaxADC_Value - val);
    Serial.print(F("Capacitance Value = "));
    Serial.print(capacitance, 3);
    Serial.print(F(" pF ADC = ("));
    Serial.print(val);
    Serial.println(F(") "));
    lcd.print(capacitance, 3);
    lcd.print(" pF");
    lcd.setCursor(0, 1);
    lcd.print("ADC=");
    lcd.print(val);
  }
  else
  {
    pinMode(Cap2InPin, OUTPUT);
    delay(1);
    pinMode(Cap2OutPin, INPUT_PULLUP);
    unsigned long u1 = micros();
    unsigned long t;
    int digVal;
    do
    {
      digVal = digitalRead(Cap2OutPin);
      unsigned long u2 = micros();
      // condition ? result_if_true : result_if_false
      t = u2 > u1 ? u2 - u1 : u1 - u2;
    } while ((digVal < 1) && (t < 400000L));

    pinMode(Cap2OutPin, INPUT);
    val = analogRead(Cap2OutPin);

    //discharge
    digitalWrite(Cap2InPin, HIGH);
    int dischargeTime = (int)(t / 1000L) * 5;
    delay(dischargeTime);
    pinMode(Cap2OutPin, OUTPUT);
    digitalWrite(Cap2OutPin, LOW);
    digitalWrite(Cap2InPin, LOW);

    float capacitance = -(float)t / Res_Pullup
                        / log(1.0 - (float)val / (float)MaxADC_Value);
    Serial.print(F("Capacitance Value = "));
    if (capacitance > 1000.0)
    {
      Serial.print(capacitance / 1000.0, 2);
      Serial.print(F(" uF"));
      lcd.print(capacitance / 1000.0, 2);
      lcd.print(" uF");
    }
    else
    {
      Serial.print(capacitance, 2);
      Serial.print(F(" nF"));
      lcd.print(capacitance, 2);
      lcd.print(" nF");
    }

    Serial.print(F(" ("));
    // condition ? result_if_true : result_if_false
    Serial.print(digVal == 1 ? F("Normal") : F("HighVal"));
    Serial.print(F(", t= "));
    Serial.print(t);
    Serial.print(F(" us, ADC= "));
    Serial.print(val);
    Serial.println(F(")"));
    lcd.print(" A=");
    lcd.print(val);
    lcd.setCursor(0, 1);
    lcd.print("t=");
    lcd.print(t);
    lcd.print("uS");
  }
  while (millis() % 1000 != 0);
  delay(4000);
  printMenuMsg();
}

//Function to carry out Test3
void CAPTestThree(void)

{
  boolean exitloop = false;
  while (exitloop != true)
  {
    if (!active)
    {
      active = true;
      triggered = false;
      digitalWrite (Cap3pulsePin, HIGH);
      startTime = micros ();
    }

    if (active && triggered)
    {
      active = false;
      Serial.print ("Capacitance = ");
      Serial.print (duration * 1000 / resistance);
      Serial.println (" nF");
      Serial.print ("duration = ");
      Serial.print(duration);
      Serial.println (" uS");
      triggered = false;
      //LCD
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print((duration * 1000 / resistance));
      lcd.print(" nF");
      lcd.setCursor(0, 1);
      lcd.print("t = ");
      lcd.print(duration);
      lcd.print(" uS");

      delay (4000);
      printMenuMsg();
      exitloop = true; //exit when test finished.
    }
  }
}

//Function to Display elasped time to LCD and Serial Monitor called from CAPTestOne function
void DisplayTime(unsigned long elaspedTime)
{
  Serial.print(elaspedTime);
  Serial.print(" mS    ");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(elaspedTime);
  lcd.print(" mS");
}

// Function to handle debounce of mode menu button press.
// If debounced succesfully increment mode variable and change menu mode display
void ReadPushButtonMode()
{
  // read and debounce push button.
  int reading = digitalRead(buttonMode);
  // If the switch changed?
  if (reading != lastButtonModeState) {
    // reset the debouncing timer
    lastDebounceModeTime = millis();
  }
  if ((millis() - lastDebounceModeTime) > debounceDelay) {
    // if the button state has changed:
    if (reading != buttonModeState) {
      buttonModeState = reading;
      if (buttonModeState == LOW) {
        mode++;
        printMenuMsg();
      }
    }
  }

  // save the reading.
  lastButtonModeState = reading;
}

// Function to handle debounce of start test button
// If debounced and succesful read, start the test by calling Testrun();
void ReadPushButtonTest()
{
  // read and debounce push button.
  int reading = digitalRead(buttonTest);
  // If the switch changed?
  if (reading != lastButtonTestState) {
    // reset the debouncing timer
    lastDebounceTestTime = millis();
  }
  if ((millis() - lastDebounceTestTime) > debounceDelay) {
    // if the button state has changed:
    if (reading != buttonTestState) {
      buttonTestState = reading;
      // start test if the new button state is low
      if (buttonTestState == LOW) {
        TestRun();
      }
    }
  }

  // save the reading.
  lastButtonTestState = reading;
}

//******************* EOF *****************
