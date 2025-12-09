#include <ADS1X15.h>
#include <Wire.h>
#include <Servo.h>
#include <AceSorting.h>
//#include <LiquidCrystal_I2C.h>
using ace_sorting::shellSortKnuth;
Servo myservo;

//LiquidCrystal_I2C lcd(0x27, 16, 2);
ADS1115 ADS(0x48);

//---Output Control Line Pin Assignments-----
const int PICSignalsEN = 2;    //  Used to connect PIC Programmer signals to DUT (3 lines)
const int MAXDAPSignalsEN = 3; //  Used to connect MAXDAP (LoRa) Programmer signala to DUT (3 lines)
const int PicMCLRPGM = 4;      //  Used to connect DUT MCLR pin to PIC Programmer
const int PicMCLRGND = 5;      //  Used to connect DUT MCLR pin to Ground
const int TestSerialEN = 40;   //  Used to disconnect serial test signals when PIC Programmer signals are connected, and vica versa
const int TempResSelect1 = 22; //  Used to enable Res.1 for Temp Sensor Test (switches 220K in and out)
const int TempResSelect2 = 24; //  Used to enable Res.2 for Temp Sensor Test (spare)
const int VRegSelect2 = 26;    //  Used to set DUT power at 2.6V
const int VRegSelect1 = 28;    //  Used to set DUT power at 3.3V
const int HighResEN = 30;      //  Used to switch between current sense resistor of 100Ohms and 1Ohm (High = 100Ohms)
const int CalLoadEN = 32;      //  Used to connect calibration load. Also used to disconnect power in case of a S.C. (High = Cal.Load connected/DUT disconnected)
const int Solenoid1EN = 34;    //  Used to Enable Solenoid 1 for magnetic Hall Sensor test 
const int Solenoid2EN = 36;    //  Used to enable Solenoid 2 to push RESET button on DUT
const int DetSwitch = 38;      //  Used as input to detect presence of the DUT on the test fixture
const int ServoPin = 8;
//---Digital Input Line Pin Assignments-----

//---Analog Input Line Pin Assignments-----
const int DUTVoltage = A1;    //  Analog Measurement of DUT Supply Voltage
const int DUTCurrent = A0;    //  Analog Measurement of DUT Supply Current

//---Miscellaneous Constants-----
const float VoltCalRef = 3.33;   // Measured Supply Voltage Reference for Voltage Correction, if necessary
const float CurrentCalRef = 5;   // Measured Current Reference for Current Measurement Correction
const float ADCReference = 5;    // ADC Voltage Reference
const int ADCRes = 1024;         // Resolution of 10bit ADC (Arduino default)
const float OpAvg = 55;          // Number of Current Measurement Averages for Regular Operation
const float CalAvg = 500;        // Number of Current Measurement Averages for Calibration Mode
const float HiCurrFactor = 910;  // High Current (>0.1mA) Coefficient (Fixed/Non-calibrated)

//---Miscellaneous Variables-----
float VoltageReading;   // ADC Voltage Reading (using Arduino ADC)
float CurrentReading;   // ADC Current Reading (using ADS1115 16bit ADC)
float CurrentGainFactor = 1;      // Current Correction Gain Factor
float CurrentOffsetFactor = 0;    // Current Correction Offset Factor

float ADCVoltageFactor = 1;  
float STDVoltageFactor = 1;

int AvgNumMax;          //  Variable for number of Filter Averaging cycles
int Parameter;          //  Variable for solenoid time duration
boolean HiResState;     //  Flag to signify presence of 1 or 560 Ohm C.S. Resistor
boolean DUTPresent;     //  Flag to signify presence of DUT
String InputText;
char Command;

//----------------------------------------------

void setup() {
  ADS.begin();
  Serial.begin(115200);
  Wire.begin();
//  lcd.init();
//  lcd.backlight();
  ADS.setGain(1);
  ADS.setDataRate(5); 
  ADS.setMode(1);
  pinMode(PICSignalsEN, OUTPUT);
  pinMode(MAXDAPSignalsEN, OUTPUT);
  pinMode(PicMCLRPGM, OUTPUT);
  pinMode(PicMCLRGND, OUTPUT);
  pinMode(TestSerialEN, OUTPUT);
  pinMode(TempResSelect1, OUTPUT);
  pinMode(TempResSelect2, OUTPUT);
  pinMode(VRegSelect2, OUTPUT);
  pinMode(VRegSelect1, OUTPUT);
  pinMode(HighResEN, OUTPUT);
  pinMode(CalLoadEN, OUTPUT);
  pinMode(Solenoid1EN, OUTPUT);
  pinMode(Solenoid2EN, OUTPUT);
  myservo.attach(ServoPin);
  myservo.write(105);
  //-------------------------------------
  pinMode(DetSwitch, INPUT_PULLUP);
  pinMode(DUTVoltage, INPUT);
  pinMode(DUTCurrent, INPUT);
  //---Initial Conditions----------------      
  digitalWrite(Solenoid1EN, LOW);   // Disable Solenoid 1
  digitalWrite(Solenoid2EN, LOW);   // Disable Solenoid 2
  digitalWrite(VRegSelect1, HIGH);  // Set DUT power supply to 3.3V
  digitalWrite(VRegSelect2, LOW);   // Set Alternate P.S. Voltage Off
  digitalWrite(HighResEN, HIGH);  HiResState = 1;   // Set current measurement resistor to 560 Ohm (for potential short circuit detection)
  digitalWrite(CalLoadEN, HIGH);    // Connect power source to Calibration Load
  digitalWrite(TempResSelect1, HIGH);
  digitalWrite(TempResSelect2, HIGH);
  digitalWrite(PICSignalsEN, HIGH);
  digitalWrite(TestSerialEN, LOW);
  digitalWrite(MAXDAPSignalsEN, HIGH);
  digitalWrite(PicMCLRPGM, HIGH);
  digitalWrite(PicMCLRGND, HIGH);
  AvgNumMax = OpAvg;   //  Set Default number of averaging cycles
  Serial.println("Radio Bridge PCB Tester Development Prototype");
  //Calibration();   // Start-up Calibration
  //VoltCalibration();
//  lcd.clear();
}

//-------------------------------------------

void loop() {
  //DUTPresent = digitalRead(DetSwitch);
  //if (DUTPresent == 0) {
     //Serial.println("DUT Detected!");
  //}
 // ADCVoltageRead();
  //ADCReadAvgCurrent();
//  LCDisplay();
  if (Serial.available() > 0) {
    InputText = Serial.readStringUntil('\n');     //  Read input text through serial port
    Command = InputText.charAt(0);
    //Serial.println(Command);
    switch (Command) {
      case 'a':                                             //  a ---> Sets the DUT power supply output voltage to 2.6V (default = 3.3V)
        digitalWrite(VRegSelect2, HIGH);
        Serial.println("OK a");
        break;
      case 'b':                                             //  b ---> Sets the DUT power supply output voltage back to 3.3V)
        digitalWrite(VRegSelect2, LOW);
        Serial.println("OK b");
        break;
     //---------------------------
      case 'c':                                             //  c ---> Connects the 660K calibration load to the power supply for calibration to precision current of 5uA (DUT disconnected)
        digitalWrite(CalLoadEN, HIGH);
        Serial.println("OK c");        
        break;
      case 'd':                                             //  d ---> Disconnects the 660K calibration load from the power supply (DUT is connected to power supply)
        digitalWrite(CalLoadEN, LOW);
        Serial.println("OK d");
        break;
     //---------------------------
      case 'e':                                             //  e ---> Sets the 560 Ohm resistor for use in current measurement (for use with low currents - <<100uA )
        digitalWrite(HighResEN, HIGH);  HiResState = 1;
        Serial.println("OK e");        
        break;
      case 'f':                                             //  f ---> Sets the 1 Ohm resistor for use in current measurement (for use with higher currents - > 100uA )
        digitalWrite(HighResEN, LOW);  HiResState = 0;
        Serial.println("OK f");        
        break;
     //---------------------------
      case 'g':                                             //  g ---> Provides energy to solenoid 1 for a specified duration for use in Hall sensor testing or RESET pushbutton actuation
        Serial.println("Sol.1 Enter ms");
        
        while (Serial.available() == 0) {
        }  
        Parameter = Serial.parseInt();
    
        if (Parameter > 0) {
      //    Serial.print("Duration for Solenoid 1: "); Serial.println(Parameter, DEC);
      //    Serial.println("Sol.1 ON...");
          digitalWrite(Solenoid1EN, HIGH);
          delay(Parameter);
      //    Serial.println("Sol.1 OFF...");
          digitalWrite(Solenoid1EN, LOW);
        }
        Serial.println("OK g");
        break;

      case 'h':                                             //  h ---> Provides energy to solenoid 2 for a specified duration for use in Hall sensor testing or RESET pushbutton actuation
        Serial.println("Sol.2 Enter ms");
        
        while (Serial.available() == 0) {
        }  
        Parameter = Serial.parseInt();
    
        if (Parameter > 0) {
       //   Serial.print("Duration for Solenoid 2: "); Serial.println(Parameter, DEC);
       //   Serial.println("Sol.2 ON...");
          digitalWrite(Solenoid2EN, HIGH);
          delay(Parameter);
       //   Serial.println("Sol.2 OFF...");
          digitalWrite(Solenoid2EN, LOW);
        }
        Serial.println("OK h");
        break;

     //---------------------------
      case 'i':                                             //  i ---> Connects 220K Ohm resistor for use in Temperature sensor testing
        digitalWrite(TempResSelect1, LOW);
        Serial.println("OK i");
        break;
      case 'j':                                             //  j ---> Disconnects 220K Ohm resistor for Temperature sensor testing (i.e. open circuit/infinite resistance)
        digitalWrite(TempResSelect1, HIGH);
        Serial.println("OK j");
        break;   
     //---------------------------
      case 'z':                                             //  k ---> Prints the measured voltage on the terminal screen 
        ADCVoltageRead();
        Serial.print("Voltage = "); Serial.println(VoltageReading);
        break;
      case 'k':                                             //  k ---> Prints the measured voltage on the terminal screen 
        VoltageRead();
        Serial.print("Voltage = "); Serial.println(VoltageReading);
        break;
      case 'l':                                             //  l ---> Prints the measured current on the terminal screen (mA or uA depends on present C.S. resistor setting)
        ADCReadAvgCurrent();
        Serial.print("Current = "); Serial.println(CurrentReading);
        break;
     //---------------------------        
      case 'm':                                             //  m ---> Connects PIC prog. signals to the DUT -> this includes '3.3VBatt', 'ICSPDAT', & 'ICSPCLK'
        digitalWrite(PICSignalsEN, LOW);
        Serial.println("OK m");
        break;
      case 'n':                                             //  n ---> Disconnects PIC prog. signals from the DUT 
        digitalWrite(PICSignalsEN, HIGH);
        Serial.println("OK n");
        break;
     //---------------------------        
      case 'o':                                             //  o ---> Connects serial test communication signals to the DUT
        digitalWrite(TestSerialEN, LOW);
        Serial.println("OK o");
        break;
      case 'p':                                             //  p ---> Disconnects serial test communication signals from the DUT
        digitalWrite(TestSerialEN, HIGH);
        Serial.println("OK p");
        break;
     //---------------------------
      case 'q':                                             //  q ---> Connects 'MCLRn' signal between the PIC Programmer and DUT
        digitalWrite(PicMCLRPGM, LOW);
        digitalWrite(PicMCLRGND, HIGH);
        Serial.println("OK q");
        break;
      case 'r':                                             //  r ---> Disconnects 'MCLRn' signal between the PIC Programmer and DUT (Grounds DUT input)
        digitalWrite(PicMCLRPGM, HIGH);
        digitalWrite(PicMCLRGND, LOW);
        Serial.println("OK r");
        break;
      case 's':                                             //  s ---> Disconnects 'MCLRn' signal between the PIC Programmer and DUT (Floats DUT input)
        digitalWrite(PicMCLRPGM, HIGH);
        digitalWrite(PicMCLRGND, HIGH);
        Serial.println("OK s");
        break;
    //----------------------------   
      case 't':                                             //  t ---> Connects MAXDAP signals (LoRa programmer) between the tester and DUT - this includes 'RSTn', 'SWDIO', & 'SWDCLK'
        digitalWrite(MAXDAPSignalsEN, LOW);
        Serial.println("OK t");
        break;
      case 'u':                                             //  u ---> Disconnects MAXDAP signals (LoRa programmer) between the tester and DUT
        digitalWrite(MAXDAPSignalsEN, HIGH);
        Serial.println("OK u");        
        break;
    //---------------------------    
      case 'v':                                             //  v ---> Queries whether or or not DUT is present based on current drain
        digitalWrite(CalLoadEN, LOW);
        digitalWrite(HighResEN, LOW);  HiResState = 0;
        delay(10);
    //    ADCCurrentRead();
        ADCReadAvgCurrent();
        if (CurrentReading >= 0.5) {
          Serial.println("DUT y");      
        }
        else {
          Serial.println("DUT n");      
        }
        break;
    //---------------------------
        case 'w':                                             //   w ---> Measures Sleep current in one command cycle after disconnecting mutiple signals
        digitalWrite(CalLoadEN, LOW);                         //   Ensure Cal.Load is not connected
        digitalWrite(PICSignalsEN, HIGH);                     //   Disconnects PIC signals   
        digitalWrite(TestSerialEN, HIGH);                     //   Disconnects Serial signals
        digitalWrite(PicMCLRPGM, HIGH);                       //   Disconnects (floats) MCLR input
        digitalWrite(PicMCLRGND, HIGH);                       //   Disconnects (floats) MCLR input
        digitalWrite(MAXDAPSignalsEN, HIGH);                  //   Disconnects MAXDAP signals
        digitalWrite(HighResEN, HIGH);  HiResState = 1;       //   Connects 560 Ohm resistor 
        delay(30);                                            
        ADCReadAvgCurrent();                                  //   Measures (and averages) Current
 //       Serial.print("Current = "); Serial.println(CurrentReading);      
        delay(30); 
        digitalWrite(HighResEN, LOW);  HiResState = 0;        //   Reconnects 1 Ohm resistor (Disconnects 560 Ohm)
        Serial.print("Current = "); Serial.println(CurrentReading);   
        break;
    //---------------------------
      case 'x':                                             //  x ---> Performs an on-demand low-current calibration procedure (this occurs automatically each time the system is powered on)
        Serial.println("Current Calibration in progress");
//        ADCReadAvgCurrent();
//        LCDisplay();
        delay(2000);
        Calibration();
        VoltCalibration();
//        ADCReadAvgCurrent();
//        LCDisplay();
        Serial.println("OK x");
        break;
      case 'y':                                             //  y ---> Queries whether or or not DUT is present based on switch
        DUTPresent = digitalRead(DetSwitch);
        if (DUTPresent == 0) {
           Serial.println("DETECTED");
        }
        else{
           Serial.println("MISSING");
        }
        break;
      case '1':
        TamperSwitchPushTest();
        break;
      default:                                              //  Warns of entry of a command character not in the above list
        Serial.println("Invalid Command");
        break;
    }
  }
}

void Calibration() {
  do {
    digitalWrite(HighResEN, HIGH);  HiResState = 1;
    digitalWrite(CalLoadEN, LOW);
    ADCReadAvgCurrent();
    if (CurrentReading >= 2) {
      Serial.println("Load present! Cannot do calibration. Please remove load!");  
    }      
  } while (CurrentReading >= 2);
  
  DUTPresent = digitalRead(DetSwitch);
  while (DUTPresent == 0) {
    DUTPresent= digitalRead(DetSwitch);
    Serial.println("DUT connected! Cannot perform calibration. Please remove it to continue..");
    delay(300); 
  }
  Serial.println("Calibration in progress...");
//  lcd.setCursor(0, 0); lcd.print("RadioBridge Test");
//  lcd.setCursor(0, 1); lcd.print("Calibration Mode");
  digitalWrite(HighResEN, HIGH);  HiResState = 1;
  CurrentOffsetFactor = 0;
  CurrentGainFactor = 1;
  AvgNumMax = CalAvg;
//----------------------------------  
  digitalWrite(CalLoadEN, LOW);
  delay(100);
//  ADCCurrentRead();
  ADCReadAvgCurrent();
  if (CurrentReading < 0) {
    CurrentReading = 0;
  }
  CurrentOffsetFactor = 0 - CurrentReading;
//----------------------------------    
  digitalWrite(CalLoadEN, HIGH);
  delay(10);
//  ADCCurrentRead();
  ADCReadAvgCurrent();
//  CurrentGainFactor = CurrentCalRef/(CurrentReading + CurrentOffsetFactor);
  CurrentGainFactor = CurrentCalRef/CurrentReading;
//----------------------------------  
  Serial.print("Offset Factor = "); Serial.println(CurrentOffsetFactor);  
  Serial.print("Gain Factor = "); Serial.println(CurrentGainFactor);
  ADCReadAvgCurrent();
  Serial.print("Corrected Current = "); Serial.println(CurrentReading);
  delay(250);
//  lcd.clear();
  digitalWrite(HighResEN, LOW);  HiResState = 0;   // Switches back to 1 ohm resistor
  digitalWrite(CalLoadEN, LOW);                    // Disengages Cal Load and switches back to DUT
  AvgNumMax = OpAvg;
}

void VoltCalibration() {
  Serial.println("Voltage Calibration in progress...");
  digitalWrite(VRegSelect2, LOW);
  ADCVoltageFactor = 1;
  STDVoltageFactor = 1;
  
  ADCVoltageRead();
  delay(20);
  ADCVoltageFactor = VoltCalRef/VoltageReading;
 
  VoltageRead();
  delay(20);
  STDVoltageFactor = VoltCalRef/VoltageReading;

  Serial.print("ADC Gain Factor = "); Serial.println(ADCVoltageFactor, 3);  
  Serial.print("Std.Gain Factor = "); Serial.println(STDVoltageFactor, 3);
}

void VoltageRead() {
  int count = AvgNumMax;
  float TotalValue = 0;
  while (count > 0) {
    VoltageReading = analogRead(DUTVoltage);
    VoltageReading = (VoltageReading / ADCRes) * ADCReference;  
    TotalValue = TotalValue + VoltageReading;
    count--;
  }
  VoltageReading = TotalValue / AvgNumMax;
  VoltageReading = VoltageReading * STDVoltageFactor;
}

void ADCVoltageRead() {
  int count = AvgNumMax;
  float TotalValue = 0;
  while (count > 0) {
    VoltageReading = ADS.readADC(2);
    float f = ADS.toVoltage(1);
    VoltageReading = VoltageReading*f;
    TotalValue = TotalValue + VoltageReading;
    count--;
  }
  VoltageReading = TotalValue / AvgNumMax;
  VoltageReading = VoltageReading * ADCVoltageFactor;
}

//void ADCCurrentRead() {
//  CurrentReading = ADS.readADC_Differential_0_1();
//  float f = ADS.toVoltage(1);
//  switch (HiResState) {
//    case 1:
//      CurrentReading = abs(((CurrentReading * 500 * f) + CurrentOffsetFactor)* CurrentGainFactor);
//      break;
//    case 0:
//      CurrentReading = abs(CurrentReading * HiCurrFactor * f);
//      break;
//  }
//}

//void LCDisplay() {
//  delay(200);
//  lcd.setCursor(0, 0); lcd.print("Voltage:");
//  lcd.setCursor(9, 0); lcd.print(VoltageReading);
//  lcd.setCursor(14, 0); lcd.print("V");
//  ADCAverageFilter();
//  lcd.setCursor(0, 1); lcd.print("Current:");
//  lcd.setCursor(9, 1); lcd.print(CurrentReading);
//  switch (HiResState) {
//  case 1:
//    lcd.setCursor(14, 1); lcd.print("uA");
//    break;
//  case 0:
//    lcd.setCursor(14, 1); lcd.print("mA");
//    break;
//  }
//  lcd.setCursor(13, 1); lcd.print(" ");
//}

void ADCReadAvgCurrent() {
  int count = AvgNumMax;
  float TotalValue = 0;
  float CurrentReadingArray[count];
  float RealAvg = 0;
  int RealAvgCount = 0;
  float AVGCurrentReading = 0;
  for(int i = 0; i < count; i++)
  {
    CurrentReading = ADS.readADC_Differential_0_1();
    if (CurrentReading < 0) {
      CurrentReading = 0;
    }
    TotalValue = TotalValue + CurrentReading;
    CurrentReadingArray[i] = CurrentReading;
    
  }

  AVGCurrentReading = TotalValue / AvgNumMax;
  for(int j = 0; j < count; j++)
  {
    if( CurrentReadingArray[j] >= (AVGCurrentReading - AVGCurrentReading * 0.05) && CurrentReadingArray[j] <= (AVGCurrentReading + AVGCurrentReading * 0.05))
    {
      RealAvg += CurrentReadingArray[j];
      RealAvgCount++;
    }
  }
  if(RealAvgCount < 10)
  {
    RealAvg = 0;
    RealAvgCount = 30;
    shellSortKnuth(CurrentReadingArray, count);
    for(int k = (count/2) - 15; k < (count/2) + 15; k++)
    {
      RealAvg += CurrentReadingArray[k];
    }
  }
  
  CurrentReading = RealAvg / RealAvgCount;
  
  double f = ADS.toVoltage(1);
  switch (HiResState) {
    case 1:
     CurrentReading = ((CurrentReading * 2000 * f) + CurrentOffsetFactor)* CurrentGainFactor;;
      break;
    case 0:
      CurrentReading = CurrentReading * HiCurrFactor * f;
      break;
  }
}


void ADCReadMaxCurrent()
{
  float voltage;
  float ampere ;
  float sum = 0;
  long startMillis = millis();
  int counter = 0;

  while ( millis() - startMillis < 2000 )   // Was 1000 earlier
  {
    voltage = ADS.readADC_Differential_0_1() * 400 ;
    ampere  = voltage * CurrentGainFactor ;
    ampere /= 1000.0;
    sum += sq(ampere);
    counter = counter + 1;
    delay(5);                                       // Intentional delay between acquisitions 
  }
  ampere = sqrt(sum / counter);
  Serial.print(sum);
  Serial.print(" , ");

  Serial.println(counter); 
  CurrentReading = ampere;
}

void TamperSwitchPushTest()
{
  myservo.write(55);
  delay(500);
  myservo.write(105);
}