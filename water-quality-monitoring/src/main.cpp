#include <Arduino.h>
#include "config.h"
#include <Wire.h>
#include <SoftWire.h>
#include <Adafruit_MCP23017.h>
#include <AccelStepper.h>
#include <MCP3017AccelStepper.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <STM32RTC.h>
#include "AS726X.h"

#include <time.h>

/* Temperature sensor global variables ****************************************/
OneWire oneWire(TEMPERATURE_PIN);
DallasTemperature ds18b20(&oneWire);
float Celcius = 0;

/* RTC global variables *******************************************************/
#define COMPILE_DATE_TIME (__DATE__ " " __TIME__)

STM32RTC &rtc = STM32RTC::getInstance();

/* Change these values to set the current initial time */
byte seconds = 0;
byte minutes = 0;
byte hours = 14;

/* Change these values to set the current initial date */
//HÉTFŐ AZ 1 (nem 0-tól indul)
byte weekDay = 0;
byte day = 10;
byte month = 2;
byte year = 21;

/* Stepper motor global variables *********************************************/
Adafruit_MCP23017 mcp1;
Adafruit_MCP23017 mcp2;

// interface, step, dir, en
MCP3017AccelStepper reagent1Dispenser(AccelStepper::DRIVER, 9, 10);  // 10, 9, 8
MCP3017AccelStepper reagent2Dispenser(AccelStepper::DRIVER, 13, 14); // 13, 14, 12
MCP3017AccelStepper reagent3Dispenser(AccelStepper::DRIVER, 6, 5);   // 6, 5, 7
MCP3017AccelStepper reagent4Dispenser(AccelStepper::DRIVER, 2, 1);   // 2, 1, 3
MCP3017AccelStepper reagent5Dispenser(AccelStepper::DRIVER, 14, 13); // 14, 13, 15

MCP3017AccelStepper distilledWaterPump(AccelStepper::DRIVER, 10, 9); // 10, 9, 11
// az utolsó motor nem működik                                        // 5, 6, 4

/* Spectral sensor global variables *******************************************/
AS726X as7262; //Creates the sensor object
byte GAIN = 0;
byte MEASUREMENT_MODE = 0;

/* PWM-driven motor functions *************************************************/
// dutyCycle: 0-100 (0 -> OFF, 100 -> FULL)
void setMixerMotorPWM(uint8_t dutyCycle)
{
  // invertálni kell, mivel N-csatornás a MOSFET:
  int invertedValue = 100 - dutyCycle;

  // 0->0 és 255->100 map-elése
  int mappedValue = map(invertedValue, 0, 100, 0, 255);
  Serial.println(mappedValue);
  analogWrite(MIXER_MOTOR_PIN, mappedValue);
}

void setPumpPWM(int32_t dutyCycle)
{
  if (dutyCycle < 0)
  {
    //vissza
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    dutyCycle = abs(dutyCycle);
  }
  else if (dutyCycle > 0)
  {
    //előre
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  // invertálni kell, mivel N-csatornás a MOSFET:
  int invertedValue = 100 - dutyCycle;

  // 0->0 és 255->100 map-elése
  int mappedValue = map(invertedValue, 0, 100, 0, 255);
  analogWrite(PUMP_PWM_PIN, mappedValue);
}

// logikai magasra nyit, de az N-csatornás FET miatt invertált (0-ra nyit)
enum ValveState
{
  OPEN = 0,
  CLOSE = 1
};

void setValve(int pin, ValveState state)
{
  digitalWrite(pin, state);
}

void peltierOn()
{
  digitalWrite(PELTIER_PIN, LOW);
}

void peltierOff()
{
  digitalWrite(PELTIER_PIN, HIGH);
}

void print2digits(int number)
{
  if (number < 10)
  {
    Serial.print("0");
  }
  Serial.print(number);
}

void printDateTime()
{
  // Print date...
  print2digits(rtc.getDay());
  Serial.print("/");
  print2digits(rtc.getMonth());
  Serial.print("/");
  print2digits(rtc.getYear());
  Serial.print(" ");

  // ...and time
  print2digits(rtc.getHours());
  Serial.print(":");
  print2digits(rtc.getMinutes());
  Serial.print(":");
  print2digits(rtc.getSeconds());

  Serial.println();
}

int StringToInt(const char *pString)
{
  int value = 0;

  // skip leading 0 and spaces
  while ('0' == *pString || *pString == ' ')
  {
    pString++;
  }

  // calculate number until we hit non-numeral char
  while ('0' <= *pString && *pString <= '9')
  {
    value *= 10;
    value += *pString - '0';
    pString++;
  }
  return value;
}

// This function takes a 20 chars long textual (English) representation of a date-time value
// of the kind "Feb 28 2017 00:38:05" and convert it to a standard time_t value
// This can be useful to convert the concatenation of the __DATE__ and __TIME__
// compilation date and time strings to a time_t value.
//
// Example:
//
//  #define COMPILE_DATE_TIME (__DATE__ " " __TIME__)
//  time_t compiletime = str20ToTime(COMPILE_DATE_TIME);
//
// I'm not happy with this function name/implementation and it might change in the future...
extern time_t str20ToTime(const char *date)
{
  struct tm tm_time;

  // Get the month  (N.B.: Jan=0, Feb=1, etc...)
  switch (date[0])
  {
  case 'J':
    if (date[1] == 'a')
      tm_time.tm_mon = 0;
    else if (date[2] == 'n')
      tm_time.tm_mon = 5;
    else
      tm_time.tm_mon = 6;
    break;
  case 'F':
    tm_time.tm_mon = 1;
    break;
  case 'A':
    tm_time.tm_mon = date[1] == 'p' ? 3 : 7;
    break;
  case 'M':
    tm_time.tm_mon = date[2] == 'r' ? 2 : 4;
    break;
  case 'S':
    tm_time.tm_mon = 8;
    break;
  case 'O':
    tm_time.tm_mon = 9;
    break;
  case 'N':
    tm_time.tm_mon = 10;
    break;
  case 'D':
    tm_time.tm_mon = 11;
    break;
  }

  // Get the day
  tm_time.tm_mday = (int8_t)StringToInt(date + 4);

  // Get the year
  int16_t year = (int16_t)StringToInt(date + 7);
  tm_time.tm_year = year - 1900;

  // Get the time
  tm_time.tm_hour = (int8_t)StringToInt(date + 12);
  tm_time.tm_min = (int8_t)StringToInt(date + 15);
  tm_time.tm_sec = (int8_t)StringToInt(date + 18);
  tm_time.tm_wday = 0;
  tm_time.tm_isdst = 0;

  return mktime(&tm_time);
}


#define SET_TIME_FROM_BUILD



/* Initialization *************************************************************/
void setup()
{
  // put your setup code here, to run once:
  pinMode(PC13, OUTPUT);

  /* Serial debug init **********************************************************/
  Serial.begin(115200);
  while (!Serial)
    ;

  delay(2000);
  Wire.begin();
  delay(2000);
  Serial.println("Water Minilab.");

  /* Stepper motors + IO Expander init ******************************************/
  mcp1.begin();  //addr 0 = A2 low , A1 low , A0 low  000
  mcp2.begin(4); //addr 4 = A2 high , A1 low , A0 low  100

  reagent1Dispenser.setMcp(mcp1);
  reagent1Dispenser.MCP3017_pinMode(8, OUTPUT);
  reagent1Dispenser.enableOutputs();
  reagent1Dispenser.setMaxSpeed(150.0);
  reagent1Dispenser.setAcceleration(100.0);
  reagent1Dispenser.moveTo(200);

  reagent2Dispenser.setMcp(mcp1);
  reagent2Dispenser.MCP3017_pinMode(12, OUTPUT);
  reagent2Dispenser.enableOutputs();
  reagent2Dispenser.setMaxSpeed(150.0);
  reagent2Dispenser.setAcceleration(100.0);
  reagent2Dispenser.moveTo(200);

  reagent3Dispenser.setMcp(mcp1);
  reagent3Dispenser.MCP3017_pinMode(7, OUTPUT);
  reagent3Dispenser.enableOutputs();
  reagent3Dispenser.setMaxSpeed(150.0);
  reagent3Dispenser.setAcceleration(100.0);
  reagent3Dispenser.moveTo(200);

  reagent4Dispenser.setMcp(mcp1);
  reagent4Dispenser.MCP3017_pinMode(3, OUTPUT);
  reagent4Dispenser.enableOutputs();
  reagent4Dispenser.setMaxSpeed(150.0);
  reagent4Dispenser.setAcceleration(100.0);
  reagent4Dispenser.moveTo(200);

  mcp2.begin(4);

  reagent5Dispenser.setMcp(mcp2);
  reagent5Dispenser.MCP3017_pinMode(15, OUTPUT);
  reagent5Dispenser.enableOutputs();
  reagent5Dispenser.setMaxSpeed(150.0);
  reagent5Dispenser.setAcceleration(100.0);
  reagent5Dispenser.moveTo(200);

  distilledWaterPump.setMcp(mcp2);
  distilledWaterPump.MCP3017_pinMode(11, OUTPUT);
  distilledWaterPump.enableOutputs();
  distilledWaterPump.setMaxSpeed(150.0);
  distilledWaterPump.setAcceleration(100.0);
  distilledWaterPump.moveTo(200);

  //motorok "elernyesztése":
  //nagyon fontos!!!!! így nem vesz fel áramot:
  reagent1Dispenser.MCP3017_digitalWrite(8, HIGH);
  reagent2Dispenser.MCP3017_digitalWrite(12, HIGH);
  reagent3Dispenser.MCP3017_digitalWrite(7, HIGH);
  reagent4Dispenser.MCP3017_digitalWrite(3, HIGH);
  reagent5Dispenser.MCP3017_digitalWrite(15, HIGH);
  distilledWaterPump.MCP3017_digitalWrite(11, HIGH);

  /* Spectral sensor init *******************************************************/
  delay(100);
  as7262.begin(Wire, GAIN, MEASUREMENT_MODE); //Initializes the sensor with non default values
  Serial.println("AS7265 initialized.");

  /* Pin direction init *********************************************************/
  pinMode(VALVE_1_PIN, OUTPUT);
  pinMode(VALVE_2_PIN, OUTPUT);
  pinMode(VALVE_3_PIN, OUTPUT);
  pinMode(VALVE_4_PIN, OUTPUT);

  pinMode(PELTIER_PIN, OUTPUT);

  pinMode(MIXER_MOTOR_PIN, OUTPUT);
  pinMode(PUMP_PWM_PIN, OUTPUT);

  // 1. pumpa (DC) H-híd:
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(WATER_LOW_LEVEL_PIN, INPUT);
  pinMode(WATER_HIGH_LEVEL_PIN, INPUT);

  /* Default pin states *********************************************************/
  setValve(VALVE_1_PIN, ValveState::CLOSE);
  setValve(VALVE_2_PIN, ValveState::CLOSE);
  setValve(VALVE_3_PIN, ValveState::CLOSE);
  setValve(VALVE_4_PIN, ValveState::CLOSE);
  peltierOff();

  // 200 Hz frekvencia
  analogWriteFrequency(200);

  setMixerMotorPWM(0);
  setPumpPWM(0);

  Serial.println("Pin directions and default states initialized.");

  // Select RTC clock source: LSI_CLOCK, LSE_CLOCK or HSE_CLOCK.
  // By default the LSI is selected as source.
  //rtc.setClockSource(STM32RTC::LSE_CLOCK);
  rtc.begin(); // initialize RTC 24H format
  Serial.println("RTC initialized.");

#if defined(SET_TIME_FROM_BUILD)
char *compile_date_time = COMPILE_DATE_TIME;
  time_t compiled_time_t = str20ToTime(compile_date_time);
  rtc.setEpoch(compiled_time_t);
#else
  rtc.setTime(hours, minutes, seconds);
  rtc.setDate(weekDay, day, month, year);
#endif

  printDateTime();

  ds18b20.begin();
  Serial.println("DS18B20 initialized.");

  ds18b20.requestTemperatures();
  Serial.print("Temperature: ");
  Serial.println(ds18b20.getTempCByIndex(0));
}

/******************************************************************************/
void I2CSendString(char *str)
{
  for (int i = 0; i < strlen(str); i++)
  {
    Wire.beginTransmission(8); // starts transmit to device (8-Slave Arduino Address)
    Wire.write(str[i]);        // sends the value x to Slave
    Wire.endTransmission();    // stop transmitting
  }
}

/* I. MINTAVÉTEL **************************************************************/
void samplingTask()
{
  Serial.println("***** MINTAVETEL *************");

  /* 1. víztelenítés **********************************************************/
  Serial.println("- Viztelenites...");

  // 2. szelep nyít, többi zárva
  setValve(VALVE_2_PIN, ValveState::OPEN);

  // 1. pumpa visszafele, időtartama: "empty"
  setPumpPWM(-10); // 10 %-os PWM

  while (1)
  {
    // ha nincs víz a rendszerben akkor 0-t ad vissza a digitalRead
    // ha van víz, akkor 1
    // nekünk addig kel járatni a pumpát, amíg el nem tűnik az alsó elektródáról a víz, vagyis amíg alacsonnyá nem válik
    if (digitalRead(WATER_LOW_LEVEL_PIN) == LOW)
    {
      // motor leáll
      setPumpPWM(0);
      break;
    }
  }

  /* 2. minta felszívása ******************************************************/
  Serial.println("- Minta felszivasa...");

  // 1. pumpa előre, időtartama: amíg az eletróda jelez, de annák X ms-el tovább
  setPumpPWM(10);

  while (1)
  {
    // pumpa megy....
    // amint a felső elektródát eléri a víz, vagyis HIGH lesz, akkor leáll a pumpa
    if (digitalRead(WATER_HIGH_LEVEL_PIN) == HIGH)
    {
      // + X ms késleltetés
      delay(100); //TODO: finomhangolni, define-olni
      setPumpPWM(0);
      break;
    }
  }

  /* 3. keverés ***************************************************************/
  Serial.println("- Keveres...");
  setMixerMotorPWM(25);
  delay(5000); //TODO: valamennyi delay: finomhangolni, define-ba kirakni az időt!!!
  setMixerMotorPWM(0);

  /* 4. víztelenítés **********************************************************/
  Serial.println("- Viztelenites...");
  setPumpPWM(-10); // 10 %-os PWM

  while (1)
  {
    // ha nincs víz a rendszerben akkor 0-t ad vissza a digitalRead
    // ha van víz, akkor 1
    // nekünk addig kel járatni a pumpát, amíg el nem tűnik az alsó elektródáról
    // a víz, vagyis amíg magassá nem válik
    if (digitalRead(WATER_LOW_LEVEL_PIN) == LOW)
    {
      setPumpPWM(0);
      break;
    }
  }

  /* 5. minta felszívása ******************************************************/
  Serial.println("- Minta felszivasa...");
  //minta felszívás, de a vízszint elérése után nem kell várni

  setPumpPWM(10);

  while (1)
  {
    // pumpa megy....
    // amint a felső elektródát eléri a víz, vagyis HIGH lesz, akkor leáll a pumpa
    if (digitalRead(WATER_HIGH_LEVEL_PIN) == HIGH)
    {
      setPumpPWM(0);
      break;
    }
  }

  /* 6. 2. szelep zár *********************************************************/
  Serial.println("- 2. szelep zar...");
  setValve(VALVE_2_PIN, ValveState::CLOSE);
}

//ezt le kell tesztelni, hogy a mi motorunknak mennyi idő kel mire az adott millilitert felszívja
#define X_MILLILITER_FOR_AMMONIA 3000
#define X_MILLILITER_FOR_PHOSPHATE 3000
#define X_MILLILITER_FOR_NITRITE 3000
#define X_MILLILITER_FOR_NITRATE 3000

//ez a függvény az egyik pumpára működik
//a többi reagenshez le kell másolni, csak reagent2Dispenser, reagent3Dispenser... objektummal
void addReagentForAmmonia(unsigned long milliliter)
{
  unsigned long startMillis = millis();
  unsigned long endMillis = startMillis + milliliter;

  reagent1Dispenser.setSpeed(100);

  reagent1Dispenser.MCP3017_digitalWrite(8, LOW);

  while (1)
  {
    if (millis() < endMillis)
    {
      reagent1Dispenser.runSpeed();
    }
    else
    {
      reagent1Dispenser.stop();
      break;
    }
  }
  // motor "elernyesztése":
  reagent1Dispenser.MCP3017_digitalWrite(8, HIGH);
}

void addreagentForPhosphate(unsigned long milliliter)
{
  unsigned long startMillis = millis();
  unsigned long endMillis = startMillis + milliliter;

  reagent2Dispenser.setSpeed(100);

  reagent2Dispenser.MCP3017_digitalWrite(12, LOW);

  while (1)
  {
    if (millis() < endMillis)
    {
      reagent2Dispenser.runSpeed();
    }
    else
    {
      reagent2Dispenser.stop();
      break;
    }
  }
  // motor "elernyesztése":
  reagent2Dispenser.MCP3017_digitalWrite(12, HIGH);
}

void addreagentForNitrite(unsigned long milliliter)
{
  unsigned long startMillis = millis();
  unsigned long endMillis = startMillis + milliliter;

  reagent3Dispenser.setSpeed(100);

  reagent3Dispenser.MCP3017_digitalWrite(7, LOW);

  while (1)
  {
    if (millis() < endMillis)
    {
      reagent3Dispenser.runSpeed();
    }
    else
    {
      reagent3Dispenser.stop();
      break;
    }
  }
  // motor "elernyesztése":
  reagent3Dispenser.MCP3017_digitalWrite(7, HIGH);
}

void addreagentForNitrate(unsigned long milliliter)
{
  unsigned long startMillis = millis();
  unsigned long endMillis = startMillis + milliliter;

  reagent4Dispenser.setSpeed(100);

  reagent4Dispenser.MCP3017_digitalWrite(3, LOW);

  while (1)
  {
    if (millis() < endMillis)
    {
      reagent4Dispenser.runSpeed();
    }
    else
    {
      reagent4Dispenser.stop();
      break;
    }
  }
  // motor "elernyesztése":
  reagent4Dispenser.MCP3017_digitalWrite(3, HIGH);
}

//WAITTIME_MS?? vagy hosszabb idő szükséges ms-nél?
#define AMMONIA_WAITTIME_MS 5000
#define PHOSPHATE_WAITTIME_MS 5000
#define NITRITE_WAITTIME_MS 5000
#define NITRATE_WAITTIME_MS 5000

enum Reagent
{
  AMMONIA = 0,
  PHOSPHATE = 1,
  NITRITE = 2,
  NITRATE = 3
};

/* II. REAKCIÓ ****************************************************************/
void reactionTask(Reagent reagent)
{
  Serial.println("***** REAKCIO ****************");

  /* 1. keverés be ************************************************************/
  Serial.println("- Keveres...");
  setMixerMotorPWM(50);
  delay(5000); //TODO: valamennyi delay: finomhangolni, define-ba kirakni az időt!!!
  setMixerMotorPWM(0);

  /* 2. reagens adagolás ******************************************************/
  Serial.println("- Reagens adagolas...");
  if (reagent == Reagent::AMMONIA)
  {
    // 2. pumpa be (vagy 3. vagy 4. vagy 5. vagy 6.), időtartam: ammonia= ? ms (vagy nitrit vagy foszfor vagy ntrat).  (minden szelep zárva)
    addReagentForAmmonia(X_MILLILITER_FOR_AMMONIA);

    /* 3. várakozás ***********************************************************/
    Serial.println("- Varakozas...");
    delay(AMMONIA_WAITTIME_MS);
  }
  else if (reagent == Reagent::PHOSPHATE)
  {
    addreagentForPhosphate(X_MILLILITER_FOR_PHOSPHATE);
    Serial.println("- Varakozas...");
    delay(PHOSPHATE_WAITTIME_MS);
  }
  else if (reagent == Reagent::NITRITE)
  {
    addreagentForNitrite(X_MILLILITER_FOR_NITRITE);
    Serial.println("- Varakozas...");
    delay(NITRITE_WAITTIME_MS);
  }
  else if (reagent == Reagent::NITRATE)
  {
    addreagentForNitrate(X_MILLILITER_FOR_NITRATE);
    Serial.println("- Varakozas...");
    delay(NITRATE_WAITTIME_MS);
  }
}

/* III. MÉRÉS *****************************************************************/
void measurementTask()
{
  Serial.println("MERES");

  /* 1. 3. és 4. szelep nyit, időtartama: cuvette= ? ms ***********************/
  Serial.println("- 3. es 4. szelep nyit...");
  setValve(VALVE_3_PIN, ValveState::OPEN);
  setValve(VALVE_4_PIN, ValveState::OPEN);
  delay(2000);

  /* 2. 3. és 4. szelep zár ***************************************************/
  Serial.println("- 3. es 4. szelep zar...");
  setValve(VALVE_3_PIN, ValveState::CLOSE);
  setValve(VALVE_4_PIN, ValveState::CLOSE);

  // SZERINTEM MÉRJÜNK TÖBBSZÖR
  for (int i = 0; i < 5; i++)
  {
    /* 3. led be **************************************************************/
    Serial.println("- LED be...");
    as7262.takeMeasurementsWithBulb();

    /* 4. mérés szenzorral ****************************************************/
    Serial.println("- Meres szenzorral...");

    float calibratedViolet;
    float calibratedBlue;
    float calibratedGreen;
    float calibratedYellow;
    float calibratedOrange;
    float calibratedRed;

    calibratedViolet = as7262.getCalibratedViolet();
    calibratedBlue = as7262.getCalibratedBlue();
    calibratedGreen = as7262.getCalibratedGreen();
    calibratedYellow = as7262.getCalibratedYellow();
    calibratedOrange = as7262.getCalibratedOrange();
    calibratedRed = as7262.getCalibratedRed();

    /* 3. led ki **************************************************************/
    Serial.println("- LED ki...");
    as7262.disableBulb();

    //pakoljunk bele hőmérséklet adatot és unix timestampet is
    ds18b20.requestTemperatures();
    float temp = ds18b20.getTempCByIndex(0);

    uint32_t epoch = rtc.getEpoch();

    char measurementString[70] = "";
    memset(measurementString, '\0', 70);

    //format: id;V;B;G;Y;O;R;TEMPERATUR;UNIXTIME
    sprintf(measurementString, "%d;%d.%d;%d.%d;%d.%d;%d.%d;%d.%d;%d.%d;%d.%d;%lu\n",
            i,
            (int)calibratedViolet, (int)(calibratedViolet * 10) - ((int)calibratedViolet * 10),
            (int)calibratedBlue, (int)(calibratedBlue * 10) - ((int)calibratedBlue * 10),
            (int)calibratedGreen, (int)(calibratedGreen * 10) - ((int)calibratedGreen * 10),
            (int)calibratedYellow, (int)(calibratedYellow * 10) - ((int)calibratedYellow * 10),
            (int)calibratedOrange, (int)(calibratedOrange * 10) - ((int)calibratedOrange * 10),
            (int)calibratedRed, (int)(calibratedRed * 10) - ((int)calibratedRed * 10),
            (int)temp, (int)(temp * 10) - ((int)temp * 10),
            epoch);
    /* 6. adatküldés **********************************************************/
    Serial.print("- Adatkuldes...");

    // (a másik board fogja csinálni LoRa-n)
    I2CSendString(measurementString);

    Serial.print(" [Packet sent on I2C: ");
    Serial.print(measurementString);
    Serial.println(" ]");
    delay(10000);
  }
}

/* IV. ÖBLÍTÉS ****************************************************************/
void washOutTask()
{
  Serial.println("***** OBLITES ****************");
  /* 1. 3. és 4. szelep nyit, várakozás: empty time=? ms **********************/

  Serial.println("- 3. es 4. szelep nyit...");
  setValve(VALVE_3_PIN, ValveState::OPEN);
  setValve(VALVE_4_PIN, ValveState::OPEN);

  // várakozsás (empty time = ? ms):
  delay(1000);

  /* 2. 2. szelep nyit, keverés be (dc=high) **********************************/
  Serial.println("- 2. szelep nyit, keveres be...");
  setValve(VALVE_2_PIN, ValveState::OPEN);
  setMixerMotorPWM(25);

  /* 3. 1. szelep nyit, elektróda jelzésére zár, delay ? ms *******************/
  Serial.println("- 1. szelep nyit, elektrda jelzesere zar...");
  setValve(VALVE_1_PIN, ValveState::OPEN);
  while (1)
  {
    // pumpa megy....
    // amint a felső elektródát eléri a víz, vagyis HIGH lesz, akkor zár az 1. szelep
    if (digitalRead(WATER_HIGH_LEVEL_PIN) == HIGH)
    {
      setValve(VALVE_1_PIN, ValveState::CLOSE);
      break;
    }
  }

  /* 4. 1. szelep nyit, elektróda jelzésére zár, delay ? ms *******************/
  Serial.println("- 1. szelep nyit, elektrda jelzesere zar...");
  setValve(VALVE_1_PIN, ValveState::OPEN);
  while (1)
  {
    // pumpa megy....
    // amint a felső elektródát eléri a víz, vagyis HIGH lesz, akkor zár az 1. szelep
    if (digitalRead(WATER_HIGH_LEVEL_PIN) == HIGH)
    {
      setValve(VALVE_1_PIN, ValveState::CLOSE);
      break;
    }
  }

  /* 5. 1. szelep nyit, elektróda jelzésére zár, delay ? ms *******************/
  Serial.println("- 1. szelep nyit, elektrda jelzesere zar...");
  setValve(VALVE_1_PIN, ValveState::OPEN);
  while (1)
  {
    // pumpa megy....
    // amint a felső elektródát eléri a víz, vagyis HIGH lesz, akkor zár az 1. szelep
    if (digitalRead(WATER_HIGH_LEVEL_PIN) == HIGH)
    {
      setValve(VALVE_1_PIN, ValveState::CLOSE);
      break;
    }
  }

  /* 6. 3. és 4. szelep zár ***************************************************/
  Serial.println("- 3. es 4. szelep zar...");
  setValve(VALVE_3_PIN, ValveState::CLOSE);
  setValve(VALVE_4_PIN, ValveState::CLOSE);

  /* 7. ha elektróda jelez 1. szelep zár **************************************/
  Serial.println("- Ha elektroda jelez, 1. szelep zar...");
  setValve(VALVE_1_PIN, ValveState::OPEN);
  while (1)
  {
    // pumpa megy....
    // amint a felső elektródát eléri a víz, vagyis HIGH lesz, akkor zár az 1. szelep
    if (digitalRead(WATER_HIGH_LEVEL_PIN) == HIGH)
    {
      setValve(VALVE_1_PIN, ValveState::CLOSE);
      break;
    }
  }

  /* 8. 2. szelep zár *********************************************************/
  setValve(VALVE_2_PIN, ValveState::CLOSE);
  Serial.println("- 2. szelep zar...");

  /* 9. keverés leáll *********************************************************/
  Serial.println("- Keveres leall...");
  setMixerMotorPWM(0);
}

/* V. SZŰRŐ MOSÁS *************************************************************/
void filterWashTask()
{
  Serial.println("***** SZURO MOSAS ************");
  /* 1. 1. szelep nyit, minden más zárva **************************************/
  Serial.println("- 1. szelep nyit, minden mas zarva...");
  setValve(VALVE_1_PIN, ValveState::OPEN);

  /* 2. 1. pumpa visszafelé, delay ??? ms *************************************/
  Serial.println("- 1. pumpa visszafele...");
  setPumpPWM(-10);
  delay(3000);

  /* 3. 1. pumpa leáll ********************************************************/
  Serial.println("- 1. pumpa leall...");
  setPumpPWM(0);

  /* 4. 1. szelep zár *********************************************************/
  Serial.println("- 1. szelep zar...");
  setValve(VALVE_1_PIN, ValveState::CLOSE);
}

#define CUVETTE_CLEAN_MS 5000

/* VI. KÜVETTA MOSÁS **********************************************************/
void cuvetteWashingTask()
{
  Serial.println("***** KUVETTE MOSAS **********");
  /* 1. 4. szelep nyit, többi zárva *******************************************/
  Serial.println("- 4. szelep nyit, tobbi zarva...");
  setValve(VALVE_4_PIN, ValveState::OPEN);

  /* 2, 3. 7. pumpa bekapcs, időtartam: cuvette clean= ? ms *******************/
  Serial.println("- 7. pumpa bekapcs...");
  distilledWaterPump.MCP3017_digitalWrite(11, LOW);

  unsigned long startMillis = millis();
  unsigned long endMillis = startMillis + CUVETTE_CLEAN_MS;

  distilledWaterPump.setSpeed(100);

  while (1)
  {
    if (millis() < endMillis)
    {
      distilledWaterPump.runSpeed();
    }
    else
    {
      distilledWaterPump.stop();
      break;
    }
  }

  Serial.println("- 7. pumpa kikapcs...");
  distilledWaterPump.MCP3017_digitalWrite(11, HIGH);

  /* 4. 4. szelep zár *********************************************************/
  Serial.println("- 4. szelep zar...");
  setValve(VALVE_4_PIN, ValveState::OPEN);

  // 5) --- alvó mód
}

#define Y_MS 250

/* VII. RENDSZER MOSÁS (napi egyszer) *****************************************/
void systemWashingTask()
{
  Serial.println("***** RENDSZER MOSAS *********");

  /* 1. 3. szelep nyit ********************************************************/
  Serial.println("- 3. szelep nyit...");
  setValve(VALVE_3_PIN, ValveState::OPEN);

  /* 2. 7. pumpa be, elektróda szintje + y ms, ezután kikapcs *****************/
  Serial.println("- 7. pumpa be...");
  distilledWaterPump.MCP3017_digitalWrite(11, LOW);

  bool delayRunning = false;
  unsigned long startDelay = 0;
  unsigned long endDelay = 0;

  distilledWaterPump.setSpeed(100);
  while (1)
  {
    // Ez a rész azért ilyen bonyolult, mert egyszerre kell figyelni az
    // elektródára, azután pedig a késleltetésre. Sima delay()-t it nem lehet
    // használni a léptetőmotor miatt
    if (digitalRead(WATER_HIGH_LEVEL_PIN) == LOW && delayRunning == false)
    {
      startDelay = millis();
      endDelay = startDelay + Y_MS;

      delayRunning = true;
    }

    if (delayRunning)
    {
      if (millis() < endDelay)
      {
        distilledWaterPump.runSpeed();
      }
      else
      {
        distilledWaterPump.stop();
        break;
      }
    }
    else
    {
      distilledWaterPump.runSpeed();
    }
  }

  distilledWaterPump.MCP3017_digitalWrite(11, HIGH);
  Serial.println("- 7. pumpa ki...");

  /* 3. 3. szelep zár, 2. szelep nyit *****************************************/
  Serial.println("- 3. szelep zar, 2. szelep nyit...");
  setValve(VALVE_3_PIN, ValveState::CLOSE);
  setValve(VALVE_2_PIN, ValveState::OPEN);

  /* 4. 1. pumpa visszafelé, z ms aztán leáll *********************************/
  Serial.println("- 1. pumpa visszafele...");
  setPumpPWM(-10); // 10 %-os PWM

  // (Z ms helyett inkább elektródát figyeljük):
  while (1)
  {
    if (digitalRead(WATER_LOW_LEVEL_PIN) == LOW)
    {
      //motor leáll
      setPumpPWM(0);
      break;
    }
  }

  /* 5. 2. szelep zár *********************************************************/
  Serial.println("- 2. szelep zar...");
  setValve(VALVE_2_PIN, ValveState::CLOSE);

  /* 6. 3. szelep nyit, 7. pumpa indít, elektróda szintje + y ms **************/
  Serial.println("- 3. szelep nyit, 7. pumpa indit...");
  setValve(VALVE_3_PIN, ValveState::OPEN);

  distilledWaterPump.MCP3017_digitalWrite(11, LOW);

  delayRunning = false;
  startDelay = 0;
  endDelay = 0;

  distilledWaterPump.setSpeed(100);
  while (1)
  {

    if (digitalRead(WATER_HIGH_LEVEL_PIN) == LOW && delayRunning == false)
    {
      startDelay = millis();
      endDelay = startDelay + Y_MS;
      delayRunning = true;
    }

    if (delayRunning)
    {
      if (millis() < endDelay)
      {
        distilledWaterPump.runSpeed();
      }
      else
      {
        distilledWaterPump.stop();
        break;
      }
    }
    else
    {
      distilledWaterPump.runSpeed();
    }
  }

  distilledWaterPump.MCP3017_digitalWrite(11, HIGH);
  Serial.println("- 7. pumpa ki...");

  /* 7. 3. szelep zár *********************************************************/
  Serial.println("- 3. szelep zar...");
  setValve(VALVE_3_PIN, ValveState::CLOSE);

  /* 8. 1. szelep nyit, 1. pumpa visszafelé Z ms aztán leáll ******************/
  Serial.println("- 1. szelep nyit, 1. pumpa visszafele...");
  setValve(VALVE_1_PIN, ValveState::OPEN);
  setPumpPWM(-10); // 10 %-os PWM

  // (Z ms helyett inkább elektródát figyeljük):
  while (1)
  {
    if (digitalRead(WATER_LOW_LEVEL_PIN) == LOW)
    {
      //motor leáll
      setPumpPWM(0);
      break;
    }
  }
  Serial.println("- 1. pumpa leall...");

  /* 9. 1. szelep zár *********************************************************/
  Serial.println("- 1. szelep zar...");
  setValve(VALVE_1_PIN, ValveState::CLOSE);

  // 10 --- alvó mód
}

/* Main loop ******************************************************************/
void loop()
{

  uint8_t hours = rtc.getHours();
  uint8_t minutes = rtc.getMinutes();
  uint8_t seconds = rtc.getSeconds();

  printDateTime();
  delay(500);

  if (hours == 12 && minutes == 0 && seconds == 0)
  {
    samplingTask();
    reactionTask(Reagent::AMMONIA);
    measurementTask();
    washOutTask();
    filterWashTask();
    cuvetteWashingTask();
  }
  else if (hours == 14 && minutes == 0 && seconds == 0)
  {
    samplingTask();
    reactionTask(Reagent::NITRATE);
    measurementTask();
    washOutTask();
    filterWashTask();
    cuvetteWashingTask();
  }
  else if (hours == 16 && minutes == 0 && seconds == 0)
  {
    samplingTask();
    reactionTask(Reagent::PHOSPHATE);
    measurementTask();
    washOutTask();
    filterWashTask();
    cuvetteWashingTask();
  }
  else if (hours == 24 && minutes == 0 && seconds == 0)
  {
    systemWashingTask();
  }
}
/******************************************************************************/
