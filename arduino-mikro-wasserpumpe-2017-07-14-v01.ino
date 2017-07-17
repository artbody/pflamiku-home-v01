
//debug
boolean debug = false;
//#define DEBUG_off
#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINT(x)  Serial.print (x)
#define DEBUG_PRINTLN(x)  Serial.println (x)
#define INIT_SERIAL(x) Serial.begin(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define INIT_SERIAL(x)
#endif

// Capacity
const int OUT_PIN = A1; // +pol cap
const int IN_PIN = A0; // -pol cap

const int OUT_PIN_2 = A3; // +pol cap
const int IN_PIN_2 = A2; // -pol cap


const float IN_STRAY_CAP_TO_GND = 24.48;
const float IN_CAP_TO_GND  = IN_STRAY_CAP_TO_GND;
const float R_PULLUP = 34.8;
const int MAX_ADC_VALUE = 1023;
int val;

//mittelwert
int mitte = 0;
int mittel_anz = 12;
int mittel_count = 0;
volatile int MittelWert_1;
volatile int MittelWert_2;

int kanal = 0;

//poti für Feuchtigkeitsgrenze
int sensorValue;
float voltage;
int sensorValue_2;
float voltage_2;
//ports poti
const int pot = 10; //is now a switch
const int pot_2 = 9; //is now a switch
const int led_ep = 8;


//werte im eeprom
#include <EEPROM.h>
// start reading from the first byte (address 0) of the EEPROM
int address = 0;
byte value;
byte hi;
byte lo;

// out motor

/*L298N H-Bridge driving DC motor on Arduino
*/

int ENA = 3; // MCU PWM Pin 10 to ENA on L298n Board
int IN1 = 2;  // MCU Digital Pin 9 to IN1 on L298n Board
int IN2 = 4;  // MCU Digital Pin 8 to IN2 on L298n Board


int IN3 = 7;  // MCU Digital pin 7 to IN3 on L298n Board
int IN4 = 6;  // MCU Digital pin 6 to IN4 on L298n Board
int ENB = 5;  // MCU PWM Pin 5 to ENB on L298n Board

// this is for interupthandling
const byte interruptPin = 0;

volatile boolean flag;

// Interrupt Service Routine (ISR)
void isr ()
{
  flag = true;
}  // end of isr

void setup()
{
  pinMode(OUT_PIN, OUTPUT);
  pinMode(IN_PIN, OUTPUT);
  pinMode(OUT_PIN_2, OUTPUT);
  pinMode(IN_PIN_2, OUTPUT);


  pinMode(ENA, OUTPUT); //Set all the L298n Pin to output
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(led_ep, OUTPUT);

  pinMode(pot, INPUT_PULLUP);
  pinMode(pot_2, INPUT_PULLUP);
  //digitalWrite(pot, HIGH);

  INIT_SERIAL(9600);
  //Serial.begin(9600);


  //pinMode(interruptPin, INPUT_PULLUP);
  //attachInterrupt (digitalPinToInterrupt (1), isr, CHANGE);  // attach interrupt handler
}



void DRIVEONE()
{
  // Run the motors on both direction at fixed speed
  digitalWrite(IN1, HIGH); // Turn HIGH motor A
  digitalWrite(IN2, LOW);
  // analogWrite(ENA, 200); // TO set the turning speed to 200 out of possible range 0 to 255
  for (int x = 0; x < 256; x++)   // Motor will accelerate from 0 to maximum speed
  {
    analogWrite(ENA, x);
    //analogWrite(ENB, x);
    delay(1);
  }


  //digitalWrite(IN3, HIGH); // turn HIGH motor B
  //digitalWrite(IN4, LOW);  // To set the turning speed to 200 out of possible range 0 to 255
  //analogWrite(ENB, 200);
  if (debug == true)
  {
    delay(2000);
  }
  else
  {
    delay(20000);  // Delay to 2 seconds
  }
  // Changing the direction of the motor
  for (int y = 255; y >= 0; --y)  // Motor will decelerate from maximum speed to 0
  {
    analogWrite(ENA, y);
    //analogWrite(ENB, y);
    delay(1);
  }
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  //digitalWrite(IN3, LOW);
  //digitalWrite(IN4, HIGH);
  delay(2); // Delay to 2 seconds

  digitalWrite(IN1, LOW); // Turn the motor off
  digitalWrite(IN2, LOW); // Turn the motor off
  digitalWrite(IN3, LOW); // Turn the motor off
  digitalWrite(IN4, LOW); // Turn the motor off
}
void DRIVETWO()
{
  /*
  	These function will turn the motors on the possible speeds, the maximum speed turns is determined
   	by the motor specs and the operating voltage. The PWM(Pulse with modulation values will sent
   	by analogWrite() function to drive the maxim speed.
  */

  //digitalWrite(IN1, LOW);
  //digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  for (int x = 0; x < 256; x++)   // Motor will accelerate from 0 to maximum speed
  {
    //analogWrite(ENA, x);
    analogWrite(ENB, x);
    delay(1);
  }
  if (debug == true)
  {
    delay(2000);
  }
  else
  {
    delay(20000);  // Delay to 20 seconds
  }
  for (int y = 255; y >= 0; --y)  // Motor will decelerate from maximum speed to 0
  {
    //analogWrite(ENA, y);
    analogWrite(ENB, y);
    delay(2);
  }

  digitalWrite(IN1, LOW); //  Tun Off All the Motors
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
void kanal_1()
{
  pinMode(IN_PIN, INPUT);
  digitalWrite(OUT_PIN, HIGH);
  val = analogRead(IN_PIN);
  digitalWrite(OUT_PIN, LOW);

  if (val < 1000)
  {
    pinMode(IN_PIN, OUTPUT);

    float capacitance = (float)val * IN_CAP_TO_GND / (float)(MAX_ADC_VALUE - val);
#ifdef DEBUG_off
    Serial.print(mittel_count);
    Serial.print(F(" Capacitance Value = "));
    Serial.print(capacitance, 3);
    Serial.print(F(" pF ("));
    Serial.print(val);
    Serial.println(F(") "));
#endif
  }
  else
  {
    pinMode(IN_PIN, OUTPUT);
    delay(1);
    pinMode(OUT_PIN, INPUT_PULLUP);
    unsigned long u1 = micros();
    unsigned long t;
    int digVal;

    do
    {
      digVal = digitalRead(OUT_PIN);
      unsigned long u2 = micros();
      t = u2 > u1 ? u2 - u1 : u1 - u2;
    }
    while ((digVal < 1) && (t < 400000L));

    pinMode(OUT_PIN, INPUT);
    val = analogRead(OUT_PIN);
    digitalWrite(IN_PIN, HIGH);
    int dischargeTime = (int)(t / 1000L) * 5;
    delay(dischargeTime);
    pinMode(OUT_PIN, OUTPUT);
    digitalWrite(OUT_PIN, LOW);
    digitalWrite(IN_PIN, LOW);

    float capacitance = -(float)t / R_PULLUP
                        / log(1.0 - (float)val / (float)MAX_ADC_VALUE);
#ifdef DEBUG_off
    Serial.print(F("Capacitance Value = "));
    if (capacitance > 1000.0)
    {
      Serial.print(capacitance / 1000.0, 2);
      Serial.print(F(" uF"));
    }
    else
    {
      Serial.print(capacitance, 2);
      Serial.print(F(" nF"));
    }

    Serial.print(F(" ("));
    Serial.print(digVal == 1 ? F("Normal") : F("HighVal"));
    Serial.print(F(", t= "));
    Serial.print(t);
    Serial.print(F(" us, ADC= "));
    Serial.print(val);
    Serial.println(F(")"));
#endif
  }



  while (millis() % 1000 != 0)
    ;




  //mittelwert int mitte = 0; int mittel_anz = 12; int mittel_count = 0;
  if  ( (mittel_count != 0) &&  (mittel_count < mittel_anz))
  {
    mitte = mitte + val;
    mittel_count++;
  }
  else if (mittel_count == 0)
  {
#ifdef DEBUG_off
    Serial.println(F(" value not used "));
#endif
    mittel_count++;
  }
  else
  {
    mitte = mitte + val;
    //mittel_count++;
    mitte = mitte / (mittel_count);
    MittelWert_1 = mitte;
#ifdef DEBUG
    Serial.print(F("Mittelwert = "));
    Serial.print(mitte);
    Serial.print(F(" bei "));
    Serial.print(mittel_count);
    Serial.println(F(" Messungen "));
#endif
    //feuchtigkeitsgrenze per poti einlesen
    // read the input on analog pin 0:
    //sensorValue = analogRead(pot);
    if (digitalRead(pot) == LOW)
    {
#ifdef DEBUG
      Serial.println(F(" Messungen schalter = LOW"));

#endif
      //val_ep = MittelWert_1;
      // Store the new value
      EEPROM.write(0, lowByte(MittelWert_1));

      EEPROM.write(1, highByte(MittelWert_1));

      byte state = LOW;
      while (digitalRead(pot) == LOW)
      {
        if (state == LOW)
        {
          digitalWrite(led_ep, state);
          delay(1000);
        }
        else
        {
          digitalWrite(led_ep, state);
          delay(1000);
        }
        state = !state;
      }

#ifdef DEBUG_off
      Serial.println(F("start mit eeprom "));
#endif
    }
    else
    {
#ifdef DEBUG_off
      Serial.println(F(" Messungen schalter = HIGH"));
#endif
    }
    digitalWrite(led_ep, LOW);
    // read a byte from the current address of the EEPROM
    //sensorValue = EEPROM.read(0);

    lo = EEPROM.read(0);
    hi = EEPROM.read(1);
    // Reassemble the integer
    sensorValue = word(hi, lo);



#ifdef DEBUG_off
    // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
    voltage = sensorValue * (5.0 / 1023.0);
    // print out the value you read:

    Serial.print(F("kanal 1 Poti Voltage = "));
    Serial.println(voltage);
#endif
#ifdef DEBUG    
    Serial.print(F("kanal 1  Schaltgrenze sensorValue = "));
    Serial.println(sensorValue);
#endif
    if (mitte <= sensorValue )
    {
      // anschalten Pumpe
#ifdef DEBUG
      Serial.println(F("kanal 1 Pumpe an = rechts"));
#endif
      DRIVEONE();
      //digitalWrite(motor, HIGH);
      //delay(10000);
      // ausschalten Pumpe
      //digitalWrite(motor, LOW);

    }

    //werte zurücksetzen
    mitte = 0;
    mittel_count = 0;

  }
}

void kanal_2()
{
  pinMode(IN_PIN_2, INPUT);
  digitalWrite(OUT_PIN_2, HIGH);
  val = analogRead(IN_PIN_2);
  digitalWrite(OUT_PIN_2, LOW);

  if (val < 1000)
  {
    pinMode(IN_PIN_2, OUTPUT);

    float capacitance = (float)val * IN_CAP_TO_GND / (float)(MAX_ADC_VALUE - val);
#ifdef DEBUG_off
    Serial.print(mittel_count);
    Serial.print(F(" Capacitance Value_2 = "));
    Serial.print(capacitance, 3);
    Serial.print(F(" pF ("));
    Serial.print(val);
    Serial.println(F(") "));
#endif
  }
  else
  {
    pinMode(IN_PIN_2, OUTPUT);
    delay(1);
    pinMode(OUT_PIN_2, INPUT_PULLUP);
    unsigned long u1 = micros();
    unsigned long t;
    int digVal;

    do
    {
      digVal = digitalRead(OUT_PIN_2);
      unsigned long u2 = micros();
      t = u2 > u1 ? u2 - u1 : u1 - u2;
    }
    while ((digVal < 1) && (t < 400000L));

    pinMode(OUT_PIN_2, INPUT);
    val = analogRead(OUT_PIN_2);
    digitalWrite(IN_PIN_2, HIGH);
    int dischargeTime = (int)(t / 1000L) * 5;
    delay(dischargeTime);
    pinMode(OUT_PIN_2, OUTPUT);
    digitalWrite(OUT_PIN_2, LOW);
    digitalWrite(IN_PIN_2, LOW);

    float capacitance = -(float)t / R_PULLUP
                        / log(1.0 - (float)val / (float)MAX_ADC_VALUE);

#ifdef DEBUG_off
    Serial.print(F("Capacitance Value_2 = "));
    if (capacitance > 1000.0)
    {
      Serial.print(capacitance / 1000.0, 2);
      Serial.print(F(" uF"));
    }
    else
    {
      Serial.print(capacitance, 2);
      Serial.print(F(" nF"));
    }

    Serial.print(F(" ("));
    Serial.print(digVal == 1 ? F("Normal") : F("HighVal"));
    Serial.print(F(", t= "));
    Serial.print(t);
    Serial.print(F(" us, ADC= "));
    Serial.print(val);
    Serial.println(F(")"));
#endif
  }



  while (millis() % 1000 != 0)
    ;




  //mittelwert int mitte = 0; int mittel_anz = 12; int mittel_count = 0;
  if  ( (mittel_count != 0) &&  (mittel_count < mittel_anz))
  {
    mitte = mitte + val;
    mittel_count++;
  }
  else if (mittel_count == 0)
  {
#ifdef DEBUG_off
    Serial.println(F(" value not used "));
#endif
    mittel_count++;
  }
  else
  {
    mitte = mitte + val;
    //mittel_count++;
    mitte = mitte / (mittel_count);
    MittelWert_2 = mitte;
#ifdef DEBUG
    Serial.print(F("Mittelwert = "));
    Serial.print(mitte);
    Serial.print(F(" bei "));
    Serial.print(mittel_count);
    Serial.println(F(" Messungen "));
#endif

    //feuchtigkeitsgrenze
    if (digitalRead(pot_2) == LOW)
    {
#ifdef DEBUG
      Serial.println(F(" Messungen schalter = LOW"));

#endif
      //val_ep = MittelWert_2;
      // Store the new value
      EEPROM.write(2, lowByte(MittelWert_2));

      EEPROM.write(3, highByte(MittelWert_2));

      byte state = LOW;
      while (digitalRead(pot_2) == LOW)
      {
        if (state == LOW)
        {
          digitalWrite(led_ep, state);
          delay(1000);
#ifdef DEBUG
          Serial.print(F("pot_2 = "));
          Serial.println(digitalRead(pot_2));
#endif
        }
        else
        {
          digitalWrite(led_ep, state);
          delay(1000);
#ifdef DEBUG
          Serial.print(F("pot_2 = "));
          Serial.println(digitalRead(pot_2));
#endif
        }
        state = !state;
      }
#ifdef DEBUG_off
      Serial.println(F("start mit eeprom "));
#endif
    }
    else
    {
#ifdef DEBUG_off
      Serial.println(F(" Messungen schalter = HIGH"));
#endif
    }

    // read a byte from the current address of the EEPROM
    //sensorValue = EEPROM.read(0);
    digitalWrite(led_ep, LOW);
    lo = EEPROM.read(2);
    hi = EEPROM.read(3);
    // Reassemble the integer
    sensorValue_2 = word(hi, lo);




#ifdef DEBUG_off
    // Convert the analog reading (which goes from 0 - 1023) to a voltage_2 (0 - 5V):
    voltage_2 = sensorValue_2 * (5.0 / 1023.0);
    // print out the value you read:

    Serial.print(F("Poti voltage_2 = "));
    Serial.println(voltage_2);
#endif
#ifdef DEBUG
    Serial.print(F("Kanal 2 Schaltgrenze sensorValue_2 = "));
    Serial.println(sensorValue_2);
#endif
    if (mitte <= sensorValue_2 )
    {
      // anschalten Pumpe
#ifdef DEBUG
      Serial.println(F("Pumpe an = links"));
#endif
      DRIVETWO();

    }

    //werte zurücksetzen
    mitte = 0;
    mittel_count = 0;

  }
}
void loop()
{

  int val_ep = 0;
#ifdef DEBUG
  //EEPROM.get(0, val_ep);
  // Read the stored values

  lo = EEPROM.read(0);
  hi = EEPROM.read(1);
  // Reassemble the integer
  val_ep = word(hi, lo);

  //Serial.print(F("int eeprom = "));
  //Serial.println(val_ep);

  if (val_ep == -1)
  {
    val_ep = 854;
    // Store the new value
    EEPROM.write(0, lowByte(val_ep));

    EEPROM.write(1, highByte(val_ep));

    Serial.println(F("start mit eeprom "));

  }
#endif
  delay(1);
  if (kanal < 14)
  {
#ifdef DEBUG_off
    Serial.print(F("Messung 1 "));
    Serial.println(kanal);
#endif
    kanal_1();
    kanal++;
  }
  else if ((kanal <= 26) && (kanal >= 14))
  {
#ifdef DEBUG_off
    Serial.print(F("Messung 2 "));
    Serial.println(kanal);
#endif
    kanal_2();
    kanal++;
  }

  // beginn mit kanal 2
  else if (kanal >= 29)
  {
#ifdef DEBUG_off
    Serial.println(F("Ende der ersten Messung "));
#endif
    kanal = 0;
    mitte = 0;
    mittel_count = 0;

#ifdef DEBUG_off
  delay(1);
#else
  delay(60000);
#endif
  }
  else
  {

    kanal++;
  }
#ifdef DEBUG_off
  Serial.print(F("Flag = "));//flag
  Serial.println(flag);
#endif
  





}











