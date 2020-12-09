/*
  Sensor Logger
  - Senses the Current(mA), Voltage(mV) and Power(mW) of the ina260 modules
  - prints data on to SD card.

  Controlling OpenLog command line from an Arduino

      For INA260 I2C connection
        [SCL][SDA][AREF][GND][PIN13]
          X    X

  Connect the following OpenLog to Arduino:
    TXO of OpenLog to RX of the Arduino
    RXI to TX
    GRN to 2
    VCC to 5V
    GND to GND

      NOTE: When uploading this example code you must temporarily disconnect TX and RX while uploading
  the new code to the Arduino. Otherwise you will get a "avrdude: stk500_getsync(): not in sync" error.

  This example code assumes the OpenLog is set to operate at 9600bps in NewLog mode, meaning OpenLog
  should power up and output '12<'. This code then sends the three escape characters and then sends
  the commands to create a new random file called nate###.txt where ### is a random number from 0 to 999.

  This code assume OpenLog is in the default state of 9600bps with ASCII-26 as the esacape character.

  Be careful when sending commands to OpenLog. println() sends extra newline characters that
  cause problems with the command parser. The new v2.51 ignores \n commands so it should be easier to
  talk to on the command prompt level. This example code works with all OpenLog v2 and higher.

  There are two status LEDs on the OpenLog to help you with troubleshooting.

  STAT1 - This blue indicator LED is attached to Arduino D5 (ATmega328 PD5) and toggles on/off when a new character is received.
  This LED blinks when Serial communication is functioning.
  STAT2 - This green LED is connected to Arduino D13 (SPI Serial Clock Line/ ATmega328 PB5).
  This LED only blinks when the SPI interface is active. You will see it flash when the OpenLog records 512 bytes to the microSD card.

  How to use:
  1. Upload code
  2. Wire up and put in SD card
  3  Press arduino Reset
  4. when completed remove SD card


*/
#include <Adafruit_INA260.h>

Adafruit_INA260 ina_out = Adafruit_INA260();//solar panel output
Adafruit_INA260 ina_in = Adafruit_INA260();//components(launchpad motor) input

char buff[50];
int fileNumber;

int statLED = 13;
int resetOpenLog = 2;

void setup() {
  //OpenLog Setup*****************************************
  pinMode(statLED, OUTPUT);
  pinMode(resetOpenLog, OUTPUT);

  digitalWrite(statLED, HIGH);

  Serial.begin(9600);

  //Reset OpenLog
  digitalWrite(resetOpenLog, LOW);
  delay(100);
  digitalWrite(resetOpenLog, HIGH);


  //Wait for OpenLog to respond with '<' to indicate it is alive and recording to a file
  while (1) {
    if (Serial.available())
      if (Serial.read() == '<') break;
  }

  //Send three control z to enter OpenLog command mode
  Serial.write(26);
  Serial.write(26);
  Serial.write(26);

  //Wait for OpenLog to respond with '>' to indicate we are in command mode
  while (1) {
    if (Serial.available())
      if (Serial.read() == '>') break;
  }

  //Creating and editing the file
  sprintf(buff, "append Data.csv\r", 1);
  Serial.print(buff); //\r in string + regular print works with older v2.5 Openlogs

  //Wait for OpenLog to indicate file is open and ready for writing
  while (1) {
    if (Serial.available())
      if (Serial.read() == '<') break;
  }

  //Sensor Setup **************************

  /*
    //Serial.begin(115200);//delete
    // Wait until serial port is opened
    Serial.begin(9600);
    while (!Serial) {
    delay(10);
    }
  */
  //Serial.println("Adafruit INA260 Test");
  if (!ina_out.begin(65)) {//65 is address for ina_out, has solder pad used for solar panel output
    //check for sensor chip
    //Serial.println("Couldn't find INA260 chip");
    //while (1);
  }

  if (!ina_in.begin(64)) {//66 is address for INA_IN, no solder pad used for buck converter output
    //check for sensor chip
    //Serial.println("Couldn't find INA260 chip");
    //while (1);
  }
  //Serial.println("Found INA260 chips");

  digitalWrite(statLED, LOW);
  Serial.println("Iout (mA),Vout (V),Pout (W), Iin (mA),Vin (V),Pin (W)");
  //use V and W instead of mV and mW becasue of data logging cap
}


void loop() {

  float Iout = 0, Vout = 0, Pout = 0, Iin = 0, Vin = 0, Pin = 0;

  for (int i = 0; i < 30; i++) {
    Iout += ina_out.readCurrent();
    Vout += ina_out.readBusVoltage();
    Pout += ina_out.readPower();
    Iin += ina_in.readCurrent();
    Vin += ina_in.readBusVoltage();
    Pin += ina_in.readPower();
    delay(2000);
  }

  Iout /= 30;
  Vout /= (30 * 1000);
  Pout /= (30 * 1000);
  Iin /= 30;
  Vin /= (30 * 1000);
  Pin /= (30 * 1000);

  Serial.print(Iout);
  Serial.print(",");
  Serial.print(Vout);
  Serial.print(",");
  Serial.print(Pout);
  Serial.print(",");
  Serial.print(Iin);
  Serial.print(",");
  Serial.print(Vin);
  Serial.print(",");
  Serial.print(Pin);
  Serial.println();
}
