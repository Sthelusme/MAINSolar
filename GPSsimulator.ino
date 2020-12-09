/*
  This code is used to send

  The circuit:
   RX is digital pin 10 (connect to TX of other device)
   TX is digital pin 11 (connect to RX of other device)

  Note:
  Not all pins on the Mega and Mega 2560 support change interrupts,
  so only the following can be used for RX:
  10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69

  Not all pins on the Leonardo and Micro support change interrupts,
  so only the following can be used for RX:
  8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).

  //$GPRMC,203722.000,A,3356.2931,N,08431.3669,W,0.30,280.05,081020,,,D*76
  //0         1         2         3         4         5         6
  //0123456789012345678901234567890123456789012345678901234567890123456789
  //code  time         lat+d       long+d        speedangle  date

*/
#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // RX, TX
//char test[] = "$GPRMC,203722.000,A,3356.2931,N,08431.3669,W,0.30,280.05,081020,,,D*76\n";

  char test[]="$GPGGA,203203.000,3356.3184,N,08431.3670,W,2,06,1.42,292.0,M,-30.8,M,0000,0000*52\n"
"$GPGSA,A,3,18,25,31,32,20,23,,,,,,,1.71,1.42,0.95*00\n"
"$GPRMC,100000.000,A,3356.3184,N,08431.3670,W,0.42,109.14,150620,,,D*74\n"
"$GPVTG,109.14,T,,M,0.42,N,0.77,K,D*33\n";
/*
 * Time::14
 * Timezone change:5
 * Date:06/15/2020
 * Location : Kennesaw State
 * Azimuth:196.54
 * Zenith:90-76=24
*/
int i;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  mySerial.begin(9600);
  i = 0;
}

void loop() { // run over and over

  mySerial.print(test[i]);
  Serial.print(test[i]);
  delay(10);
  i++;
  if (test[i] == '\0') {
    mySerial.print('\0');
    Serial.print('\0');
    i = 0;
  }


  //mySerial.println(test);
}
