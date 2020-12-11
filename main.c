/************************************************************************
  Dual Axis Solar Tracking Controller for solar panels with GPS and LDR
  Last updated 12-11-2020
  Developed and written by:
      Danielle Cranston
      Mary Caroline Dowd
      Stephen Thelusme
      Andre Sands



  TASK SUMMARY
Choose mode based on switch
    GPS - Global Positioning
        Import GPS values
        put values in new array
        parse GPS VARIBLES
        calculate Azimuth, Zenith angles - finds suns position and represents in angles
        read in potentiometer - calculate panel position
        Calculate motor movement - finds time to move the motor
        move motors
        check threshold - Go back up to Sensor comparison if doesn't reach threshold or number of tries
    LDR - Light Dependent Resistors
L       Light sensor setup
        sensor comparison
        move motors
        check threshold - Go back up to Sensor comparison if doesn't reach threshold or number of times
   Sleep mode
       turn off all pins
       set up RTC - Real time Clock
       Goes to LPM3
       wake up in 20 mins

Devices:
    MSP432P401
    Adafruit Ultimate GPS
    L298N DC motor driver
    4 LDR
    4 2K  Ohm resistors\
    LM2596
    2 POT
    RNG-50D Renogy Solar panel

PIN---------Function
P5.5  |<--- A0 (Analog Input) LDR 0, NE              sensorBuffer[0]
P5.4  |<--- A1 (Analog Input) LDR 1, NW              sensorBuffer[1]
P5.2  |<--- A3 (Analog Input) LDR 3, SE              sensorBuffer[3]
P5.1  |<--- A4 (Analog Input) Potentiometer NS       sensorBuffer[4]
P5.0  |<--- A5 (Analog Input) Potentiometer EW       sensorBuffer[5]
P4.7  |<----A6 (Analog Input) LDR 2, SW              sensorBuffer[2]

P3.2  |<----RX(TX from device)
P3.3  |<----TX(RX from device)
P4.1  |<----GPS enable

P1.2  |<----UART:pc (not a pin, USB cord)
P1.3  |<----UART:pc (not a pin, USB cord)
P2.0  |<----RedLED
P2.1  |<----GreenLED
P2.2  |<----BlueLED

P3.5  |<----Mode toggle switch
P3.6  |<----Sensor enable
P3.7  |<----Pot enable

POTNS
    G- MC
    BR- GND
    BL- VOLTAGE


POTEW
    BL - POWER
    G - MC
    BR - GND

IN 1 ----P2.7--------North-----NS+
IN 2 ----P2.6--------South-----NS-
IN 3 ----P2.4--------East------EW+
IN 4 ----P5.6--------West------EW-
++ contract motor
+- extend motor




 ****************************************************************************/
/*
//psuedo GPS sentence for testing
 *
 char gpsdata[] = {'$','G','P','R','M','C',',','2','2','1','5','0','3','.','0','0','0',',','A',',','3','3','5','6','.','3','1','8','4',',','N',',','0','8','4','3','1',
                                   '.','3','6','7','0',',','W',',','0','.','4','2',',','1','0','9','.','1','4',',','2','0','1','0','2','0'};
 *$GPGGA,203203.000,3356.3184,N,08431.3670,W,2,06,1.42,292.0,M,-30.8,M,0000,0000*52
 *$GPRMC,203203.000,A,3356.3184,N,08431.3670,W,0.42,109.14,151020,,,D*74
 *01234567890123456789012345678901234567890123456789012345678901234567890123456789012345
 *0         1         2         3         4         5         6         7         8

 */

/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <time.h>

/*GPSmode
 * true= GPS mode
 * false = LDR mode
 * if Hybrid mode= true, it will change GPSmode
 * */
bool Hybridmode=false;
bool GPSmode=true;
int sleep_time=20;


/* GLOBAL VARIABLES */
// where deg refers to degrees and rad is radians
// parsing variables
int century = 2000; // 21st century
float hour = 7, min, sec;// current time
float latdeg, latmin, lat_min_decimal, latitude_deg, lat_rad;//latitude value holder
float longdeg, longmin, long_min_decimal, longitude_deg;// longitude value holder
int day, month, year;// the current day, month, year
int timezone = -5; // from UTC
const float PI = 3.14159265;
double hour_angle_deg, hour_angle_rad;// angle of the sun determined from the hour
float solar_decl_angle_deg,solar_decl_angle_rad; //the solar declination angle
float altitude_angle_deg;// for solar tilt
float Azimuth;
float a,b,d,e,f,g,h;
float fract_year;
float zenith_deg, zenith_rad;
char conv_buffer[4];
int daynumber;
int daysToMonth[2][12] =
{  //days from jan1, where row 0 is non-leap year, row 1 is leap year
   { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 },
   { 0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335 },
};
float motorNS_speed=.0101,motorEW_speed=.0101;//speed when plugged in to battery
//float motorNS_speed=.05,motorEWspeed=.04375;//speed when 10V

//GPS data storing variables
char rxBuffer[800];//all data received from GPS
char gpsdata[80];//sentence that we are looking at

/*
//psuedo GPS sentence for testing
char gpsdata[] = {'$','G','P','R','M','C',',','1','3','3','0','0','0','.','0','0','0',',','A',',','3','3','0','0','.','0','0','0','0',',','N',',','0','8','4','0','0',
 '.','0','0','0','0',',','W',',','0','.','4','2',',','1','0','9','.','1','4',',','2','0','1','1','2','0'};
 */

uint16_t rxWriteIndex=0;//index used to read rxBuffer
uint8_t data;
static uint16_t sensorBuffer[6]; //buffer that contains sensor data
bool fix=false;//GPS satalite fix

//delay part

void delay(int wait_time_ms){
    const int mtime= 3000000/1000;//clock speed /10^3 - clock speed in miliseconds
    for(;wait_time_ms>0;wait_time_ms--){
        __delay_cycles(mtime);
    }
    return;
}

//function declarations
int convert_toHundreds(char hundreds, char tens, char ones);//converts character arrays into int numbers
bool is_leap_year(void);// checks if there is a leap year
void calc_azimuth_zenith(void);//calculates the zenith and azimuth angle
void parseGPS(void);// collect GPS data and convert to proper data types values
void readGPS();// will pull correct sentence from rxBuffer
void read_sensors(void);//read inputs from all sensors
void moveN(int time);//retract the NS motor turning the normal of the panel towards the north
void moveS(int time);//extends the NS motor turning the normal of the panel towards the south
void moveE(int time);//retract the EW motor turning the normal of the panel towards the east
void moveW(int time);//extends the EW motor turning the normal of the panel towards the west
void compare_LDR(void);// compare LDR readings to move motors
void gotosleep(int time);//makes the motor sleep for minutes
void GPS_align_panel(void); //move panels to align with calculated sun position
void LED_indicator(char color);//LED Function
char LED_color='o';
char LED_old='o';

/*Configuration setup*/
const eUSCI_UART_ConfigV1 uartConfig = //UART configuration settings.
{
 EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
 78,                                     // BRDIV = 78
 2,                                       // UCxBRF = 2
 0,                                       // UCxBRS = 0
 EUSCI_A_UART_NO_PARITY,                  // No Parity
 EUSCI_A_UART_LSB_FIRST,                  // LSB First
 EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
 EUSCI_A_UART_MODE,                       // UART mode
 EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION // Oversampling
 // EUSCI_A_UART_8_BIT_LEN                  // 8 bit data length
};


RTC_C_Calendar calendarTime ={// Sleep mode calendar setup
                              0,     /* Seconds */
                              0,     /* Minutes */
                              0,     /* Hour */
                              1,     /* Day of Week */
                              1,     /* Day */
                              1,     /* Month */
                              2020   /* Year */
};

/*************************************************************************************************************************************************************/
int main(void)
{
    //setup************************************************************

    while(1){
        /* Halting WDT  */
        WDT_A_holdTimer();
        Interrupt_disableSleepOnIsrExit();
        /* Setting DCO to 12MHz */
        CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);
        /* Configuring UART Module */
        UART_initModule(EUSCI_A2_BASE, &uartConfig);
        UART_initModule(EUSCI_A0_BASE, &uartConfig);
        /* Setting reference voltage to 2.5  and enabling reference */
        REF_A_setReferenceVoltage(REF_A_VREF2_5V);
        REF_A_enableReferenceVoltage();
        //Motor pin setup
        GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN7); //North
        GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN6); //South
        GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN4); //East
        GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN6); //West
        //GPS enable
        GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN1);//GPS ENABLE
        //Sensor enable pin
        GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN6);//Powers LDR
        GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN7);//Powers POT
        Interrupt_enableMaster();
        LED_indicator('g');//light turns green to signal on
        /* Enabling interrupts */
        Interrupt_enableInterrupt(INT_EUSCIA2); //Enable Interrupt for clock
        //Setup: Sensors Analog Digital Converter Input******************************************************
        /* Initializing ADC (MCLK/1/1) */
        ADC14_enableModule();
        ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1,
                         0);
        /* Configuring GPIOs for Analog In */
        GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5,
                                                   GPIO_PIN5 | GPIO_PIN4 | GPIO_PIN2 | GPIO_PIN1
                                                   | GPIO_PIN0, GPIO_TERTIARY_MODULE_FUNCTION);
        GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4,
                                                   GPIO_PIN7, GPIO_TERTIARY_MODULE_FUNCTION);
        /*Set up input pin for mode switching*/
        GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P3,GPIO_PIN5);
        /* Configuring ADC Memory (ADC_MEM0 - ADC_MEM7 (A0 - A7)  with no repeat)
         * with internal 2.5v reference */
        ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM5, false);
        ADC14_configureConversionMemory(ADC_MEM0,
                                        ADC_VREFPOS_INTBUF_VREFNEG_VSS,
                                        ADC_INPUT_A0, false);
        ADC14_configureConversionMemory(ADC_MEM1,
                                        ADC_VREFPOS_INTBUF_VREFNEG_VSS,
                                        ADC_INPUT_A1, false);
        //P4.7 will be mapped to MEM2 or sensor buffer[2]
        ADC14_configureConversionMemory(ADC_MEM2,
                                        ADC_VREFPOS_INTBUF_VREFNEG_VSS,
                                        ADC_INPUT_A6, false);
        ADC14_configureConversionMemory(ADC_MEM3,
                                        ADC_VREFPOS_INTBUF_VREFNEG_VSS,
                                        ADC_INPUT_A3, false);
        ADC14_configureConversionMemory(ADC_MEM4,
                                        ADC_VREFPOS_INTBUF_VREFNEG_VSS,
                                        ADC_INPUT_A4, false);
        ADC14_configureConversionMemory(ADC_MEM5,
                                        ADC_VREFPOS_INTBUF_VREFNEG_VSS,
                                        ADC_INPUT_A5, false);
        //sets motors to low
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7); //North
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN6); //South
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN4); //East
        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN6); //West

        fflush(stdout);
        printf("Setup completed!\n");
        fflush(stdout);

        //Moving phase************************************************************

        /*
        //Hybrid mode - checks ldr for brightness to determine mode
        if (Hybridmode){
            read_sensors();
            int temp = fmin(sensorBuffer[0],sensorBuffer[1]);//finds the minimum value
            int minVal = fmin(sensorBuffer[2],sensorBuffer[3]);
            minVal = fmin(temp,minVal);
            if (minVal>9000){// finds the max value and check if it has enough sunlight for GPS mode
                GPSmode=false;//turns on LDR mode
            }
            else{
                GPSmode=true;// make it turn to GPS mode
            }
        }
         */

        //mode toggle switch
        read_sensors();// this will set the mode based on the switch
        if(GPSmode){
            //GPS mode
            fflush(stdout);
            printf("GPS mode\n");
            fflush(stdout);
            LED_indicator('c');
            fix=false;
            rxWriteIndex=0;
            //SETUP GPS READ functionality**********************************
            /* Selecting P3.2 and P3.3 in UART mode
             * this is for communication to the GPS*/
            GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,
                                                       GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
            //GPS fix pin setup
            /* Select P1.2 and P1.3 in UART mode
             * this is for the communication to the computer if debugging GPS input
             * */
            GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
                                                       GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
            /* Enable UART module */
            UART_enableModule(EUSCI_A2_BASE);//ENABLE UART interrupt for GPS UART
            UART_enableModule(EUSCI_A0_BASE);//ENABLE UART interrupt for Computer UART
            UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT); // enable the interrupt for the UART GPS reading
            GPIO_setOutputHighOnPin(GPIO_PORT_P4,GPIO_PIN1);//turns on GPS, GPS Enable
            //GPS fix check
            //UART_disableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT); // disable the interrupt for the UART GPS reading
            do{//checks for active data
                //waits for the GPS buffer to fill up
                while(rxWriteIndex<800){
                }
                readGPS(); // first time reading on startup - will read from rxBuffer into gpsdata[]
                if (gpsdata[18]=='A'){//If GPS is active, continue
                    fix=true;
                    UART_disableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT); // disable the interrupt for the UART GPS reading
                    //fflush(stdout);
                    //printf("GPS Active\n");
                    //fflush(stdout);
                }
                else
                {
                    fix=false;
                    rxWriteIndex=0;
                    //fflush(stdout);
                    //printf("GPS Data Void\n");
                    //fflush(stdout);
                }
            }while(!fix);
            LED_indicator('b');
            parseGPS();   // collect GPS data and convert to proper data types values
            calc_azimuth_zenith();   //calculates the zenith and azimuth angle
            GPS_align_panel(); //move panels to align
        }
        else{
            // LDR mode
            fflush(stdout);
            printf("LDR mode\n");
            fflush(stdout);
            LED_indicator('g');
            read_sensors(); // read inputs from all sensors
            compare_LDR();  // compare LDR readings to move motors
            compare_LDR();  // compare LDR readings to move motors
        }
        //Sleep Mode***********************************************************************************************************
        gotosleep(sleep_time);  //makes the board sleep for minutes
    }
}

/***************************************************************************************************************************************************************/

/* EUSCI A0 UART ISR - Echoes data back to PC host */
void EUSCIA2_IRQHandler(void) //fill up the rxBuffer array
{
    //9600 Baud
    uint32_t status = UART_getEnabledInterruptStatus(EUSCI_A2_BASE);

    UART_clearInterruptFlag(EUSCI_A2_BASE,status);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        data=UART_receiveData(EUSCI_A2_BASE);
        rxBuffer[rxWriteIndex++]=data;
        //UART_transmitData(EUSCI_A0_BASE, data);
        // printf("%c",data);
        if(rxWriteIndex>=800){
            //Interrupt_disableMaster();// disables all the interrupts
            //PCM_gotoLPM0InterruptSafe();//allows board to go to sleep mode with interupt enable

        }
    }
}


void readGPS(){
    int k=0;
    int n=0;
    for(k = 0;k<=800;k++){
        if(rxBuffer[k]=='$'){
            if(rxBuffer[k+5]=='C'){
                while(rxBuffer[k]!='\0'){
                    gpsdata[n]=rxBuffer[k++];
                    //UART_transmitData(EUSCI_A0_BASE,);
                    n++;
                }
                k=801;
            }
        }

    }
}

int convert_toHundreds(char hundreds, char tens, char ones)
{
    //converts character arrays into int numbers
    int intOut = 0;
    conv_buffer[0] = hundreds;
    conv_buffer[1] = tens;
    conv_buffer[2] = ones;
    conv_buffer[3] = '.';

    intOut =atoi(conv_buffer);

    return intOut;
}

bool is_leap_year()
{
    // checks if there is a leap year
    //leap year condition
    if (year % 4 == 0)
    {
        if (year % 100 == 0)
        {
            if(year % 400 == 0)
                return true;
            else
                return false;
        }
        else
            return true;
    }
    else
        return false;
}



void calc_azimuth_zenith()
{
    /*this function finds:
     * day number,
     * hour angle,
     * solar declination angle of the sun
     * zenith
     * azimuth
     */

    //find the day number, hour angle, and solar declination angle of the sun
    daynumber = daysToMonth[is_leap_year()][month-1] + day; // calculate the daynumber of the year
    hour_angle_deg = 15.00*((hour + min/60.00) - 12.00);
    hour_angle_rad = hour_angle_deg*PI/180.00;
    fract_year = 360.00/(is_leap_year() ? 366.00 :365.00)*(284 + daynumber); // if it is a leap year use 366 else use 365
    solar_decl_angle_deg = 23.45 * sin(fract_year *PI/180);//find sine and convert to radians
    solar_decl_angle_rad = solar_decl_angle_deg * PI/180.00;

    //calculates the zenith and azimuth angle
    // zenith = asin(sin(gps_lat)*sin(SDA) + cos(gps_lat)*cos(SDA)*cos*(hour_angle))
    lat_rad = latitude_deg * (PI/180.00);
    a = sin(lat_rad); // where degrad is the gps lat in radian
    b = sin(solar_decl_angle_rad);
    d = cos(lat_rad);
    e = cos(solar_decl_angle_rad);
    f = cos(hour_angle_rad); // cos(Hradian)
    g = sin(hour_angle_rad); // sin(Hradian_
    zenith_rad = acos((a*b)+(d*e*f)); // acos((a*b)+(d*e*f)); measured from zenith(vertical)
    h=cos(zenith_rad);
    zenith_deg = zenith_rad * 180.00/PI;
    altitude_angle_deg = 90.00 - zenith_deg; // for solar tilt

    float abc=(((sin(lat_rad)*cos(zenith_rad))-sin(solar_decl_angle_rad))/(cos(lat_rad)*sin(zenith_rad)));
    float Azimuth_p1=acos((((sin(lat_rad)*cos(zenith_rad))-sin(solar_decl_angle_rad))/(cos(lat_rad)*sin(zenith_rad))))*180/PI;//in degrees
    if (hour_angle_deg>0){
        Azimuth=fmod(Azimuth_p1+180,360);
    }
    else{
        Azimuth=fmod(540-Azimuth_p1,360);
    }
}


void parseGPS()
{
    // collect GPS data and convert to proper data types values

    // pull out all the required information from the GPS data
    hour = convert_toHundreds('0',gpsdata[7],gpsdata[8]) + timezone;
    //hour=12;//used for testing
    min = convert_toHundreds('0',gpsdata[9],gpsdata[10]);
    sec = convert_toHundreds('0',gpsdata[11],gpsdata[12]);

    // Received data from GPS is in deg + min , but needs to be in degrees only
    // result_deg = deg + min/60
    latdeg = convert_toHundreds('0',gpsdata[20],gpsdata[21]);
    latmin = convert_toHundreds('0',gpsdata[22],gpsdata[23]);
    lat_min_decimal = convert_toHundreds(gpsdata[25],gpsdata[26],gpsdata[27])/1000.00 + convert_toHundreds('0','0',gpsdata[28])/10000.00;
    latitude_deg = latdeg + (latmin + lat_min_decimal)/60.00; // latitude in degrees = latdeg + latmin/60
    if (gpsdata[30] != 'N') // north is a positive value
    {latitude_deg = (-1)*latitude_deg;}

    longdeg = convert_toHundreds(gpsdata[32],gpsdata[33],gpsdata[34]);
    longmin = convert_toHundreds('0',gpsdata[35],gpsdata[36]);
    long_min_decimal = convert_toHundreds(gpsdata[38],gpsdata[39],gpsdata[40])/1000.00 + convert_toHundreds('0','0',gpsdata[41])/10000.00;
    longitude_deg = longdeg + (longmin + long_min_decimal)/60.00; // latitude in degrees = latdeg + latmin/60
    if (gpsdata[43] != 'E') // east is a positive value
    {longitude_deg = (-1)*longitude_deg;}

    int q=0;//size of gpsdata
    int comma_count=0;
    //month, day and year don't come out in the same position every time, this function finds them in the GPSdata array
    while(q<80){
        if (gpsdata[q]==','){
            comma_count++;
        }

        if (comma_count>=9){
            break;
        }
        q++;
    }

    day = convert_toHundreds('0',gpsdata[q+1],gpsdata[q+2]);
    month = convert_toHundreds('0',gpsdata[q+3],gpsdata[q+4]);
    year = convert_toHundreds('0',gpsdata[q+5],gpsdata[q+6])+ century;
}

void read_sensors(){
    //read inputs from all sensors

    //enable sensor pins
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6);//turns on the LDR
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7);//turns on the POT

    delay(250);
    /* Zero-filling buffer */
    memset(sensorBuffer, 0x00, 6 * sizeof(uint16_t));
    /* Setting up the sample timer to automatically step through the sequence
     * convert.
     */
    ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);

    /* Triggering the start of the sample */
    ADC14_enableConversion();
    ADC14_toggleConversionTrigger();

    //fill up array with results
    ADC14_getMultiSequenceResult(sensorBuffer);
    //turn off to save power
    ADC14_disableSampleTimer();
    ADC14_disableConversion();
    //read in mode toggle switch
    GPSmode=GPIO_getInputPinValue(GPIO_PORT_P3,GPIO_PIN5);
    //turns off sensor pins
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7);
}

void moveN(int time){
    //retract the NS motor turning the normal of the panel towards the north
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7); //North
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN6); //South
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN4); //East
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN6); //West

    delay(time);

    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7); //North
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN6); //South
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN4); //East
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN6); //West

}

void moveS(int time){
    //extends the NS motor turning the normal of the panel towards the south
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7); //North
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN6); //South
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN4); //East
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN6); //West

    delay(time);

    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7); //North
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN6); //South
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN4); //East
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN6); //West
}

void moveE(int time){
    //retract the EW motor turning the normal of the panel towards the east
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7); //North
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN6); //South
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN4); //East
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN6); //West

    delay(time);

    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7); //North
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN6); //South
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN4); //East
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN6); //West
}

void moveW(int time){
    //extends the EW motor turning the normal of the panel towards the west
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7); //North
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN6); //South
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN4); //East
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN6); //West

    delay(time);

    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7); //North
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN6); //South
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN4); //East
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN6); //West
}

void compare_LDR(void){
    // compare LDR readings to move motors
    int temp = 0;//value holder
    int maxVal = 0;//maximum value and sunlight
    int maxIdx = 0;//LDR sensor with most sun
    int light_tol=0;//amount of sunlight needed for operation
    int thresh = 10; // this is the tolerance between LDRs
    int adj_sensorNS = 0;//sensor that is north or south adjacent to max value
    int adj_sensorEW = 0;//sensor that is east or west adjacent to max value
    int move_time=15000; // the time allowed for motor movement
    int stop_countNS = 400;// Amount of of attempts the motor has to align
    int stop_countEW = 400;

    /*
    int sensorNE=sensorBuffer[0]; // LDR0
    int sensorNW=sensorBuffer[1]; // LDR1
    int sensorSW=sensorBuffer[2]; // LDR2
    int sensorSE=sensorBuffer[3]; // LDR3
     */

    // find the max value among the LDRs
    temp = fmax(sensorBuffer[0],sensorBuffer[1]); //
    maxVal = fmax(sensorBuffer[2],sensorBuffer[3]); //
    maxVal = fmax(temp,maxVal);
    int i = 0;
    for (;i<4;i++){ // get the index of where the max val occurs
        if (maxVal == sensorBuffer[i]){
            maxIdx = i;
            break;
        }
    }

    adj_sensorNS = 3 - maxIdx; //gets the north/south adjacent sensor

    // to get the east/west adjacent sensor
    switch(maxIdx){
    case 0: adj_sensorEW = 1; break;
    case 1: adj_sensorEW = 0; break;
    case 2: adj_sensorEW = 3; break;
    case 3: adj_sensorEW = 2; break;
    }

    bool moveenabled = true;
    // the higher the reading on a LDR means high intensity light
    while(moveenabled){
        // compare the max val to determined light tolerance
        if (maxVal > light_tol){
            //      if met, continue
            // compare highest value with the E/W adjacent pin
            LED_indicator('r');
            while((  sensorBuffer[maxIdx] - sensorBuffer[adj_sensorEW] > thresh)&& stop_countEW){
                if(maxIdx == 0 || maxIdx == 3 )
                    moveE(move_time); // tilt panel's east side downward
                else
                    moveW(move_time); // tilts upward
                stop_countEW--;
                read_sensors();
            }
            // compare highest value with the N/S adjacent pin
            while((  sensorBuffer[maxIdx] - sensorBuffer[adj_sensorNS]> thresh) && stop_countNS){
                if(maxIdx < 2)
                    moveN(move_time); // tilt panel's north side downward 1s
                else
                    moveS(move_time); // tilts upward
                stop_countNS--;
                read_sensors();
            }
            LED_indicator('g');
            moveenabled = false; // panel has been adjusted to LDR equalization
        }
    }
    fflush(stdout);
    printf("Alignment Complete\n");
    fflush(stdout);
}

void gotosleep(int time){
    // Sleep mode setup
    //makes the TI Device sleep for (time) minutes
    fflush(stdout);
    printf("Sleep Mode\n");
    fflush(stdout);

    /* Terminating all remaining pins to minimize power consumption. This is
            done by register accesses for simplicity and to minimize branching API
            calls */
    GPIO_setAsOutputPin(GPIO_PORT_P1, PIN_ALL16);
    GPIO_setAsOutputPin(GPIO_PORT_P2, PIN_ALL16);
    GPIO_setAsOutputPin(GPIO_PORT_P3, PIN_ALL16);
    GPIO_setAsOutputPin(GPIO_PORT_P4, PIN_ALL16);
    GPIO_setAsOutputPin(GPIO_PORT_P5, PIN_ALL16);
    GPIO_setAsOutputPin(GPIO_PORT_P6, PIN_ALL16);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, PIN_ALL16);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, PIN_ALL16);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, PIN_ALL16);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, PIN_ALL16);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, PIN_ALL16);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, PIN_ALL16);


    /* Configuring LFXTOUT and LFXTIN for XTAL operation and P1.0 for LED */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_PJ,
                                               GPIO_PIN0, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Setting LFXT to lowest drive strength and current consumption */
    CS_startLFXT(CS_LFXT_DRIVE0);

    /* Disabling high side voltage monitor/supervisor */
    PSS_disableHighSide();

    /* Initializing RTC to 11/19/2013 10:10:00 */
    RTC_C_initCalendar(&calendarTime, RTC_C_FORMAT_BINARY);

    /* Setting alarm for one minute later */
    RTC_C_setCalendarAlarm(calendarTime.minutes+time,0,1,1);

    /* Configuring P1.1 as an input and enabling interrupts for ondemand wakeup*/
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
    GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1);
    Interrupt_enableInterrupt(INT_PORT1);

    /* Setting up interrupts for the RTC. Once we enable interrupts, if there
     * was a pending interrupt due to a wake-up from partial shutdown then the
     * ISR will immediately fire*/
    RTC_C_enableInterrupt(RTC_C_CLOCK_ALARM_INTERRUPT);
    Interrupt_enableInterrupt(INT_RTC_C);
    Interrupt_enableMaster();
    RTC_C_startClock();
    //PCM_shutdownDevice(PCM_LPM35_VCORE0);
    PCM_gotoLPM3();

    //turning off the wake up button.
    GPIO_unregisterInterrupt(GPIO_PORT_P1);
    GPIO_disableInterrupt(GPIO_PORT_P1,GPIO_PIN1);
}

/* Real Time Clock ISR */
void RTC_C_IRQHandler(void)
{
    RTC_C_clearInterruptFlag(RTC_C_getInterruptStatus());
}

void GPS_align_panel(void){

    /*move panels to align with calculated sun position
     * alpha and beta angles for the sun(s) and the and the panel(p)
     * alpha is angle between normal vector and x-axis (+x is N, -x is S; motor1)
     * beta is angle between normal vector and y-axis
     */
    //Convert zenith and azimuth to Cartesian
    float x_cart = cos(Azimuth*PI/180)*sin(zenith_deg*PI/180); // rho=p=1
    //Find Cartesian angles from Cartesian coordinates
    float alpha_s = acos(x_cart)*180/PI; // output in degrees

    // Convert zenith and azimuth to Cartesian
    float y_cart = sin(Azimuth*PI/180)*sin(zenith_deg*PI/180);
    // Find Cartesian angles from Cartesian coordinates
    float beta_s = acos(y_cart)*180/PI; // output in degrees

    read_sensors();//updates sensor buffer
    int potNS=sensorBuffer[4];
    float alpha_p = 0.0127*potNS - 25.82; //formula to convert POT value to angle of the panel using Linear approximation
    int count = 5;//number of attempts
    do{
        float length_NS_p = (alpha_p - 45.96)/9.210;// Current length(in) of actuator

        float length_NS_s = (alpha_s - 45.96)/9.210;// length (in) needed to align with sun

        float delta_length_NS = length_NS_s - length_NS_p;//difference between the two

        float motorNS_time = fabs(delta_length_NS/motorNS_speed)/2;
        if (motorNS_time>180){// motor time move limit 3mins*60s/min=180s
            motorNS_time=180;
        }
        //move motors
        LED_indicator('r');
        if (fabs(delta_length_NS)>0.02){  //alignment threshold
            if (delta_length_NS<0)
                moveN(motorNS_time*1000);
            else
                moveS(motorNS_time*1000);
        }
        LED_indicator('b');
        //feedback checking
        read_sensors();
        potNS=sensorBuffer[4];
        alpha_p = 0.0127*potNS - 25.82;
        count--;
    }while(fabs(alpha_p-alpha_s)>=.5 && count);

    //beta is angle between normal vector and y-axis(+y is E, -y is W; motor2)
    read_sensors();
    int potEW=sensorBuffer[5];
    float beta_p = -0.0136*potEW + 118.12; //formula to convert POT value to angle of the panel using Linear approximation
    count = 5;

    do{  //convert zenith and azimuth to Cartesian

        float length_EW_p = (beta_p - 63.44)/5.585;// Current length (in) of actuator

        float length_EW_s = (beta_s - 63.44)/5.585;// length (in) needed to align with sun

        float delta_length_EW = length_EW_s - length_EW_p; // negative means retract

        float motorEW_time = fabs(delta_length_EW/motorEW_speed)/2;
        if (motorEW_time>180){// motor time move limit 3mins*60s/min=180s
            motorEW_time=180;
        }

        //move motors
        //float alpha_dif=alpha_s-alpha_p;
        //float beta_dif=beta_s-beta_p;
        LED_indicator('r');
        if (fabs(delta_length_EW)>0.02){  //alignment threshold
            if (delta_length_EW<0){
                if (length_EW_p>=1.09)//max length of actuator
                    moveE(motorEW_time*1000);
            }
            else{
                if (length_EW_p<=8.97)//max length of actuator
                    moveW(motorEW_time*1000);
            }
            LED_indicator('b');
        }
        read_sensors();
        int potEW=sensorBuffer[5];
        //Linear relationship between angle and potentiometer reading
        beta_p = -0.0136*potEW + 118.12; //formula to convert pot value to angle of the panel
        //find current panel position using potentiometers
        count--;
    }while(fabs(beta_p-beta_s)>=.3 && count);
    return;
}

void LED_indicator(char color){
    //changes the LED color
    //r=red, b=blue, g=green, y=yellow, c=cyan, p=purple, w=white, o=off

    GPIO_setAsOutputPin(GPIO_PORT_P2,GPIO_PIN0);//Red
    GPIO_setAsOutputPin(GPIO_PORT_P2,GPIO_PIN1);//Blue
    GPIO_setAsOutputPin(GPIO_PORT_P2,GPIO_PIN2);//Green
    LED_old=LED_color;
    LED_color=color;

    switch (color){
    case 'r':{
        //red
        GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN0);//red
        GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN1);//green
        GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN2);//blue
        break;
    }
    case 'b':{
        //blue
        GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN0);//red
        GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN1);//green
        GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN2);//blue
        break;
    }
    case 'g':{
        //green
        GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN0);//red
        GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN1);//green
        GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN2);//blue
        break;
    }
    case 'p':{
        //purple = blue+red
        GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN0);//red
        GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN1);//green
        GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN2);//blue
        break;
    }
    case 'y':{
        //yellow = red+green - bad color
        GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN0);//red
        GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN1);//green
        GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN2);//blue
        break;
    }
    case 'c':{
        //cyan = green + blue
        GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN0);//red
        GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN1);//green
        GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN2);//blue
        break;
    }
    case 'w':{
        //white = red+blue+green
        GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN0);//red
        GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN1);//green
        GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN2);//blue
        break;
    }
    case 'o':{
        //off
        GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN0);//red
        GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN1);//green
        GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN2);//blue
        break;
    }
    }
    return;
}
