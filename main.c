/******************************************************************************
  MSP432- 11.14.2020

  TASK SUMMARY
        search array for values*
        put values in new array*
        parsing GPS VARIBLES*
        light sensors
            sensor setup *
            sensor comparison
        calculate Azimuth, Zenith angles *
        read in potentiometer
            calculate panel position
        Calculate motor movement - finds time to move the motor
        move motors*
            find the speed of motors
        sleep mode
        night time mode?

 MSP432P401

PIN---------Function
P5.5  |<--- A0 (Analog Input) LDR 0, NE              sensorbuffer[0]
P5.4  |<--- A1 (Analog Input) LDR 1, NW              sensorbuffer[1]
P5.2  |<--- A3 (Analog Input) LDR 3, SE              sensorbuffer[3]
P5.1  |<--- A4 (Analog Input) Potentiometer NS       sensorbuffer[4]
P5.0  |<--- A5 (Analog Input) Potentiometer EW       sensorbuffer[5]
P4.7  |<----A6 (Analog Input) LDR 2, SW              sensorbuffer[2]
P3.2--------RX(TX from device)
P3.3--------TX(RX from device)
P1.2--------UART:pc
P1.3--------UART:pc
P3.6--------Sensor enable
P4.3--------Pot enable

OUT 1 ----P2.7--------North-----NS+
OUT 2 ----P2.6--------South-----NS-
OUT 3 ----P2.4--------East------EW+
OUT 4 ----P5.6--------West------EW-


 ***************************************************************************************************************************************************/

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

/*Mode
 * true= GPS mode
 * false = LDR mode
 * */
bool GPSmode=true;

/* GLOBAL VARIABLES */
// where deg refers to degrees and rad is radians
// parsing variables
int century = 2000; // 21st century
float deg; // from last group this was the latitude brought in from GPS
float degWhole; // last group - adjusted lat deg to be in deg, not deg-minutes
float hour = 0.00, min, sec;// current time
int time_24h;// current hour in 24 hour -- not being used audit all varibles ??????????????????????????????????????????????????????????????????????????
float latdeg, latmin, lat_min_decimal, latitude_deg, lat_rad;//latitude values
float longdeg, longmin, long_min_decimal, longitude_deg;// longitude values
int day, month, year;// the current day, month, year
int timezone = -5; // from UTC
const float PI = 3.14159265;
double hour_angle_deg, hour_angle_rad;// angle of the sun determined from the hour
float solar_decl_angle_deg,solar_decl_angle_rad; //
float beta;// for solar tilt
float Azimuth;
float a,b,d,e,f,g,h,i,m,y;//??
float ernie, isis, ony, cass;
float E_deg, i_rad, i_deg;
float degrad, zenith_deg, zenith_rad, DaytimeAdjust;
bool updateEnabled = true;
char conv_buffer[4];
float alpha_p,beta_p,alpha_s,beta_s;
/* alpha and beta angles for the sun(s) and the and the panel(p)
alpha is angle between normal vector and x-axis
beta is angle between normal vector and y-axis
 */
float motorNS_speed=15, motorEW_speed=15;// speed of the motors in degrees/second????????????????????????????????????????????????????????????????????????????????fix this
float SDA;
int daynumber;
int daysToMonth[2][12] =
{  //days from jan1, where row 0 is non-leap year, row 1 is leap year
   { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 },
   { 0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335 },
};

//GPS data storing variables
char rxBuffer[800];
char gpsdata[80];
uint16_t rxWriteIndex=0;
uint8_t data;
static uint16_t sensorBuffer[6]; //

//delay part
const int mtime= 3000000/1000;//clock speed /10^3 - clock speed in miliseconds

/*
psuedo GPS sentence for testing
 char gpsdata[] = {'$','G','P','R','M','C',',','2','2','1','5','0','3','.','0','0','0',',','A',',','3','3','5','6','.','3','1','8','4',',','N',',','0','8','4','3','1',
                                   '.','3','6','7','0',',','W',',','0','.','4','2',',','1','0','9','.','1','4',',','2','0','1','0','2','0'};
 *$GPGGA,203203.000,3356.3184,N,08431.3670,W,2,06,1.42,292.0,M,-30.8,M,0000,0000*52
 *$GPRMC,203203.000,A,3356.3184,N,08431.3670,W,0.42,109.14,151020,,,D*74
 *01234567890123456789012345678901234567890123456789012345678901234567890123456789012345
 *0         1         2         3         4         5         6         7         8

 */

void delay(int wait_time_ms){
    for(;wait_time_ms>0;wait_time_ms--){
        __delay_cycles(mtime);
    }
    return;
}

//function declarations
int convert_toHundreds(char hundreds, char tens, char ones);
bool is_leap_year(void);// checks if there is a leap year
void get_hour_angle(void); //
void calc_azimuth_zenith(void);//calculates the zenith and azimuth angle
void parseGPS(void);// collect GPS data and convert to proper data types values
void readGPS();// will pull correct sentence from rxBuffer

void read_sensors(void);//read inputs from all sensors
void moveN(int time);
void moveS(int time);
void moveE(int time);
void moveW(int time);
void compare_LDR(void);// compare LDR readings to move motors
void gotosleep(int time);//makes the motor sleep for minutes
void GPS_align_panel(void); //move panels to align



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

// Sleep mode calendar setup

RTC_C_Calendar calendarTime =
{
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
    /* Halting WDT  */
    MAP_WDT_A_holdTimer();
    MAP_Interrupt_disableSleepOnIsrExit();
    /* Setting DCO to 12MHz */
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);
    /* Configuring UART Module */
    MAP_UART_initModule(EUSCI_A2_BASE, &uartConfig);
    MAP_UART_initModule(EUSCI_A0_BASE, &uartConfig);
    /* Setting reference voltage to 2.5  and enabling reference */
    MAP_REF_A_setReferenceVoltage(REF_A_VREF2_5V);
    MAP_REF_A_enableReferenceVoltage();

    while(1){
        //SETUP GPS READ functionality**********************************
        /* Selecting P3.2 and P3.3 in UART mode
         * this is for communication to the GPS
         */
        MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,
                                                       GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
        /* Select P1.2 and P1.3 in UART mode
         * this is for the communication to the computer
         * */
        MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
                                                       GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

        /* Enable UART module */
        MAP_UART_enableModule(EUSCI_A2_BASE);
        MAP_UART_enableModule(EUSCI_A0_BASE);

        /* Enabling interrupts */
        MAP_UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT); // enable the interupt for the UART GPS reading
        MAP_Interrupt_enableInterrupt(INT_EUSCIA2); //Enable Interrupt for clock
        MAP_Interrupt_enableMaster();

        //Setup: Sensors Analog Input*****************************************

        /* Initializing ADC (MCLK/1/1) */
        MAP_ADC14_enableModule();
        MAP_ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1,
                             0);
        /* Configuring GPIOs for Analog In */
        MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5,
                                                       GPIO_PIN5 | GPIO_PIN4 | GPIO_PIN2 | GPIO_PIN1
                                                       | GPIO_PIN0, GPIO_TERTIARY_MODULE_FUNCTION);
        MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4,
                                                       GPIO_PIN7, GPIO_TERTIARY_MODULE_FUNCTION);
        /*Set sensors as input
            MSP432P401
         *             ------------------
         *         /|\|                  |
         *          | |                  |
         *          --|RST         P5.5  |<--- A0 (Analog Input) LDR 1, NW    sensorbuffer 0
         *            |            P5.4  |<--- A1 (Analog Input) LDR 2, SW    sensorbuffer 1
         *            |           XP5.3  |XXXX A2 (Analog Input) not using
         *            |            P5.2  |<--- A3 (Analog Input) LDR 3, SE    sensorbuffer 3
         *            |            P5.1  |<--- A4 (Analog Input) Potentiometer NS  sensorbuffer 4
         *            |            P5.0  |<--- A5 (Analog Input) Potentiometer EW  sensorbuffer 5
         *            |            P4.7  |<----A6 (Analog Input) LDR 0, NE   sensorbuffer 2
         *
         * */

        //Sensor enable pin
        MAP_GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN6);
        /* Configuring ADC Memory (ADC_MEM0 - ADC_MEM7 (A0 - A7)  with no repeat)
         * with internal 2.5v reference */
        MAP_ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM5, false);
        MAP_ADC14_configureConversionMemory(ADC_MEM0,
                                            ADC_VREFPOS_INTBUF_VREFNEG_VSS,
                                            ADC_INPUT_A0, false);
        MAP_ADC14_configureConversionMemory(ADC_MEM1,
                                            ADC_VREFPOS_INTBUF_VREFNEG_VSS,
                                            ADC_INPUT_A1, false);
        //P4.7 will be mapped to MEM2 or sensor buffer[2]
        MAP_ADC14_configureConversionMemory(ADC_MEM2,
                                            ADC_VREFPOS_INTBUF_VREFNEG_VSS,
                                            ADC_INPUT_A6, false);
        MAP_ADC14_configureConversionMemory(ADC_MEM3,
                                            ADC_VREFPOS_INTBUF_VREFNEG_VSS,
                                            ADC_INPUT_A3, false);
        MAP_ADC14_configureConversionMemory(ADC_MEM4,
                                            ADC_VREFPOS_INTBUF_VREFNEG_VSS,
                                            ADC_INPUT_A4, false);
        MAP_ADC14_configureConversionMemory(ADC_MEM5,
                                            ADC_VREFPOS_INTBUF_VREFNEG_VSS,
                                            ADC_INPUT_A5, false);
        //Motor pin setup
        MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN7); //North
        MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN6); //South
        MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN4); //East
        MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN6); //West

        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7); //North
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN6); //South
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN4); //East
        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN6); //West
        fflush(stdout);
        printf("setup completed\n");
        fflush(stdout);

        read_sensors();
        if (fmin(sensorBuffer[0],sensorBuffer[1],sensorBuffer[2],sensorBuffer[3])>9000){//  if it is really dark
            bool GPSmode=true;
        }
        else
            bool GPSmode=false

            if(GPSmode){
                //GPS mode
                while(rxWriteIndex<800){
                    //waits for the GPS buffer to fill up
                }
                //UART_disableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT); // disable the interrupt for the UART GPS reading
                readGPS(); // first time reading on startup - will read from rxBuffer into gpsdata
                parseGPS();   // collect GPS data and convert to proper data types values
                calc_azimuth_zenith();   //calculates the zenith and azimuth angle
                GPS_align_panel(); //move panels to align
            }
            else{

                // LDR mode
                read_sensors(); // read inputs from all sensors
                compare_LDR();  // compare LDR readings to move motors
            }

        gotosleep(0);  //makes the board sleep for minutes
    }
}

/***************************************************************************************************************************************************************/

/* EUSCI A0 UART ISR - Echoes data back to PC host */
void EUSCIA2_IRQHandler(void) //fill up the rxBuffer array
{
    //9600 Baud
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A2_BASE);

    MAP_UART_clearInterruptFlag(EUSCI_A2_BASE,status);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        data=MAP_UART_receiveData(EUSCI_A2_BASE);
        rxBuffer[rxWriteIndex++]=data;
        //UART_transmitData(EUSCI_A0_BASE, data);
        // printf("%c",data);
        if(rxWriteIndex>=800){
            MAP_Interrupt_disableMaster();// disables all the interrupts
            //MAP_PCM_gotoLPM0InterruptSafe();//allows board to go to sleep mode with interupt enable

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


void get_hour_angle()
{
    if (hour < 12)
    {
        hour_angle_deg = 15.00*(-(hour + min/60.00) - 12.00); // negative hour angle
    }
    else
    {
        hour_angle_deg = 15.00*(hour + min/60.00 - 12.00); // positive hour angle
    }

    hour_angle_rad = hour_angle_deg*PI/180.00;
}


void calc_azimuth_zenith()
{

    //this function finds the day number, hour angle, and solar declination angle of the sun
    daynumber = daysToMonth[is_leap_year()][month-1] + day; // calculate the daynumber of the year
    get_hour_angle();

    //SDA = 23.45*sin(360.00/365.00*(284 + daynumber)); // solar declination angle, where () is from last group
    //                     |-------isis------------|
    //             |----ernie----------------------| ==> (Edegree)
    //delta = 23.45* Edegree                         ==> (Dradian) --> solar_decl_angle_deg and solar_decl_angle_deg

    isis = 360.00/(is_leap_year() ? 366.00 :365.00)*(284 + daynumber); // if it is a leap year use 366 else use 365
    ernie = sin(isis*PI/180);//find sine and convert to radians
    E_deg = ernie;// * (180.00/PI); // convert to degrees
    solar_decl_angle_deg = 23.45 * E_deg;
    solar_decl_angle_rad = solar_decl_angle_deg * PI/180.00;

    //calculates the zenith and azimuth angle
    // zenith = asin(sin(gps_lat)*sin(SDA) + cos(gps_lat)*cos(SDA)*cos*(hour_angle))

    lat_rad = latitude_deg * (PI/180.00);
    degrad = lat_rad;
    a = sin(degrad); // where degrad is the gps lat in radian
    b = sin(solar_decl_angle_rad);
    d = cos(degrad);
    e = cos(solar_decl_angle_rad);
    f = cos(hour_angle_rad); // cos(Hradian)
    g = sin(hour_angle_rad); // sin(Hradian_
    zenith_rad = acos((a*b)+(d*e*f)); // acos((a*b)+(d*e*f)); measured from zenith(vertical)
    h=cos(zenith_rad);
    zenith_deg = zenith_rad * 180.00/PI;
    beta = 90.00 - zenith_deg; // for solar tilt

    Azimuth=180-acos(-((sin(lat_rad)*cos(zenith_rad))-sin(solar_decl_angle_rad))/(cos(lat_rad)*sin(zenith_rad)));
}


void parseGPS()
{
    // pull out all the required information from the GPS data
    hour = convert_toHundreds('0',gpsdata[7],gpsdata[8]) + timezone;
    min = convert_toHundreds('0',gpsdata[9],gpsdata[10]);
    sec = convert_toHundreds('0',gpsdata[11],gpsdata[12]);
    time_24h = hour*100 + min; // this is the 24 hour time


    // recieved data from GPS is in deg + min , but needs to be in degrees only
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
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6);
    /* Zero-filling buffer */
    memset(sensorBuffer, 0x00, 6 * sizeof(uint16_t));
    /* Setting up the sample timer to automatically step through the sequence
     * convert.
     */
    MAP_ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);

    /* Triggering the start of the sample */
    MAP_ADC14_enableConversion();
    MAP_ADC14_toggleConversionTrigger();

    //fill up array with results
    MAP_ADC14_getMultiSequenceResult(sensorBuffer);
    //turn off to save power
    //MAP_ADC14_disableSampleTimer(ADC_AUTOMATIC_ITERATION);
    //MAP_ADC14_disableConversion();
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6);
}

void moveN(int time){
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
    int temp = 0;
    int minVal = 0;
    int minIdx=0;
    int light_tol=0;
    int thresh = 500; // this is the tolerance between LDRs
    int adj_sensorNS = 0;
    int adj_sensorEW = 0;
    int move_time=1000; // the time allowed for motor movement


    /*
    int sensorNE=sensorBuffer[0]; // LDR0
    int sensorNW=sensorBuffer[1]; // LDR1
    int sensorSW=sensorBuffer[2]; // LDR2
    int sensorSE=sensorBuffer[3]; // LDR3
     */

    // find the min value among the LDRs
    temp = fmin(sensorBuffer[0],sensorBuffer[1]); //
    minVal = fmin(sensorBuffer[2],sensorBuffer[3]); //
    minVal = fmin(temp,minVal);
    int i = 0;
    for (;i<4;i++){ // get the index of where the max val occurs
        if (minVal == sensorBuffer[i]){
            minIdx = i;
            break;
        }
    }

    adj_sensorNS = 3 - minIdx; //gets the north/south adjacent sensor

    // to get the east/west adjacent sensor
    switch(minIdx){
    case 0: adj_sensorEW = 1; break;
    case 1: adj_sensorEW = 0; break;
    case 2: adj_sensorEW = 3; break;
    case 3: adj_sensorEW = 2; break;
    }

    bool moveenabled = true;
    // the higher the reading on a LDR means the least reading on light
    while(moveenabled){
        int stop_count = 10;
        // compare the max val to determined light tolerance
        if (minVal > light_tol){
            //      if met, continue

            // compare highest value with the E/W adjacent pin
            while(( sensorBuffer[adj_sensorEW] - sensorBuffer[minIdx] > thresh)&& stop_count){

                if(minIdx == 0 || minIdx == 3 )
                    moveE(move_time); // tilt panel's east side downward
                else
                    moveW(move_time); // tilts upward
                stop_count--;
                read_sensors();
            }
            stop_count = 10;
            // compare highest value with the N/S adjacent pin
            while(( sensorBuffer[adj_sensorNS] - sensorBuffer[minIdx] > thresh) && stop_count){

                if(minIdx < 2)
                    moveN(move_time); // tilt panel's north side downward 1s
                else
                    moveS(move_time); // tilts upward
                stop_count--;
                read_sensors();

            }


            moveenabled = false; // panel has been adjusted to LDR equalization
        }
    }
}

void gotosleep(int time){
    // Sleep mode setup
    /* If this flag has been set, it means that the device has already
     * been into LPM3.5 mode before.
     */
    if (MAP_ResetCtl_getPCMSource() & RESET_LPM35)
    {
        /* Clearing the PCM Reset flags */
        MAP_ResetCtl_clearPCMFlags();

        /* Unlocking the latched GPIO/LPM configuration flag */
        PCM->CTL1 = PCM_CTL1_KEY_VAL;
    }
    /* Terminating all remaining pins to minimize power consumption. This is
            done by register accesses for simplicity and to minimize branching API
            calls */
    MAP_GPIO_setAsOutputPin(GPIO_PORT_PA, PIN_ALL16);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_PB, PIN_ALL16);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_PC, PIN_ALL16);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_PD, PIN_ALL16);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_PE, PIN_ALL16);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_PJ, PIN_ALL16);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_PA, PIN_ALL16);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_PB, PIN_ALL16);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_PC, PIN_ALL16);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_PD, PIN_ALL16);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_PE, PIN_ALL16);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_PJ, PIN_ALL16);

    /* Configuring LFXTOUT and LFXTIN for XTAL operation and P1.0 for LED */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_PJ,
                                                   GPIO_PIN0, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Setting LFXT to lowest drive strength and current consumption */
    MAP_CS_startLFXT(CS_LFXT_DRIVE0);

    /* Disabling high side voltage monitor/supervisor */
    MAP_PSS_disableHighSide();

    /* Initializing RTC to 11/19/2013 10:10:00 */
    MAP_RTC_C_initCalendar(&calendarTime, RTC_C_FORMAT_BINARY);

    /* Setting alarm for one minute later */
    MAP_RTC_C_setCalendarAlarm(calendarTime.minutes+time,0,1,1);

    /* Setting up interrupts for the RTC. Once we enable interrupts, if there
     * was a pending interrupt due to a wake-up from partial shutdown then the
     * ISR will immediately fire and blinkLED will be set to true.*/
    MAP_RTC_C_enableInterrupt(RTC_C_CLOCK_ALARM_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_RTC_C);
    MAP_Interrupt_enableMaster();
    MAP_RTC_C_startClock();
    MAP_PCM_shutdownDevice(PCM_LPM35_VCORE0);

}

/* RTC ISR */
void RTC_C_IRQHandler(void)
{
    MAP_RTC_C_clearInterruptFlag(MAP_RTC_C_getInterruptStatus());
}

void GPS_align_panel(void){
    //alpha is angle between normal vector and x-axis (+x is N, -x is S; motor1)
    //beta is angle between normal vector and y-axis(+y is E, -y is W; motor2)

    //find current panel position using potentiometers
    read_sensors();
    int potNS=sensorBuffer[5], potEW=sensorBuffer[6];

    //Linear relationship between angle and potentiometer reading
    alpha_p=0.0127*potNS-25.82; //formula to convert pot value to angle of the panel
    beta_p=-0.0141*potEW+118.4; //formula to convert pot value to angle of the panel

    //convert zenith and azimuth to Cartesian
    float x_cart, y_cart;


    // Find Cartesian coordinates from spherical angles
    x_cart = cos(Azimuth*PI/180)*sin(zenith_deg*PI/180); // rho=p=1
    y_cart = sin(Azimuth*PI/180)*sin(zenith_deg*PI/180);

    // Find cartesian angles from cartesian coordinates
    alpha_s = acos(x_cart)*180/PI; // output in degrees
    beta_s = acos(y_cart)*180/PI; // output in degrees

    //move motors
    float alpha_dif=alpha_s-alpha_p;
    if (alpha_dif>0.02){  //change zero to allow for alignment
        moveS(alpha_dif*1000/motorNS_speed);
    }
    else if (alpha_dif<-0.02){
        moveN(-alpha_dif*1000/motorNS_speed);
    }
    float beta_dif=beta_s-beta_p;
    if (beta_dif>0.02){  //change zero to allow for alignment
        moveE(beta_dif*1000/motorEW_speed);
    }
    else if(beta_dif<-0.02){
        moveW(-beta_dif*1000/motorEW_speed);
    }
}

