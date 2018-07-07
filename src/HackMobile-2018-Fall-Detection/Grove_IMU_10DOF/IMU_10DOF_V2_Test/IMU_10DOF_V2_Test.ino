#include "Wire.h"


#include "I2Cdev.h"
#include "MPU9250.h"
#include "BMP280.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU9250 accelgyro;
I2Cdev   I2C_M;

uint8_t buffer_m[6];


int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

int pulsesensor = 0;
int Signal;

float heading;
float tiltheading;

float Axyz[3];
float Gxyz[3];
float Prev_Axyz[3];
float Prev_Gxyz[3];
float Mxyz[3];
float diff_x;
float diff_y;
float diff_z;
float diff_x_ang;
float diff_y_ang;
float diff_z_ang;
float w;
float a;

int state = 0;
int flag = 0;
int hadFall = 0;

#define sample_num_mdate  5000

volatile float mx_sample[3];
volatile float my_sample[3];
volatile float mz_sample[3];

static float mx_centre = 0;
static float my_centre = 0;
static float mz_centre = 0;

volatile int mx_max = 0;
volatile int my_max = 0;
volatile int mz_max = 0;

volatile int mx_min = 0;
volatile int my_min = 0;
volatile int mz_min = 0;

float temperature;
float pressure;
float atm;
float altitude;
BMP280 bmp280;

void setup()
{
    
    Wire.begin();

    Serial.begin(9600);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
    bmp280.init();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU9250 connection successful" : "MPU9250 connection failed");

    delay(1000);
    Serial.println("     ");

    //  Mxyz_init_calibrated ();

}

void loop()
{

    getAccel_Data();
    getGyro_Data();
    getCompassDate_calibrated(); // compass data has been calibrated here
    getHeading();               //before we use this function we should run 'getCompassDate_calibrated()' frist, so that we can get calibrated data ,then we can get correct angle .
    getTiltHeading();

    /*
    Serial.println("calibration parameter: ");
    Serial.print(mx_centre);
    Serial.print("         ");
    Serial.print(my_centre);
    Serial.print("         ");
    Serial.println(mz_centre);
    Serial.println("     ");
    */

    Prev_Axyz[0] = Axyz[0];
    Prev_Axyz[1] = Axyz[1];
    Prev_Axyz[2] = Axyz[2];
    delay(100);
    getAccel_Data();

    diff_x = Axyz[0] - Prev_Axyz[0];
    diff_y = Axyz[1] - Prev_Axyz[1];
    diff_z = Axyz[2] - Prev_Axyz[2];
    a = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);
    /*
    Serial.print("a ");
    Serial.print(a);
    Serial.println(', ');
*/    

    
    /*
    Serial.print(diff_x);
    Serial.print(",");
    Serial.print(diff_y);
    Serial.print(",");
    Serial.println(diff_z);
    */
    /*
    int test = sqrt(Axyz[0] * Axyz[0] + Axyz[1] * Axyz[1] + Axyz[2] * Axyz[2]);
    Serial.println(test);
    */

    Prev_Gxyz[0] = Gxyz[0];
    Prev_Gxyz[1] = Gxyz[1];
    Prev_Gxyz[2] = Gxyz[2];

    delay(100);
    getGyro_Data();
    diff_x_ang = Gxyz[0] - Prev_Gxyz[0];
    diff_y_ang = Gxyz[1] - Prev_Gxyz[1];
    diff_z_ang = Gxyz[2] - Prev_Gxyz[2];
    w = sqrt(diff_x_ang * diff_x_ang + diff_y_ang * diff_y_ang + diff_z_ang * diff_z_ang);
  /*   
   Serial.print("w ");
    Serial.print(w);
    Serial.print(", ");
*/
    if (a >= 2 and w >= 300) //3, 400
    {
      /*
      Serial.println("a: ");
      Serial.println(a);
      Serial.print("w: ");
      Serial.println(w);*/
      //Serial.println("Fall");
      Serial.println("Fall detected. Are you okay? (1 for yes, 0 for no)");
      hadFall = 1;
    }
    sendMessage();

    Signal = analogRead(pulsesensor);
    //Serial.println(Signal);
    //Serial.println(',');
    //Serial.println();

}

void sendMessage(void)
{
  
    //if some data is sent, read it and save it in the state variable
  if (Serial.available() > 0 && hadFall == 1) {
    state = Serial.read();
    flag = 0; //flag for indicating response ready to be processed
  }
  // if the state is 0 the led will turn off
  if (state == '1') {
    if (flag == 0) {
      Serial.println("Ok. Will not contact emergency services.");
      flag = 1;
      hadFall = 0;
    }
  }
  // if the state is 1 the led will turn on
  else if (state == '0') {
    if (flag == 0) {
      Serial.println("Contacting emergency services.");
      flag = 1;
      hadFall = 0;
    }
  }
}
void getHeading(void)
{
    heading = 180 * atan2(Mxyz[1], Mxyz[0]) / PI;
    if (heading < 0) heading += 360;
}

void getTiltHeading(void)
{
    float pitch = asin(-Axyz[0]);
    float roll = asin(Axyz[1] / cos(pitch));

    float xh = Mxyz[0] * cos(pitch) + Mxyz[2] * sin(pitch);
    float yh = Mxyz[0] * sin(roll) * sin(pitch) + Mxyz[1] * cos(roll) - Mxyz[2] * sin(roll) * cos(pitch);
    float zh = -Mxyz[0] * cos(roll) * sin(pitch) + Mxyz[1] * sin(roll) + Mxyz[2] * cos(roll) * cos(pitch);
    tiltheading = 180 * atan2(yh, xh) / PI;
    if (yh < 0)    tiltheading += 360;
}



void Mxyz_init_calibrated ()
{

    Serial.println(F("Before using 9DOF,we need to calibrate the compass frist,It will takes about 2 minutes."));
    Serial.print("  ");
    Serial.println(F("During  calibratting ,you should rotate and turn the 9DOF all the time within 2 minutes."));
    Serial.print("  ");
    Serial.println(F("If you are ready ,please sent a command data 'ready' to start sample and calibrate."));
    while (!Serial.find("ready"));
    Serial.println("  ");
    Serial.println("ready");
    Serial.println("Sample starting......");
    Serial.println("waiting ......");

    get_calibration_Data ();

    Serial.println("     ");
    Serial.println("compass calibration parameter ");
    Serial.print(mx_centre);
    Serial.print("     ");
    Serial.print(my_centre);
    Serial.print("     ");
    Serial.println(mz_centre);
    Serial.println("    ");
}


void get_calibration_Data ()
{
    for (int i = 0; i < sample_num_mdate; i++)
    {
        get_one_sample_date_mxyz();
        /*
        Serial.print(mx_sample[2]);
        Serial.print(" ");
        Serial.print(my_sample[2]);                            //you can see the sample data here .
        Serial.print(" ");
        Serial.println(mz_sample[2]);
        */



        if (mx_sample[2] >= mx_sample[1])mx_sample[1] = mx_sample[2];
        if (my_sample[2] >= my_sample[1])my_sample[1] = my_sample[2]; //find max value
        if (mz_sample[2] >= mz_sample[1])mz_sample[1] = mz_sample[2];

        if (mx_sample[2] <= mx_sample[0])mx_sample[0] = mx_sample[2];
        if (my_sample[2] <= my_sample[0])my_sample[0] = my_sample[2]; //find min value
        if (mz_sample[2] <= mz_sample[0])mz_sample[0] = mz_sample[2];

    }

    mx_max = mx_sample[1];
    my_max = my_sample[1];
    mz_max = mz_sample[1];

    mx_min = mx_sample[0];
    my_min = my_sample[0];
    mz_min = mz_sample[0];



    mx_centre = (mx_max + mx_min) / 2;
    my_centre = (my_max + my_min) / 2;
    mz_centre = (mz_max + mz_min) / 2;

}






void get_one_sample_date_mxyz()
{
    getCompass_Data();
    mx_sample[2] = Mxyz[0];
    my_sample[2] = Mxyz[1];
    mz_sample[2] = Mxyz[2];
}


void getAccel_Data(void)
{
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Axyz[0] = (double) ax / 16384;
    Axyz[1] = (double) ay / 16384;
    Axyz[2] = (double) az / 16384;
}

void getGyro_Data(void)
{
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Gxyz[0] = (double) gx * 250 / 32768;
    Gxyz[1] = (double) gy * 250 / 32768;
    Gxyz[2] = (double) gz * 250 / 32768;
}

void getCompass_Data(void)
{
    I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
    delay(10);
    I2C_M.readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer_m);

    mx = ((int16_t)(buffer_m[1]) << 8) | buffer_m[0] ;
    my = ((int16_t)(buffer_m[3]) << 8) | buffer_m[2] ;
    mz = ((int16_t)(buffer_m[5]) << 8) | buffer_m[4] ;

    Mxyz[0] = (double) mx * 1200 / 4096;
    Mxyz[1] = (double) my * 1200 / 4096;
    Mxyz[2] = (double) mz * 1200 / 4096;
}

void getCompassDate_calibrated ()
{
    getCompass_Data();
    Mxyz[0] = Mxyz[0] - mx_centre;
    Mxyz[1] = Mxyz[1] - my_centre;
    Mxyz[2] = Mxyz[2] - mz_centre;
}
