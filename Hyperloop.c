#include <gtk/gtk.h>
#include <sys/time.h>
#include <unistd.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>
#include "IMU.c"
#include "Waveshare_BMP388.h"

//To compile type: 
//gcc -Wall -o Hyperloop Hyperloop.c Waveshare_BMP388.c -lwiringPi -lm -std=gnu99^C

#define DT 0.02         // [s/loop] loop period. 20ms
#define AA 0.97         // complementary filter constant

#define A_GAIN 0.0573      // [deg/LSB]
#define G_GAIN 0.070     // [deg/s/LSB]
#define RAD_TO_DEG 57.29578
#define M_PI 3.14159265358979323846


void  INThandler(int sig)
{
        signal(sig, SIG_IGN);
        exit(0);
}

int mymillis()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (tv.tv_sec) * 1000 + (tv.tv_usec)/1000;
}

int timeval_subtract(struct timeval *result, struct timeval *t2, struct timeval *t1)
{
    long int diff = (t2->tv_usec + 1000000 * t2->tv_sec) - (t1->tv_usec + 1000000 * t1->tv_sec);
    result->tv_sec = diff / 1000000;
    result->tv_usec = diff % 1000000;
    return (diff<0);
}

void main(int argc, char* argv[])
{
	char msg[32]={0};
	
	gtk_init(&argc, &argv);
	
	GtkBuilder *builder = gtk_builder_new();
	gtk_builder_add_from_file(builder,"hyperloopData.glade",NULL);
	
	GtkWidget *win = (GtkWidget *)gtk_builder_get_object(builder, "dataWindow");
		
	

	float rate_gyr_y = 0.0;   // [deg/s]
	float rate_gyr_x = 0.0;    // [deg/s]
	float rate_gyr_z = 0.0;     // [deg/s]
	int  accRaw[3];
	int  magRaw[3];
	int  gyrRaw[3];
	
	float initialAccValueX = 0.0;
	float initialAccValueY = 0.0;
	float initialAccValueZ = 0.0;
	float AccValueX = 0.0;
	float AccValueY = 0.0;
	float AccValueZ = 0.0;


	int startInt  = mymillis();
	struct  timeval tvBegin, tvEnd,tvDiff;


	signal(SIGINT, INThandler);

	detectIMU();
	enableIMU();
	
	//Accelerometer sensitivity +- 8g

	gettimeofday(&tvBegin, NULL);
	
	PRESS_EN_SENSOR_TYPY enPressureType;
	int32_t s32PressureVal = 0, s32TemperatureVal = 0, s32AltitudeVal = 0;

	pressSensorInit( &enPressureType );
  	if(PRESS_EN_SENSOR_TYPY_BMP388 == enPressureType)
	{
		printf("Pressure sersor is BMP388\n");
	}
	else
	{
		printf("Pressure sersor NULL\n");
	}
	printf("\r\n /-------------------------------------------------------------/\r\n ");
	delay(1000);

	while(1)
	{
		
		pressSensorDataGet(&s32TemperatureVal, &s32PressureVal, &s32AltitudeVal);
		g_snprintf(msg, sizeof msg, "\r\n Pressure: %.2f  Altitude: %.2f Temperature: %.1f\r\n",
				(float)s32PressureVal/100, 
				(float)s32AltitudeVal/100, 
				(float)s32TemperatureVal/100);
		delay(100);
		
		GtkWidget *tempVal = gtk_label_new(msg, "tempVal");
		gtk_container_add(GTK_CONTAINER(win),tempVal);
		
		startInt = mymillis();


		//read ACC and GYR data
		readACC(accRaw);
		readGYR(gyrRaw);
	
		int i = 1;
		while(i==1){
			initialAccValueX = (float) ((accRaw[0]*0.244) / 1000);
			initialAccValueY = (float) ((accRaw[1]*0.244) / 1000);
			initialAccValueZ = (float) ((accRaw[2]*0.244) / 1000);
			i=i+1;
		}
		AccValueX = (float) ((accRaw[0]*0.244) / 2000);
		AccValueY = (float) ((accRaw[1]*0.244) / 2000);
		AccValueZ = (float) ((accRaw[2]*0.244) / 2000);
	    
	    
		printf("Acceleration in X = %f G                Accleration in Y = %f G                  Acceleration in Z = %f G \n", AccValueX, AccValueY, AccValueZ);
		
		
		//Each loop should be at least 20ms.
		while(mymillis() - startInt < (DT*1000))
		{
			usleep(10000);
		}

		// printf("Loop Time %d\t", mymillis()- startInt);
		}
		gtk_widget_show_all(win);

		gtk_main();	
		return 0;
}
