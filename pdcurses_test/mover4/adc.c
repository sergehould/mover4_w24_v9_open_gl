/*
*	adc.c
* 
*	Description: Contains functions related to ADC.
*
*	Author				Date					Version
*	Serge Hould			16 May 2022				v1.0.0	First version
*	SH					6 Mar. 2023				v1.1.0	renamed adc_read()
*														Add UDP mode
*	
*********************************************************************************************************/


#include <Windows.h>
#include "header/task_controller.h"
#include "header/can.h"
#include <math.h>	
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <curses.h>
#include <stdint.h>
#include <ctype.h> 
#include <errno.h>
#include <sys/types.h>
#ifdef _WIN32
#define HAVE_STRUCT_TIMESPEC  // for win32 only. Because TIMESPEC is re-defined inside pthread.h
#endif
#include <pthread.h>
#include "header/config.h"


#ifndef _WIN32
#define MAX_BUF	100
//read adc function definitions  
int adc_read(unsigned int pin)
{  
	int fd;          //file pointer  
	char buf[MAX_BUF];     //file buffer  
	char val[]="    ";     //holds up to 4 digits for ADC value  
      
      //Create the file path by concatenating the ADC pin number to the end of the string  
      //Stores the file path name string into "buf"  
	snprintf(buf, sizeof(buf),"/sys/bus/iio/devices/iio:device0/in_voltage%d_raw", pin);     //Concatenate ADC file name  
	//snprintf(buf, sizeof(buf), "/sys/devices/ocp.3/helper.15/AIN%d", pin);     //Concatenat	      
	fd = open(buf, O_RDONLY);     //open ADC as read only  
      
      //Will trigger if the ADC is not enabled  
	if (fd < 0) {
		perror("ADC - problem opening ADC");  
	}//end if
	read(fd, &val, 4);     //read ADC ing val (up to 4 digits 0-1799)
	close(fd);     //close file and stop reading
	return atoi(val);     //returns an integer value (rat }//end read ADC()
}
#else
#if defined UDP
int adc_read(unsigned int pin)
{
	return readADC_udp();
}
#else
int adc_read(unsigned int pin)
{ 
	return 0;
}
#endif
#endif