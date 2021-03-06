/*
 ============================================================================
 Name        : PiCAM_OV3640.c
 Author      : Lee
 Version     : V1.0
 Copyright   : ArduCAM demo (C)2014 Lee
 Description :
 ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/time.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include "utft_spi.h"
#include "arducam.h"

#define BOOL int
#define TRUE 1
#define FALSE 0
#define BMPIMAGEOFFSET 66

int Playback(void);
int GrabImage(char* str);

const char bmp_header[BMPIMAGEOFFSET] PROGMEM =
{
      0x42, 0x4D, 0x36, 0x58, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x42, 0x00, 0x00, 0x00, 0x28, 0x00,
      0x00, 0x00, 0x40, 0x01, 0x00, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x01, 0x00, 0x10, 0x00, 0x03, 0x00,
      0x00, 0x00, 0x00, 0x58, 0x02, 0x00, 0xC4, 0x0E, 0x00, 0x00, 0xC4, 0x0E, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x00, 0x00, 0xE0, 0x07, 0x00, 0x00, 0x1F, 0x00,
      0x00, 0x00
};

void setup()
{
  uint8_t temp;

  UTFT();
  arducam(smOV3640);

  printf("ArduCAM Start!\n");

  //Check if the ArduCAM SPI bus is OK
  arducam_write_reg(ARDUCHIP_TEST1, 0x55);
  temp = arducam_read_reg(ARDUCHIP_TEST1);
  if(temp != 0x55)
  {
  	printf("SPI interface Error!");
  	while(1);
  }

  //Change MCU mode
  arducam_write_reg(ARDUCHIP_MODE, 0x00);

  //Initialize the LCD Module
  InitLCD();

  arducam_init();
}

int main()
{
  setup();
  unsigned long previous_time = 0;
  uint8_t temp;
  struct timeval tv;
  struct timezone tz;
  arducam_write_reg(ARDUCHIP_MODE, 0x01);		 	//Switch to CAM

  while(1)
  {
    temp = arducam_read_reg(ARDUCHIP_TRIG);

    if(!(temp & VSYNC_MASK))				//New Frame is coming
    {
       arducam_write_reg(ARDUCHIP_MODE, 0x00);    	//Switch to MCU
       resetXY();
       arducam_write_reg(ARDUCHIP_MODE, 0x01);    	//Switch to CAM
       while(!(arducam_read_reg(ARDUCHIP_TRIG)&0x01)); 	//Wait for VSYNC is gone
    }
    else if(temp & SHUTTER_MASK)
    {
    	gettimeofday (&tv , &tz);
    	previous_time = tv.tv_sec;
		//printf("previous_time is %d.\n",previous_time);
       while(arducam_read_reg(ARDUCHIP_TRIG) & SHUTTER_MASK)
       {
    	 gettimeofday (&tv , &tz);
		 //printf("put time is %d.\n",tv.tv_sec);
         if((tv.tv_sec - previous_time) >= 2)
         {
           Playback();
		   arducam_delay_ms(1000);
         }
       }
	   
       gettimeofday (&tv , &tz);  
       if((tv.tv_sec - previous_time) < 2)
       {
         char filePath[128];
         time_t timep;
         struct tm *p;
         time(&timep);
         p = localtime(&timep);
         printf("Capture Done!\n");
         snprintf(filePath, sizeof(filePath), "/home/pi/%04d%02d%02d%02d%02d%02d.bmp", 1900+p->tm_year, 1+p->tm_mon, p->tm_mday, p->tm_hour, p->tm_min, p->tm_sec);
    	   //Open the new file
    	   FILE *fp = fopen(filePath,"w+");
    	   if (fp == NULL)
    	   {
    		   printf("open file failed\n");
    		   return 0;
    	   	  }				//Generate file name
         arducam_write_reg(ARDUCHIP_MODE, 0x00);    	//Switch to MCU, freeze the screen
         GrabImage(filePath);
       }
    }
  }
}


int GrabImage(char* str)
{
  char VH,VL;
  uint8_t buf[256];
  static int k = 0;
  int i,j = 0;
  int nmemb = 1;
  printf("GrabImage.\n");

  //Open the new file
  FILE *fp = fopen(str,"w+");
  if (fp == NULL)
  {
	  printf("open file failed\n");
	  return 0;
  }
  //Switch to FIFO Mode
  arducam_write_reg(ARDUCHIP_TIM, MODE_MASK);
  //Flush the FIFO
  arducam_flush_fifo();
  //Start capture
  arducam_start_capture();
  printf("Start Capture\n");

  //Polling the capture done flag
  while(!(arducam_read_reg(ARDUCHIP_TRIG) & CAP_DONE_MASK));
  printf("Capture Done!\n");

  k = 0;
  //Write the BMP header
  for( i = 0; i < BMPIMAGEOFFSET; i++)
  {
    char ch = pgm_read_byte(&bmp_header[i]);
    buf[k++] = ch;
  }
  fwrite(buf,k,nmemb,fp);
  //Read first dummy byte
  //myCAM.read_fifo();

  k = 0;
  //Read 320x240x2 byte from FIFO
  //Save as RGB565 bmp format
  for(i = 0; i < 240; i++)
    for(j = 0; j < 320; j++)
  {
      VH = arducam_read_fifo();
      VL = arducam_read_fifo();
      buf[k++] = VL;
      buf[k++] = VH;
      //Write image data to bufer if not full
      if(k >= 256)
      {
        //Write 256 bytes image data to file from buffer
        fwrite(buf,256,nmemb,fp);
        k = 0;
      }
  }
  //Close the file
  fclose(fp);
  //Clear the capture done flag
  arducam_clear_fifo_flag();

  //Switch to LCD Mode
  arducam_write_reg(ARDUCHIP_TIM, 0);
  return 1;
}

int Playback(void)
{
	int nmemb = 1;
	FILE *bmppath,*fnum,*photo;
	system("dir /home/pi/*.bmp > /home/pi/bmp.txt");
	printf("Camera Playback. \n");
	arducam_write_reg(ARDUCHIP_MODE, 0x00);    		//Switch to MCU
	InitLCD(PORTRAIT);
	
	
	fnum = fopen("/home/pi/bmp.txt","r");
	fseek(fnum, 0, SEEK_END);   
    int size = ftell(fnum);
	fclose(fnum);
	printf("There are %d bmp photos.\n",size/28);

	char files[size];
	bmppath = fopen("/home/pi/bmp.txt","r+");
	fread(files,size,nmemb,bmppath);
	fclose(bmppath);
	//printf("%s",files);
	
	char *p;
	char str[28];
	p = index(files,'/');
	
	while(p!=0)
	{
		memset(str,'\0',28);
		memcpy(str,p,27);
		printf("display photo %s.\n",str);
		memset(p,'0',27);
		p = index(files,'/');
		//if(read_reg(ARDUCHIP_TRIG)& SHUTTER_MASK)
		//{
			//p=0;
			//break;
		//}
		//Open the new file
		photo = fopen(str,"r");
		if (photo == NULL)
		{
			printf("open file failed.\n");
			//return 0;
		}
		//clrScr();
		//myGLCD.resetXY();
		dispBitmap(photo);
		fclose(photo);
	}
	printf("Finish playback.\n");
	return 1;
}
