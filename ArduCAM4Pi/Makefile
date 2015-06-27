all : OV2640_Playback	OV3640_Playback	OV5642_Playback	OV7670_Playback OV2640_DigitalCamera	OV5642_DigitalCamera

objects = DefaultFonts_SPI.o UTFT_SPI.o PiCAM.o

OV2640_Playback : $(objects) PiCAM_OV2640_Playback.o 
	gcc -o OV2640_Playback $(objects) PiCAM_OV2640_Playback.o -lwiringPi
OV3640_Playback : $(objects) PiCAM_OV3640_Playback.o 
	gcc -o OV3640_Playback $(objects) PiCAM_OV3640_Playback.o -lwiringPi
OV5642_Playback : $(objects) PiCAM_OV5642_Playback.o 
	gcc -o OV5642_Playback $(objects) PiCAM_OV5642_Playback.o -lwiringPi
OV7670_Playback : $(objects) PiCAM_OV7670_Playback.o 
	gcc -o OV7670_Playback $(objects) PiCAM_OV7670_Playback.o -lwiringPi
OV2640_DigitalCamera : $(objects) PiCAM_OV2640_DigitalCamera.o 
	gcc -o OV2640_DigitalCamera $(objects) PiCAM_OV2640_DigitalCamera.o -lwiringPi
OV5642_DigitalCamera : $(objects) PiCAM_OV5642_DigitalCamera.o 
	gcc -o OV5642_DigitalCamera $(objects) PiCAM_OV5642_DigitalCamera.o -lwiringPi
	
DefaultFonts_SPI.o : DefaultFonts_SPI.c
	gcc -c DefaultFonts_SPI.c -lwiringPi
UTFT_SPI.o : UTFT_SPI.c
	gcc -c UTFT_SPI.c -lwiringPi
PiCAM.o : PiCAM.c
	gcc -c PiCAM.c -lwiringPi
PiCAM_OV2640_Playback.o : PiCAM_OV2640_Playback.c
	gcc -c PiCAM_OV2640_Playback.c -lwiringPi
PiCAM_OV3640_Playback.o : PiCAM_OV3640_Playback.c
	gcc -c PiCAM_OV3640_Playback.c -lwiringPi
PiCAM_OV5642_Playback.o : PiCAM_OV5642_Playback.c
	gcc -c PiCAM_OV5642_Playback.c -lwiringPi
PiCAM_OV7670_Playback.o : PiCAM_OV7670_Playback.c
	gcc -c PiCAM_OV7670_Playback.c -lwiringPi
PiCAM_OV2640_DigitalCamera.o : PiCAM_OV2640_DigitalCamera.c
	gcc -c PiCAM_OV2640_DigitalCamera.c -lwiringPi
PiCAM_OV5642_DigitalCamera.o : PiCAM_OV5642_DigitalCamera.c
	gcc -c PiCAM_OV5642_DigitalCamera.c -lwiringPi	

clean : 
	rm -f OV2640_Playback OV3640_Playback OV5642_Playback OV7670_Playback OV2640_DigitalCamera OV5642_DigitalCamera $(objects) \
	PiCAM_OV2640_Playback.o PiCAM_OV3640_Playback.o PiCAM_OV5642_Playback.o PiCAM_OV7670_Playback.o \
	PiCAM_OV2640_DigitalCamera.o PiCAM_OV5642_DigitalCamera.o