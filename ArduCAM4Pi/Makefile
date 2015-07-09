all : ov2640_playback	ov3640_playback	ov5642_playback	ov7670_playback ov2640_digital_camera	ov5642_digital_camera ov2640_capture

objects = default_fonts_spi.o utft_spi.o arducam.o

ov2640_playback : $(objects) arducam_ov2640_playback.o 
	gcc -o ov2640_playback $(objects) arducam_ov2640_playback.o -lwiringPi -Wall
ov3640_playback : $(objects) arducam_ov3640_playback.o 
	gcc -o ov3640_playback $(objects) arducam_ov3640_playback.o -lwiringPi -Wall
ov5642_playback : $(objects) arducam_ov5642_playback.o 
	gcc -o ov5642_playback $(objects) arducam_ov5642_playback.o -lwiringPi -Wall
ov7670_playback : $(objects) arducam_ov7670_playback.o 
	gcc -o ov7670_playback $(objects) arducam_ov7670_playback.o -lwiringPi -Wall
ov2640_digital_camera : $(objects) arducam_ov2640_digital_camera.o 
	gcc -o ov2640_digital_camera $(objects) arducam_ov2640_digital_camera.o -lwiringPi -Wall
ov2640_capture : $(objects) arducam_ov2640_capture.o 
	gcc -o ov2640_capture $(objects) arducam_ov2640_capture.o -lwiringPi -Wall
ov5642_digital_camera : $(objects) arducam_ov5642_digital_camera.o 
	gcc -o ov5642_digital_camera $(objects) arducam_ov5642_digital_camera.o -lwiringPi -Wall
	
default_fonts_spi.o : default_fonts_spi.c
	gcc -c default_fonts_spi.c -lwiringPi -Wall
utft_spi.o : utft_spi.c
	gcc -c utft_spi.c -lwiringPi -Wall
arducam.o : arducam.c
	gcc -c arducam.c -lwiringPi -Wall
arducam_ov2640_playback.o : arducam_ov2640_playback.c
	gcc -c arducam_ov2640_playback.c -lwiringPi -Wall
arducam_ov3640_playback.o : arducam_ov3640_playback.c
	gcc -c arducam_ov3640_playback.c -lwiringPi -Wall
arducam_ov5642_playback.o : arducam_ov5642_playback.c
	gcc -c arducam_ov5642_playback.c -lwiringPi -Wall
arducam_ov7670_playback.o : arducam_ov7670_playback.c
	gcc -c arducam_ov7670_playback.c -lwiringPi -Wall
arducam_ov2640_digital_camera.o : arducam_ov2640_digital_camera.c
	gcc -c arducam_ov2640_digital_camera.c -lwiringPi -Wall
arducam_ov2640_capture.o : arducam_ov2640_capture.c
	gcc -c arducam_ov2640_capture.c -lwiringPi -Wall
arducam_ov5642_digital_camera.o : arducam_ov5642_digital_camera.c
	gcc -c arducam_ov5642_digital_camera.c -lwiringPi -Wall	

clean : 
	rm -f ov2640_playback ov3640_playback ov5642_playback ov7670_playback ov2640_digital_camera ov2640_capture ov5642_digital_camera $(objects) *.o
