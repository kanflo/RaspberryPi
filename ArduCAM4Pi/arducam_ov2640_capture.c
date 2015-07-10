/* 
 * Copyright (c) 2015 Johan Kanflo
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

// Based on PiCAM_OV2640_DigitalCamera.c

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <wiringPiSPI.h>
#include <unistd.h>
#include "arducam.h"

#define OV2640_CHIPID_HIGH  0x0A
#define OV2640_CHIPID_LOW   0x0B

#define BUF_SIZE (128*1024)

void setup()
{
    uint8_t vid,pid;
    uint8_t temp;

    arducam(smOV2640);

    // Check if the ArduCAM SPI bus is OK
    arducam_write_reg(ARDUCHIP_TEST1, 0x55);
    temp = arducam_read_reg(ARDUCHIP_TEST1);
    if(temp != 0x55) {
        printf("SPI interface error!\n");
        exit(EXIT_FAILURE);
    }

    // Change MCU mode
    arducam_write_reg(ARDUCHIP_MODE, 0x00);

    // Check if the camera module type is OV2640
    arducam_i2c_read(OV2640_CHIPID_HIGH, &vid);
    arducam_i2c_read(OV2640_CHIPID_LOW, &pid);
    if((vid != 0x26) || (pid != 0x42)) {
        printf("Can't find OV2640 module!\n");
        exit(EXIT_FAILURE);
    } else {
        printf("OV2640 detected\n");
    }
}

int main(int argc, char *argv[])
{
    uint8_t buf[BUF_SIZE];
    int i = 0;
    uint8_t temp, temp_last;

    if (argc == 1) {
        printf("Usage: %s [-s <resolution>] | [-c <filename]", argv[0]);
        printf(" -s <resolution> Set resolution, valid resolutions are:\n");
        printf("                   160x120\n");
        printf("                   176x144\n");
        printf("                   320x240\n");
        printf("                   352x288\n");
        printf("                   640x480\n");
        printf("                   800x600\n");
        printf("                   1024x768\n");
        printf("                   1280x1024\n");
        printf("                   1600x1200\n");
        printf(" -c <filename>   Capture image\n");
        exit(EXIT_SUCCESS);
    }

    if (strcmp(argv[1], "-s") == 0 && argc == 3) {
        setup();
        // Change to JPEG capture mode and initialize the OV2640 module
        arducam_set_format(fmtJPEG);
        arducam_init();

        if (strcmp(argv[2], "160x120") == 0) arducam_set_jpeg_size(sz160x120);
        else if (strcmp(argv[2], "176x144") == 0) arducam_set_jpeg_size(sz176x144);
        else if (strcmp(argv[2], "320x240") == 0) arducam_set_jpeg_size(sz320x240);
        else if (strcmp(argv[2], "352x288") == 0) arducam_set_jpeg_size(sz352x288);
        else if (strcmp(argv[2], "640x480") == 0) arducam_set_jpeg_size(sz640x480);
        else if (strcmp(argv[2], "800x600") == 0) arducam_set_jpeg_size(sz800x600);
        else if (strcmp(argv[2], "1024x768") == 0) arducam_set_jpeg_size(sz1024x768);
        else if (strcmp(argv[2], "1280x1024") == 0) arducam_set_jpeg_size(sz1280x1024);
        else if (strcmp(argv[2], "1600x1200") == 0) arducam_set_jpeg_size(sz1600x1200);
        else {
            printf("Unknown resolution %s\n", argv[2]);
            exit(EXIT_FAILURE);
        }
        sleep(1); // Let auto exposure do it's thing after changing image settings
        printf("Changed resolution to %s\n", argv[2]);
        exit(EXIT_SUCCESS);
    } else if (strcmp(argv[1], "-c") == 0 && argc == 3) {
        setup();
        // Flush the FIFO
        arducam_flush_fifo();
        // Clear the capture done flag
        arducam_clear_fifo_flag();
        // Start capture
        printf("Start capture\n");
        arducam_start_capture();
        while (!(arducam_read_reg(ARDUCHIP_TRIG) & CAP_DONE_MASK)) ;
        printf(" Done\n");

        // Open the new file
        FILE *fp = fopen(argv[2], "w+");
        if (!fp) {
            printf("Error: could not open %s\n", argv[2]);
            exit(EXIT_FAILURE);
        }

        printf("Reading FIFO\n");
        i = 0;
        temp = arducam_read_fifo();
        // Write first image data to buffer
        buf[i++] = temp;

        // Read JPEG data from FIFO
        while((temp != 0xD9) | (temp_last != 0xFF)) {
            temp_last = temp;
            temp = arducam_read_fifo();
            // Write image data to buffer if not full
            if(i < BUF_SIZE) {
                buf[i++] = temp;
            } else {
                // Write BUF_SIZE uint8_ts image data to file
                fwrite(buf, BUF_SIZE, 1, fp);
                i = 0;
                buf[i++] = temp;
            }
        }
        // Write the remain uint8_ts in the buffer
        if(i > 0) {
            fwrite(buf, i, 1, fp);
        }

        // Close the file
        fclose(fp);

        // Clear the capture done flag
        arducam_clear_fifo_flag();
    } else {
        printf("Error: unknown or missing argument.\n");
        exit(EXIT_FAILURE);
    }
    exit(EXIT_SUCCESS);
}
