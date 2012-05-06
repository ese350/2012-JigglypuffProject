/**
 * Max Guo
 * Thomas Ly
 *
 * ESE350 Final Project - Sentry Gun
 * main.c
 */

//includes
#include <stdio.h>
#include <stdlib.h>

//defines
#define PORT "/dev/ttyUSB0" //port at which lidar connects to
#define NUM_PACKETS 98
#define PACKET_SIZE (2 * NUM_PACKETS)
#define CHECKSUM 613
#define COMBINE(L, H) (((((unsigned int)H) << 8) | (unsigned int)L) & 0x1FFF)

//structs
#pragma pack(push, 1) //align memory to 1 byte
typedef struct {
    unsigned char low_byte;
    unsigned char high_byte;
} data_point;

typedef struct {
    data_point data[NUM_PACKETS];
} DATA;
#pragma pack(pop)

//global variables
FILE *fd; //file descriptor for lidar port

//function prototypes
int InitLidar();
void StartCap();
void ProcessPacket(DATA*);

/**
 * main() method - opens up a connection to lidar then captures data from lidar
 *
 * inputs: none
 * returns: int
 *     0 - successful execution
 *     1 - error occurred
 */
int main() {
    printf("Starting Lidar... ");
    if (InitLidar()) {
        printf("InitLidar() error.\n");
        return 1;
    }
    printf("done.\n");
    
    printf("Starting data capture.\n");
    printf("Press Ctrl-C to end program.\n");
    StartCap();
    return 0;
}

/**
 * InitLidar() method - initializes connection to lidar port, then
 *                      initializes angle setting
 *
 * inputs: none
 * returns: int
 *     0 - successful execution
 *     1 - error occurred
 */
int InitLidar() {
    int i;
    unsigned char *test_byte = (unsigned char *)calloc(1, sizeof(char));
    unsigned char *startup_msg = (unsigned char *)calloc(28, sizeof(char));
    unsigned char *change_buf = (unsigned char *)calloc(14, sizeof(char));
    unsigned char change_angle[] = {0x02, 0x00, 0x05, 0x00, 0x3B, 0x64, 0x00, 0x64, 0x00, 0x1D, 0x0F};

    //open connection to port
    fd = fopen(PORT, "rw+");
    if (!fd) {
        return 1;
    }
    fflush(fd);

    //read in start message, occasionally there is an extra byte
    //so we check for this first
    fread(test_byte, 1, 1, fd);
    while (test_byte[0] != 0x02) {
        fread(test_byte, 1, 1, fd);
    }
    fread(startup_msg, 1, 28, fd);

    //change angle setting to 100 degree view, 1 degree resolution
    fwrite(change_angle, 1, sizeof(change_angle), fd);
    fread(change_buf, 1, 14, fd);
    return 0;
}

/**
 * StartCap() method - data capture method, gets data from lidar,
 *                     if data misaligns, lidar restarts data stream
 *
 * inputs: none
 * returns: void
 */
void StartCap() {
    DATA *packet;
    int i = 0;
    int value = 0;
    unsigned char *start_buf = (unsigned char *)calloc(10, sizeof(char));
    unsigned char *data_buf = (unsigned char *)calloc(PACKET_SIZE, sizeof(char));
    unsigned char *test_byte = (unsigned char *)calloc(1, sizeof(char));
    unsigned char start_command[] = {0x02, 0x00, 0x02, 0x00, 0x20, 0x24, 0x34, 0x08};

    fwrite(start_command, 1, sizeof(start_command), fd);
    fread(start_buf, 1, 10, fd);
    
    //check for header
    while (1) {
        value = 0;
        fread(test_byte, 1, 1, fd);
        if (test_byte[0] == 0x02) {
            value += 2;
            fread(test_byte, 1, 1, fd);
            if (test_byte[0] == 0x80) {
                value += 128;
                fread(test_byte, 1, 1, fd);
                if (test_byte[0] == 0xCE) {
                    value += 206;
                    fread(test_byte, 1, 1, fd);
                    if (test_byte[0] == 0x00) {
                        value += 0;
                        fread(test_byte, 1, 1, fd);
                        if (test_byte[0] == 0xB0) {
                            value += 176;
                            fread(test_byte, 1, 1, fd);
                            if (test_byte[0] == 0x65) {
                                value += 101;
                                fread(test_byte, 1, 1, fd);
                            }
                        }
                    }
                }
            }
        }

        if (value == CHECKSUM) {
            fread(data_buf, 1, PACKET_SIZE, fd);
            //printf("packet %d received\n", i);
            //i++;
            packet = (DATA *)data_buf;

            ProcessPacket(packet);

            //for (i = 0; i < NUM_PACKETS; i++) {
            //    printf("%d ", COMBINE(packet->data[i].low_byte, packet->data[i].high_byte));
            //}
            //printf("\n\n");
        }        
    }
}

/**
 * ProcessPacket() method - processes data, sends minimum distance and angle
 *                          as well as the command to fire
 *
 * inputs: DATA * - data packet to process
 * returns: void
 */
void ProcessPacket(DATA *d) {
    int i;
    int min_index;

    printf("hi\n");
}

