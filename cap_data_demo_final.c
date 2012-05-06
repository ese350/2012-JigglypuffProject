/**
 * Max Guo
 * Thomas Ly
 *
 * ESE350 Final Project - Sentry Gun
 * main.c
 *
 * compile with:
 * gcc -Wall -o cap_data cap_data.c -lm
 *
 * run:
 * ./cap_data
 */

//includes
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

//defines
#define DEG_TO_RAD 0.0174532925 //convert from degrees to radians, PI/180
#define LIDAR_PORT "/dev/ttyUSB0"
#define MBED_PORT "/dev/ttyUSB1"
#define NUM_PACKETS 98
#define PACKET_SIZE (2 * NUM_PACKETS)
#define CHECKSUM 613
#define THRESHOLD_DIST 1000 //vertical distance in cm, not radial
#define INF 100000
#define FRAMES 30
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
FILE *l_fd; //file descriptor for lidar port
unsigned int *reference; //initial reference frame

//function prototypes
int InitLidar();
void StartCap();
int CalcVertDist(int, int);
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
    int i;
    reference = (unsigned int *)calloc(NUM_PACKETS, sizeof(int));

    for (i = 0; i < NUM_PACKETS; i++) {
        reference[i] = INF;
    }

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
 *                      changes angle setting
 *
 * inputs: none
 * returns: int
 *     0 - successful execution
 *     1 - error occurred
 */
int InitLidar() {
    unsigned char *test_byte = (unsigned char *)calloc(1, sizeof(char));
    unsigned char *startup_msg = (unsigned char *)calloc(28, sizeof(char));
    unsigned char *change_buf = (unsigned char *)calloc(14, sizeof(char));
    unsigned char change_angle[] = {0x02, 0x00, 0x05, 0x00, 0x3B, 0x64, 0x00, 0x64, 0x00, 0x1D, 0x0F};

    //open connection to port
    l_fd = fopen(LIDAR_PORT, "rw+");
    if (!l_fd) {
        return 1;
    }
    fflush(l_fd);

    //read in start message, occasionally there is an extra byte
    //so we check for this first
    fread(test_byte, 1, 1, l_fd);
    while (test_byte[0] != 0x02) {
        fread(test_byte, 1, 1, l_fd);
    }
    fread(startup_msg, 1, 28, l_fd);

    //change angle setting to 100 degree view, 1 degree resolution
    fwrite(change_angle, 1, sizeof(change_angle), l_fd);
    fread(change_buf, 1, 14, l_fd);
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
    int j;
    unsigned int temp;
    int i = 0;
    int value = 0;
    unsigned char *start_buf = (unsigned char *)calloc(10, sizeof(char));
    unsigned char *data_buf = (unsigned char *)calloc(PACKET_SIZE, sizeof(char));
    unsigned char *test_byte = (unsigned char *)calloc(1, sizeof(char));
    unsigned char start_command[] = {0x02, 0x00, 0x02, 0x00, 0x20, 0x24, 0x34, 0x08};

    fwrite(start_command, 1, sizeof(start_command), l_fd);
    fread(start_buf, 1, 10, l_fd);
    
    //check for header
    while (1) {
        value = 0;
        fread(test_byte, 1, 1, l_fd);
        if (test_byte[0] == 0x02) {
            value += 2;
            fread(test_byte, 1, 1, l_fd);
            if (test_byte[0] == 0x80) {
                value += 128;
                fread(test_byte, 1, 1, l_fd);
                if (test_byte[0] == 0xCE) {
                    value += 206;
                    fread(test_byte, 1, 1, l_fd);
                    if (test_byte[0] == 0x00) {
                        value += 0;
                        fread(test_byte, 1, 1, l_fd);
                        if (test_byte[0] == 0xB0) {
                            value += 176;
                            fread(test_byte, 1, 1, l_fd);
                            if (test_byte[0] == 0x65) {
                                value += 101;
                                fread(test_byte, 1, 1, l_fd);
                            }
                        }
                    }
                }
            }
        }

        if (value == CHECKSUM) {
            fread(data_buf, 1, PACKET_SIZE, l_fd);
            packet = (DATA *)data_buf;

            if (i < FRAMES) {
                for (j = 0; j < NUM_PACKETS; j++) {
                    temp = (unsigned int)CalcVertDist(COMBINE(packet->data[j].low_byte, packet->data[j].high_byte), j + 40);
                    reference[j] = (temp < reference[j]) ? temp : reference[j];
                }

                i++;
            } else if (i == FRAMES) {
                for (j = 0; j < NUM_PACKETS; j++) {
                    if (reference[j] > THRESHOLD_DIST) {
                        reference[j] = THRESHOLD_DIST;
                    }
                    if (reference[j] > 10) {
                        reference[j] -= 10;
                    }
                }

                i++;
                ProcessPacket(packet);
            } else {
                ProcessPacket(packet);
            }
        }
    }
}

/**
 * CalcVertDist() method - calculates vertical distance to Lidar
 *                         from the distance and angle measurements
 *                         provided by the Lidar
 *
 * inputs: int
 *     rad_dist - distance data from lidar
 *     angle - corresponding angle to that point
 * returns: int - calculated vertical distance
 */
int CalcVertDist(int rad_dist, int angle) {
    return (int)((double)rad_dist * sin((double)angle * DEG_TO_RAD));
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
    int temp_dist;
    unsigned int dist_arr[NUM_PACKETS];
    unsigned char min = 0;
    unsigned char flag = 0;
    unsigned int min_dist = INF;
    
    FILE *m_fd = fopen(MBED_PORT, "w");
    fflush(m_fd);
    
    for (i = 0; i < NUM_PACKETS; i++) {
        temp_dist = CalcVertDist(COMBINE(d->data[i].low_byte, d->data[i].high_byte), i + 40);
        dist_arr[i] = ((temp_dist < ((int)reference[i])) && (temp_dist < THRESHOLD_DIST) && (temp_dist > 0)) ? (unsigned int)temp_dist : INF;
    }
    
    for (i = 0; i < NUM_PACKETS; i++) {
        if (dist_arr[i] < min_dist) {
            min_dist = dist_arr[i];
            min = (unsigned char)i;
        }
    }
    
    flag = ((((int)min_dist) < (int)reference[min]) && (min_dist < THRESHOLD_DIST) && (min_dist > 0));
    printf("%u %u %u %d\n\n", flag, min, min_dist, ((int)reference[min]));
    min |= (flag << 7);
    fwrite(&min, 1, 1, m_fd);
    fclose(m_fd);
}

