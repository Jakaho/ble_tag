#ifndef ACCELEROMETER_DATA_H
#define ACCELEROMETER_DATA_H

#include <string.h>
#define BUFFER_SIZE (60 * 4)


#include <stdint.h>

uint8_t* get_buffer_data(size_t* out_size);

#endif