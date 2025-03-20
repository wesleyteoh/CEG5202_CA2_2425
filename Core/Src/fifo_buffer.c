#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define fifo_max_size 100

typedef enum {
    sense_temp,
    sense_humidity,
    sense_accel,
    sense_pressure,
    sense_magneto
} SensorType;

// Define the structure for each sensor type
typedef struct {
    float temperature;
    uint32_t timestamp; // Store the time of data collection
} tempData;

typedef struct {
    float humidity;
    uint32_t timestamp; // Store the time of data collection
} humidityData;

typedef struct {
    float accel[3]; // X, Y, Z
    uint32_t timestamp; // Store the time of data collection
} accelData;

typedef struct {
    float pressure;
    uint32_t timestamp; // Store the time of data collection
} pressureData;

typedef struct {
    float magneto[3]; // X, Y, Z
    uint32_t timestamp; // Store the time of data collection
} magnetoData;

// Separate FIFO buffers for each sensor
tempData temperature_buffer[fifo_max_size];
humidityData humidity_buffer[fifo_max_size];
accelData accel_buffer[fifo_max_size];
pressureData pressure_buffer[fifo_max_size];
magnetoData magneto_buffer[fifo_max_size];

// Indexes for FIFO buffers
uint16_t temp_head = 0, temp_tail = 0, temp_count = 0;
uint16_t humidity_head = 0, humidity_tail = 0, humidity_count = 0;
uint16_t accel_head = 0, accel_tail = 0, accel_count = 0;
uint16_t pressure_head = 0, pressure_tail = 0, pressure_count = 0;
uint16_t magneto_head = 0, magneto_tail = 0, magneto_count = 0;

// Temperature buffer input function
void add_to_temperature_buffer(float new_data, uint32_t current_time) {
    if (temp_count >= fifo_max_size) {
        return; // Discard data if buffer is full
    }
    SensorData *entry = &temperature_buffer[temp_head];
    entry->temperature = new_data;
    entry->timestamp = current_time;

    // Update FIFO index
    temp_head = (temp_head + 1) % fifo_max_size;
    temp_count++;
}

// Humidity buffer input function
void add_to_humidity_buffer(float new_data, uint32_t current_time) {
    if (humidity_count >= fifo_max_size) {
        return; // Discard data if buffer is full
    }
    SensorData *entry = &humidity_buffer[humidity_head];
    entry->humidity = new_data;
    entry->timestamp = current_time;

    // Update FIFO index
    humidity_head = (humidity_head + 1) % fifo_max_size;
    humidity_count++;
}

// Accelerometer buffer input function
void add_to_accel_buffer(float new_data[3], uint32_t current_time) {
    if (accel_count >= fifo_max_size) {
        return; // Discard data if buffer is full
    }
    SensorData *entry = &accel_buffer[accel_head];
    memcpy(entry->accel, new_data, sizeof(float) * 3);
    entry->timestamp = current_time;

    // Update FIFO index
    accel_head = (accel_head + 1) % fifo_max_size;
    accel_count++;
}

// Pressure buffer input function
void add_to_pressure_buffer(float new_data, uint32_t current_time) {
    if (pressure_count >= fifo_max_size) {
        return; // Discard data if buffer is full
    }
    SensorData *entry = &pressure_buffer[pressure_head];
    entry->pressure = new_data;
    entry->timestamp = current_time;

    // Update FIFO index
    pressure_head = (pressure_head + 1) % fifo_max_size;
    pressure_count++;
}

// Magnetometer buffer input function
void add_to_magneto_buffer(float new_data[3], uint32_t current_time) {
    if (magneto_count >= fifo_max_size) {
        return; // Discard data if buffer is full
    }
    SensorData *entry = &magneto_buffer[magneto_head];
    memcpy(entry->magneto, new_data, sizeof(float) * 3);
    entry->timestamp = current_time;

    // Update FIFO index
    magneto_head = (magneto_head + 1) % fifo_max_size;
    magneto_count++;
}

float get_temp_input_frequency() {
	return (HAL_GetTick - temperature_buffer[temp_tail].timestamp) / temp_count;
}

SensorType get_most_occupancy_buffer() {
	uint16_t highest_count = temp_count;
	SensorType max_occu_sense = sense_temp;
	if(highest_count < humidity_count) {
		highest_count = humidity_count;
		max_occu_sense = sense_humidity;
	}
	if(highest_count < accel_count) {
		highest_count = accel_count;
		max_occu_sense = sense_accel;
	}
	if(highest_count < pressure_count) {
		highest_count = pressure_count;
		max_occu_sense = sense_pressure;
	}
	if(highest_count < magneto_count) {
		highest_count = magneto_count;
		max_occu_sense = sense_magneto;
	}
//	return highest_count / fifo_max_size;
	return max_occu_sense;
}


