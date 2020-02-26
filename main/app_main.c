/* SPI Slave example, receiver (uses SPI Slave driver to communicate with sender)

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/igmp.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "soc/rtc_periph.h"
#include "driver/spi_slave.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "esp_timer.h"
#include "driver/gpio.h"

/*
SPI receiver (slave) example.

This example is supposed to work together with the SPI sender. It uses the standard SPI pins (MISO, MOSI, SCLK, CS) to
transmit data over in a full-duplex fashion, that is, while the master puts data on the MOSI pin, the slave puts its own
data on the MISO pin.

This example uses one extra pin: GPIO_HANDSHAKE is used as a handshake pin. After a transmission has been set up and we're
ready to send/receive data, this code uses a callback to set the handshake pin high. The sender will detect this and start
sending a transaction. As soon as the transaction is done, the line gets set low again.
*/

/*
Pins in use. The SPI Master can use the GPIO mux, so feel free to change these if needed.
*/
#define GPIO_HANDSHAKE 2
#define GPIO_MOSI 12
#define GPIO_MISO 13
#define GPIO_SCLK 15
#define GPIO_CS 14

#define PATTERN_LEN 10

#ifdef CONFIG_IDF_TARGET_ESP32
#define RCV_HOST    HSPI_HOST
#define DMA_CHAN    2

#elif defined CONFIG_IDF_TARGET_ESP32S2
#define RCV_HOST    SPI2_HOST
#define DMA_CHAN    RCV_HOST

#endif

#define STEP_DELAY 75
#define LOW 0
#define HIGH 1

// Immediate variable type between float and long
typedef long float_i;

// Flags for the flag byte of each command
enum flags {
	FLAG1 = 0x80, // 10000000
	FLAG2 = 0x40, // 01000000
	FLAG3 = 0x20, // 00100000
	FLAG4 = 0x10, // 00010000
	FLAG5 = 0x08, // 00001000
	FLAG6 = 0x04, // 00000100
	FLAG7 = 0x02, // 00000010
	FLAG8 = 0x01, // 00000001
};

// Holds the hex representation of each command
enum cmds {
	PATTERN = 0xFF,
	MOTORPOSITION = 0x01,
};

// Holds the byte index for different parts of the commmand
enum byte_index {
	flag_byte = 0,
	cmd_byte = 1,
};

bool dir1;
bool dir2;
int currentstep1 = 0;
int currentstep2 = 0;

//Called after a transaction is queued and ready for pickup by master. We use this to set the handshake line high.
void my_post_setup_cb(spi_slave_transaction_t *trans) {
    WRITE_PERI_REG(GPIO_OUT_W1TS_REG, (1<<GPIO_HANDSHAKE));
}

//Called after transaction is sent/received. We use this to set the handshake line low.
void my_post_trans_cb(spi_slave_transaction_t *trans) {
    WRITE_PERI_REG(GPIO_OUT_W1TC_REG, (1<<GPIO_HANDSHAKE));
}


// Function Declarations
void handleCommand(char* buffer);
bool findCommand(char* buffer);
char* getStringHex(char* buffer, char* output, size_t length);

static void doStep(void* arg);

size_t overwriteBytes(char* buffer, size_t byte_start, char* buf, size_t inc_start, size_t byte_inc);
size_t encodePattern(char* buffer, size_t byte_start);
size_t encodeInt16(char* buffer, size_t byte_start, int16_t num);
size_t encodeInt32(char* buffer, size_t byte_start, int32_t num);
size_t encodeFloat(char* buffer, size_t byte_start, float num);

int16_t decodeInt16(char* buffer, size_t byte_start);
int32_t decodeInt32(char* buffer, size_t byte_start);
float decodeFloat(char* buffer, size_t byte_start);

// Global Varaibles
DMA_ATTR static uint8_t BUFFER_SIZE = 64;
DMA_ATTR static int64_t ACC_MAX = 5;
DMA_ATTR static uint8_t MOTOR_NUM = 4;

DMA_ATTR int64_t stepper = 0;
DMA_ATTR int64_t acceleration = 0;

DMA_ATTR int64_t motor_positions[4];
DMA_ATTR int64_t motor_setpoints[4];
DMA_ATTR int64_t motor_differents[4];

DMA_ATTR gpio_num_t STEPS[4] = {GPIO_NUM_32, GPIO_NUM_25, GPIO_NUM_5, GPIO_NUM_16};
DMA_ATTR gpio_num_t DIRS[4] = {GPIO_NUM_33, GPIO_NUM_33, GPIO_NUM_17, GPIO_NUM_4};

DMA_ATTR char sendbuf[65] = "";
DMA_ATTR char recvbuf[65] = "";
DMA_ATTR char outputbuf[65] = "";

//Main application
void app_main(void)
{
    esp_err_t ret;

    //Configuration for the SPI bus
    spi_bus_config_t buscfg={
        .mosi_io_num=GPIO_MOSI,
        .miso_io_num=GPIO_MISO,
        .sclk_io_num=GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    //Configuration for the SPI slave interface
    spi_slave_interface_config_t slvcfg={
        .mode=0,
        .spics_io_num=GPIO_CS,
        .queue_size=3,
        .flags=0,
        .post_setup_cb=my_post_setup_cb,
        .post_trans_cb=my_post_trans_cb
    };

    //Configuration for the handshake line
    gpio_config_t io_conf = {
        .intr_type = GPIO_PIN_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = ((1ULL << GPIO_NUM_32) | (1ULL << GPIO_NUM_25) | (1ULL << GPIO_NUM_5) | (1ULL << GPIO_NUM_16)
			| (1ULL << GPIO_NUM_33) | (1ULL << GPIO_NUM_26) | (1ULL << GPIO_NUM_17) | (1ULL << GPIO_NUM_4)),
		.pull_down_en = 0,
		.pull_up_en = 0
    };

    //Configure handshake line as output
    gpio_config(&io_conf);

	//Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected.
    gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);

	for(int i = 0; i < MOTOR_NUM; i++) {
		gpio_pad_select_gpio(STEPS[i]);
		gpio_set_direction(STEPS[i], GPIO_MODE_OUTPUT);

		gpio_pad_select_gpio(DIRS[i]);
		gpio_set_direction(DIRS[i], GPIO_MODE_OUTPUT);
	}

	const esp_timer_create_args_t timer_args = {
            .callback = &doStep,
            /* name is optional, but may help identify the timer when debugging */
            .name = "periodic"
    };
	esp_timer_handle_t timer;
	ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
	ESP_ERROR_CHECK(esp_timer_start_periodic(timer, STEP_DELAY));

	stepper = 0;
	acceleration = 1;

    //Initialize SPI slave interface
    ret=spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, DMA_CHAN);
    assert(ret==ESP_OK);
    //https://www.quora.com/What-is-word-Alignment
    memset(recvbuf, 0x00, BUFFER_SIZE);
    spi_slave_transaction_t t;
    memset(&t, 0, sizeof(t));

	srand(time(NULL));

    while(1) {
        //Clear receive buffer, set send buffer to something sane
        memset(recvbuf, 0x00, BUFFER_SIZE);
		memset(sendbuf, 0x00, BUFFER_SIZE);

		size_t current_byte = 0;
		current_byte += encodePattern(sendbuf, current_byte);
		sendbuf[current_byte++] = 0x00;
		sendbuf[current_byte++] = MOTORPOSITION;
		int index = rand() % 20;
		float radians = (rand() % 12800) + ((0.0 + rand() % 100) / 100000);
		current_byte += encodeInt16(sendbuf, current_byte, index);
        current_byte += encodeFloat(sendbuf, current_byte, radians);
        current_byte += encodePattern(sendbuf, current_byte);

		printf("Sending Position: %d %f\n", index, radians);
		printf("Sending: %s\n", getStringHex(sendbuf, outputbuf, BUFFER_SIZE));

        //Set up a transaction of 128 bytes to send/receive
        t.length = BUFFER_SIZE * 8;
        t.tx_buffer = sendbuf;
        t.rx_buffer = recvbuf;
        /* This call enables the SPI slave interface to send/receive to the sendbuf and recvbuf. The transaction is
        initialized by the SPI master, however, so it will not actually happen until the master starts a hardware transaction
        by pulling CS low and pulsing the clock etc. In this specific example, we use the handshake line, pulled up by the
        .post_setup_cb callback that is called as soon as a transaction is ready, to let the master know it is free to transfer
        data.
        */
        ret = spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY);

        //spi_slave_transmit does not return until the master has done a transmission, so by here we have sent our data and
        //received data from the master. Print it.
        //printf("Received: %s\n", getStringHex(recvbuf, outputbuf, BUFFER_SIZE));

		// TEST IF COMPLETE
		if(findCommand(recvbuf)) {
			handleCommand(recvbuf);
		}
        //motorMove(1000,1500,1200,3000);
    }
	esp_timer_delete(timer);

}

/**
 Handles the command in the given buffer
 @param buffer The buffer where the command is (must start at beginning of buffer)
 */
void handleCommand(char* buffer) {
	//WORD_ALIGNED_ATTR char outputbuf[BUFFER_SIZE] = "";
  	//printf("Received Command: %s\n", getStringHex(buffer, outputbuf, BUFFER_SIZE));
	if(buffer[cmd_byte] == MOTORPOSITION) {
		// If command is the current motor position
		size_t current_byte = 2;
		int index = (int) decodeInt16(buffer, current_byte);
		current_byte += 2;
		double radians = (double) decodeFloat(buffer, current_byte);
		int steps = (int) radians;
		printf("Received Motor Radians: %d %f\n", index, radians);
		motor_setpoints[index] = steps;
		motor_differents[index] = abs(motor_setpoints[index] - motor_positions[index]);
	}
}

static void doStep(void* arg) {
	for(int index = 0; index < MOTOR_NUM; index++) {
		if(acceleration != 0 && motor_positions[index] != motor_setpoints[index]) {
			stepper++;
			if (motor_positions[index] > motor_setpoints[index]){
				gpio_set_level(DIRS[index], LOW);
				gpio_set_level(STEPS[index], stepper % 2);
				if(stepper % 2) {
					motor_positions[index] -= 1;
				}
		 	} else if (motor_positions[index] < motor_setpoints[index]){
				gpio_set_level(DIRS[index], HIGH);
				gpio_set_level(STEPS[index], stepper % 2);
				if(stepper % 2) {
					motor_positions[index] += 1;
				}
		    }
			acceleration--;
		} else if(motor_positions[index] != motor_setpoints[index]) {
			int64_t difference = abs(motor_setpoints[index] - motor_positions[index]);
			if(difference <= motor_differents[index] / 2) {
				float ratio = (0.0 + difference) / (motor_differents[index] / 2);
				acceleration = round(ratio * ACC_MAX + 0.5);
			} else {
				float ratio = (0.0 + difference - (motor_differents[index] / 2)) / (motor_differents[index] / 2);
				acceleration = round(ACC_MAX - (ratio * ACC_MAX) + 0.5);
			}
		} else {
		   gpio_set_level(STEPS[index], LOW);
	   }
   }
}

/**
 Finds a valid command in the given buffer
 If a command was found, writies that command into the beginning of the buffer
 @param buffer
 @return Whether a command was found or not
 */
bool findCommand(char* buffer) {
	//printf("Buffer: %s\n", getStringHex(buffer, outputbuf, BUFFER_SIZE));

	WORD_ALIGNED_ATTR char buf[BUFFER_SIZE];
	overwriteBytes(buf, 0, buffer, 0, BUFFER_SIZE);

	size_t patterns[2];
	size_t lengths[2];
	size_t start_bypte = 0;
	size_t counter = 0;
  	size_t current = 0;
	bool on_one = false;

	// Parse throught each byte in the buffer
	for(size_t i = 0; i < BUFFER_SIZE; i++) {
		// If found pattern byte
		if(buffer[i] == PATTERN) {
			// If not on pattern yet
			if(!on_one) {
				on_one = true;
				start_bypte = i;
				counter++;
			// If on pattern add to counter
			} else if(on_one) {
				counter++;
			}
		} else {
			// If on one but less length or longer than pattern length
			if(on_one && (counter < PATTERN_LEN || counter > PATTERN_LEN)) {
				start_bypte = 0;
				counter = 0;
				on_one = false;
			}
			// If on one which is exactly  pattern length
			if(on_one && counter == PATTERN_LEN) {
				patterns[current] = start_bypte;
				lengths[current] = counter;
				start_bypte = 0;
				counter = 0;
				on_one = false;
		        current++;
		        if(current > 2) {
		          return false;
		        }
			}
		}
	}
	printf("%d\n", current);
	if(current == 2) {
		overwriteBytes(buffer, 0, buf, patterns[0] + lengths[0],
			patterns[1] - patterns[0] - lengths[0]);
		return true;
	}
	return false;
}

/**
 Gets the std::string represention of the given buffer
 @param buffer The given buffer to represent
 @param length The length of the buffer to represent
 @return The std::string hex value
 */
char* getStringHex(char* buffer, char* output, size_t length) {
	WORD_ALIGNED_ATTR char hex[length * 2 + 1];
	for(size_t i = 0; i < length; i++) {
   		sprintf(hex + 2 * i, "%.2x", buffer[i]);
	}
	hex[length * 2 + 1] = '\0';

	sprintf(output, "0x%s", hex);
	return output;
}

/**
 Overwrites the bytes in the given buffer with bytes from the other given buffer
 @param buffer The buffer to write into
 @param byte_start The starting location to write to
 @param buf The buffer to write from
 @param int_start The starting location to write from
 @param byte_inc The number of bytes to write
 @return The number of bytes written
*/
size_t overwriteBytes(char* buffer, size_t byte_start,
  char* buf, size_t inc_start, size_t byte_inc) {
	for(size_t i = 0; i < byte_inc; i++) {
		buffer[byte_start + i] = buf[inc_start + i];
	}
	return byte_inc;
}

/**
 Writes the pattern into the given buffer at the given position
 @param buffer The buffer to write into
 @param byte_start The starting byte to writing into
 @return The number of bytes written
 */
size_t encodePattern(char* buffer, size_t byte_start) {
	buffer[byte_start + 0] = PATTERN;
	buffer[byte_start + 1] = PATTERN;
	buffer[byte_start + 2] = PATTERN;
	buffer[byte_start + 3] = PATTERN;
	buffer[byte_start + 4] = PATTERN;
	buffer[byte_start + 5] = PATTERN;
	buffer[byte_start + 6] = PATTERN;
	buffer[byte_start + 7] = PATTERN;
	buffer[byte_start + 8] = PATTERN;
	buffer[byte_start + 9] = PATTERN;
	return PATTERN_LEN;
}

/**
 Encodes a 16 bit int into the given buffer
 @param buffer The buffer to write into
 @param byte_start The starting byte to write into
 @param num The 16 bit int to encode
 @return The number of bytes encoded
 */
size_t encodeInt16(char* buffer, size_t byte_start, int16_t num) {
	buffer[byte_start + 0] = (num >> 8) & 0xFF;
	buffer[byte_start + 1] = num & 0xFF;
	return sizeof(int16_t);
}

/**
 Encodes a 32 bit int into the given buffer
 @param buffer The buffer to write into
 @param byte_start The starting byte to write into
 @param num The 32 bit int to encode
 @return The number of bytes encoded
 */
size_t encodeInt32(char* buffer, size_t byte_start, int32_t num) {
	buffer[byte_start + 0] = (num >> 24) & 0xFF;
	buffer[byte_start + 1] = (num >> 16) & 0xFF;
	buffer[byte_start + 2] = (num >> 8) & 0xFF;
	buffer[byte_start + 3] = num & 0xFF;
	return sizeof(int32_t);
}

/**
 Encodes a 32 bit float into the given buffer
 @param buffer The buffer to write into
 @param byte_start The starting byte to write into
 @param num The 32 bit float to encode
 @return The number of bytes encoded
 */
size_t encodeFloat(char* buffer, size_t byte_start, float num) {
	float_i number = *(float_i*) &num;
	buffer[byte_start + 0] = (number >> 24) & 0xFF;
	buffer[byte_start + 1] = (number >> 16) & 0xFF;
	buffer[byte_start + 2] = (number >> 8) & 0xFF;
	buffer[byte_start + 3] = number & 0xFF;
	return sizeof(float);
}

/**
 Decodes a 16 bit int from given buffer
 @param buffer The buffer to read from
 @param byte_start The starting byte to read from
 @return The 16 bit int decoded
 */
int16_t decodeInt16(char* buffer, size_t byte_start) {
	WORD_ALIGNED_ATTR char buf[2];
	buf[0] = buffer[byte_start + 1];
	buf[1] = buffer[byte_start + 0];
	int16_t number = *(int16_t*) &buf;

	return number;
}

/**
 Decodes a 32 bit int from given buffer
 @param buffer The buffer to read from
 @param byte_start The starting byte to read from
 @return The 32 bit int decoded
 */
int32_t decodeInt32(char* buffer, size_t byte_start) {
	WORD_ALIGNED_ATTR char buf[4];
	buf[0] = buffer[byte_start + 3];
	buf[1] = buffer[byte_start + 2];
	buf[2] = buffer[byte_start + 1];
	buf[3] = buffer[byte_start + 0];
	int32_t number = *(int32_t*) &buf;

	return number;
}

/**
 Decodes a float from given buffer
 @param buffer The buffer to read from
 @param byte_start The starting byte to read from
 @return The float decoded
 */
float decodeFloat(char* buffer, size_t byte_start) {
	WORD_ALIGNED_ATTR char buf[4];
	buf[0] = buffer[byte_start + 3];
	buf[1] = buffer[byte_start + 2];
	buf[2] = buffer[byte_start + 1];
	buf[3] = buffer[byte_start + 0];
	float number = *(float*) &buf;

	return number;
}
