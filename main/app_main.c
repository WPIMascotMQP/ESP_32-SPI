/* SPI Slave example, receiver (uses SPI Slave driver to communicate with sender)

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

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

#ifdef CONFIG_IDF_TARGET_ESP32
#define RCV_HOST    HSPI_HOST
#define DMA_CHAN    2

#elif defined CONFIG_IDF_TARGET_ESP32S2
#define RCV_HOST    SPI2_HOST
#define DMA_CHAN    RCV_HOST

#endif

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

//Main application
void app_main(void)
{
    int n=0;
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
    gpio_config_t io_conf={
        .intr_type=GPIO_INTR_DISABLE,
        .mode=GPIO_MODE_OUTPUT,
        .pin_bit_mask=(1<<GPIO_HANDSHAKE)
    };

    //Configure handshake line as output
    gpio_config(&io_conf);
    //Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected.
    gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);

    //Initialize SPI slave interface
    ret=spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, DMA_CHAN);
    assert(ret==ESP_OK);

    WORD_ALIGNED_ATTR char sendbuf[8]="";
    WORD_ALIGNED_ATTR char recvbuf[8]="";
    memset(recvbuf, 0, 8);
    spi_slave_transaction_t t;
    memset(&t, 0, sizeof(t));

    while(1) {
        //Clear receive buffer, set send buffer to something sane
        memset(recvbuf, 0x0, 8);
      int bits;
      char flagInArry[8] = "00000000";
      unsigned char flag = 0;
      for ( bits = 0; bits < 8; ++bits )
          flag |= (flagInArry[bits] == '1') << (7 - bits);
       
      char cmdInArry[8] = "11111111";
      unsigned char cmd = 0;
      for ( bits = 0; bits < 8; ++bits )
          cmd |= (cmdInArry[bits] == '1') << (7 - bits);
       
       char dataInArry[128];
       unsigned char data[16];
       int dataByts;
       //for(dataByts; dataByts < 16; ++dataByts){
       //  for ( bits = 0; bits < 8; ++bits ){
       //     data[dataByts] |= (dataInArry[bits] == '1') << (7 - bits);
       //  }
       // }
       
         
        sprintf(sendbuf, "Moved");

        //Set up a transaction of 128 bytes to send/receive
        t.length=5*8;
        t.tx_buffer=sendbuf;
        t.rx_buffer=recvbuf;
        /* This call enables the SPI slave interface to send/receive to the sendbuf and recvbuf. The transaction is
        initialized by the SPI master, however, so it will not actually happen until the master starts a hardware transaction
        by pulling CS low and pulsing the clock etc. In this specific example, we use the handshake line, pulled up by the
        .post_setup_cb callback that is called as soon as a transaction is ready, to let the master know it is free to transfer
        data.
        */
        ret=spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY);

        //spi_slave_transmit does not return until the master has done a transmission, so by here we have sent our data and
        //received data from the master. Print it.
        printf("Received: %s\n", recvbuf);
        n++;
        //motorMove(1000,1500,1200,3000);

    }

}

class Pulser
{

    long highTime; // micoseconds of HIGH
    long lowTime; // microseconds of LOW

    int serPin;    
    int startState;  
    int dirPin;  

  public:
  unsigned long previousMicro; // last time servo updated
  
  public:
  Pulser(int spin, int dPin, long high, long low)
  {
    serPin = spin;
    pinMode(serPin, OUTPUT);
    dirPin = dPin;     
    pinMode(dirPin, OUTPUT);
          
    highTime = high;
    lowTime = low;
        
    startState = LOW; 
    previousMicro = 0;
  }
     
  void Update()
  {
    unsigned long currentMicro = micros();
         
    if((startState == HIGH) && (currentMicro - previousMicro >= highTime))
    {
      previousMicro = currentMicro;  
      digitalWrite(serPin, startState);
      startState = LOW; 
    }
    else if ((startState == LOW) && (currentMicro - previousMicro >= lowTime))
    {
      previousMicro = currentMicro;
      digitalWrite(serPin, startState);
      startState = HIGH;

    }
  }
};
//Setting pins and speeds
Pulser Pulser1(6, 7, 5, 5);
Pulser Pulser2(5, 8, 5, 5);

void motorMove(unsigned long mot1, unsigned long mot2, unsigned long mot3, unsigned long mot4){ 

  
    if(currentstep1<mot1){        
        digitalWrite(Pulser1.dirPin, HIGH);
        dir1 = true;
    }
    else if(currentstep1>mot1){
        digitalWrite(Pulser1.dirPin, LOW);
        dir1 = false;
    }
    if(currentstep2<mot2){
        digitalWrite(Pulser2.dirPin, HIGH);
        dir2 = true;
    }
    else if(currentstep2>mot2){
        digitalWrite(Pulser2.dirPin, LOW);
        dir2 = false;
    }

    if(currentstep1 != mot1){
        Pulser1.Update();
        Pulser1.Update();
        
        if(dir1 == true){
          currentstep1 = currentstep1 - 1;}
        if(dir1 == false){
          currentstep1 = currentstep1 + 1;}
  
    }
    if(currentstep2 != mot2){
        Pulser2.Update();
        Pulser2.Update();
        
        
        if(dir2 == true){
          currentstep2 = currentstep2 - 1;}
        if(dir2 == false){
          currentstep2 = currentstep2 + 1;}
        
    }
}
void rpmToMicroDelay(){
    int StepRate = 12800;
    int rpm = 10;
    unsigned long HighTimeCalc = (StepRate * (60/rpm) * (10^6)) - 5; 
}

void radToStep(){
 int rad;
 int turns = (rad/2*M_PI)/12800;
}

