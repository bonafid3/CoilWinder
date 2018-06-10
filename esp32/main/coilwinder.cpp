#include <stdio.h>
#include <string>
#include <iostream>
#include <list>
#include <vector>
#include <set>
#include <unordered_set>

#include <freertos/FreeRTOS.h>

#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>

#include <nvs_flash.h>
#include <nvs.h>

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/timer.h>

#include <cmath>

#include <lwip/sockets.h>
#include <lwip/netdb.h>

#include <cstring>
#include <string>

#include <sys/time.h>

#include <rom/ets_sys.h>

#include <esp_event.h>
#include <esp_event_loop.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <esp_wifi_types.h> 

#include "sdkconfig.h"

#define STATUS_PIN GPIO_NUM_21

#define LED_PIN GPIO_NUM_13

#define STEP1_PIN GPIO_NUM_26
#define DIR1_PIN GPIO_NUM_27
#define STEP2_PIN GPIO_NUM_14
#define DIR2_PIN GPIO_NUM_12

xQueueHandle gCmdQueue;
xQueueHandle gNetQueue;

SemaphoreHandle_t gMutex = nullptr;

const int CONNECTED_BIT = BIT0;

static struct sockaddr_in bcast;

static EventGroupHandle_t wifi_event_group;

volatile bool gLaserOn = false;

volatile int gMainRPM=0;
volatile int gCoilLengthSteps=0;
volatile int gCnt1=0;

volatile uint64_t gTickCount=0;
volatile uint64_t gAlarmCount=0;

volatile float gX=0;
volatile float gY=0;
volatile float gXAccum=0;
volatile float gYAccum=0;
volatile float gE=0;

volatile int gDesiredDelay0=0;
volatile int gCurrentDelay0=0;
volatile int gDelayModifier=0;

volatile uint32_t gIntensity = 32;

#define STEPS_PER_MM (40.0f/6400.0f)

#define DELAY_BETWEEN_STEPS 10000

#define INV_OUT

#define MAX_DUTY (1 << LEDC_TIMER_10_BIT)

struct sMotion
{
  enum eState { IDLE, ACC, DEC, MAXSPD };
  eState state;
  int stepsUsed;
};

struct sCommand {
    enum CmdType { cmdSTOP, cmdSTART };
    sCommand(){}
    sCommand(CmdType t):type(t){}
    char pkt[3] = {'p', 'k', 't'};
    char type;
    uint32_t mainRPM=0;
    float wireDiameter=0;
    uint32_t coilLength=0;
    uint32_t direction=0;
};

struct sCommandReply {
  char data[4];
};

volatile sMotion gMotion;

std::unordered_set<int> mStateChanges;
std::list<sCommand> mCommands;

#define forever while(1)


class cFreeRTOS {
public:
  cFreeRTOS(){}
  ~cFreeRTOS(){}
  static void startTask(void task(void *), std::string taskName, void *param=nullptr, int stackSize = 4096) {
    ::xTaskCreate(task, taskName.data(), stackSize, param, 10, NULL);
  }
};


float distanceToSteps(float dist) {
  //printf("steps per mm %f\n", STEPS_PER_MM);
  return dist / STEPS_PER_MM;
}


void pwmInit()
{
    ledc_channel_config_t ledc_channel_left = {};
    ledc_channel_left.gpio_num = LED_PIN;
    ledc_channel_left.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_channel_left.channel = LEDC_CHANNEL_0;
    ledc_channel_left.intr_type = LEDC_INTR_DISABLE;
    ledc_channel_left.timer_sel = LEDC_TIMER_2;
    ledc_channel_left.duty = 0;

    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_timer.bit_num = LEDC_TIMER_10_BIT;
    ledc_timer.timer_num = LEDC_TIMER_2;
    ledc_timer.freq_hz = 500;

    ledc_channel_config(&ledc_channel_left);
    ledc_timer_config(&ledc_timer);
}


void pwmSet(uint32_t duty)
{
  //printf("max duty: %d, duty: %d\n", MAX_DUTY, duty);

  #ifdef INV_OUT
    duty = MAX_DUTY - duty;
  #endif

  ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
} 


int createAndBindSocket() {
    int socket, ret;

    socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket < 0) {
        std::cout << "Failed to create UDP socket!" << std::endl;
        return -1;
    }
    std::cout << "UDP socket created: " << socket << ", bind..." << std::endl;

    struct sockaddr_in sock_addr;
    memset(&sock_addr, 0, sizeof(sock_addr));
    sock_addr.sin_family = AF_INET;
    sock_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    sock_addr.sin_port = htons(8080);
    ret = bind(socket, (struct sockaddr*)&sock_addr, sizeof(sock_addr));
    if (ret) {
        std::cout << "Failed to bind to UDP socket!" << std::endl;
        return -1;
    }
    std::cout << "UDP socket bound!" << std::endl;

    return socket;
}


static void writeSocket(void *arg) {
    int socket = *((int*)arg);

    printf("Socket: %d is ready for writing...\n", socket);

    //struct sockaddr_in bcast;
    memset(&bcast, 0, sizeof(bcast));
    bcast.sin_family = AF_INET;
    bcast.sin_addr.s_addr = inet_addr("192.168.0.104"); // pc
    //bcast.sin_addr.s_addr = inet_addr("255.255.255.255");
    bcast.sin_port = htons(8080);

    int numbytes;
    sCommandReply cmdReply;

    int failCnt=0;
    forever {
        if(xQueueReceive(gNetQueue, &cmdReply, 1000)) {
          bcast.sin_port = htons(8080);
            if ((numbytes = sendto(socket, (char*)&cmdReply, sizeof(cmdReply), 0, (struct sockaddr *)&bcast, sizeof(bcast))) == -1) {
              if(xEventGroupGetBits(wifi_event_group) == CONNECTED_BIT) {
                if(failCnt++ % 10 == 0) {
                  printf("Failed to write UDP socket, but retrying...\n");
                }
              } else {
                printf("Failed to write UDP socket because not connected!\n");
                break;
              }
            } else {
              printf("Data sent to %s:%d\n",inet_ntoa(bcast.sin_addr.s_addr), ntohs(bcast.sin_port));
            }
        } else {
            printf("No data in network queue!\n");
        }
    }
    printf("Delete task: writeSocket...\n");
    vTaskDelete(NULL);
}


static void readSocket(void *arg) {
    int socket = *((int*)arg);
    struct sockaddr_storage client_addr;
    socklen_t n = (socklen_t) sizeof( client_addr );

    printf("Socket: %d is ready for reading...\n", socket);

    #define MAXBUFLEN 1024
    char *buf = new char[MAXBUFLEN]; 
    int numbytes;

    forever {
      memset(buf, 0, MAXBUFLEN);
        if ((numbytes = recvfrom(socket, buf, MAXBUFLEN, 0, (struct sockaddr *)&client_addr, &n)) != -1) {
          sCommand scmd;
            if(numbytes == 0) {
              printf("Connection closed gracefully.\n");
            } else {
              struct sockaddr_in *sin = (struct sockaddr_in *)&client_addr;
              printf("Received %d bytes from: %s:%d\n", numbytes, inet_ntoa(sin->sin_addr.s_addr), ntohs(sin->sin_port));
              bcast.sin_addr.s_addr = sin->sin_addr.s_addr;
              if(numbytes == sizeof(sCommand))
              {
                memcpy((void*)&scmd, (void*)buf, 24);
                if(scmd.pkt[0] == 'p' && scmd.pkt[1] == 'k' && scmd.pkt[2] == 't') {
                  printf("Got command: mainRPM: %d, wireDiameter: %f, coilLength: %d, direction: %d\n", scmd.mainRPM, scmd.wireDiameter, scmd.coilLength, scmd.direction);

                  xSemaphoreTake(gMutex, portMAX_DELAY);

                  // send command to queue
                  mCommands.push_back(scmd);
                  printf("mCommand.size: %d\n", mCommands.size());
                  xQueueSend(gNetQueue, "OK\r\n", 5);

                  xSemaphoreGive(gMutex);
                } else {
                  printf("No PKT signature present!\n");
                }
              } else {
                printf("Unknown data from %s:%d: %s\n", inet_ntoa(sin->sin_addr.s_addr), sin->sin_port, buf);
              }
          }
        } else {
          if(xEventGroupGetBits(wifi_event_group) == CONNECTED_BIT) {
          } else {
            std::cout << "Failed to read UDP socket because not connected!\n" << std::endl;
            break;
          }
        }
    }
    delete []buf;
    printf("Deleting task: readSocket...\n");
    vTaskDelete(NULL);
}


static void wifiTask(void *arg)
{
  int sock;
  forever {
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);

    while((sock = createAndBindSocket()) == -1) {
      printf("Failed to create socket, retrying...\n");
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    cFreeRTOS::startTask(readSocket, "read socket task", (void*)&sock);
    cFreeRTOS::startTask(writeSocket, "write socket task", (void*)&sock);

    while(xEventGroupGetBits(wifi_event_group) == CONNECTED_BIT) {
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    printf("Closing socket %d\n", sock);
    close(sock);
  }
}


static esp_err_t esp32_wifi_eventHandler(void *ctx, system_event_t *event) {
  //int sock;
    switch(event->event_id) {
        case SYSTEM_EVENT_AP_START:
            printf("we are an access point! good!\n");
            //sock = createSocket();
            //xTaskCreate(readSocket, "readSocket", 1024 * 2, (void* ) &sock, 10, NULL);
            //xTaskCreate(writeSocket, "writeSocket", 1024 * 2, (void* ) &sock, 10, NULL);
        break;
        case SYSTEM_EVENT_AP_STACONNECTED:
            printf("Client connected to us\n");

        break;
        case SYSTEM_EVENT_AP_STADISCONNECTED:
            printf("Client disconnected from us!\n");
        break;

    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;

        default:
            //printf("default case event handler: %d\n", event->event_id);
            break;
    }
    return ESP_OK;
}


void wifi_station_init() {
    tcpip_adapter_init();

    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(esp32_wifi_eventHandler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );

    wifi_config_t wifi_config = { };

    std::string ssid = "Telekom-eea510";
    std::string pass = "YMYTZKHJYXET";

    memcpy(wifi_config.sta.ssid, ssid.c_str(), ssid.size());
    memcpy(wifi_config.sta.password, pass.c_str(), pass.size());

    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// MAIN TASK 
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void mainTask(void *pvParameter)
{
  forever
  {
    sCommand cmd;
    if(mCommands.size())
    {
      cmd = mCommands.front(); mCommands.pop_front();

      gMainRPM = cmd.mainRPM;
      printf("main RPM: %d\n", gMainRPM);

      uint32_t stepDelay = 40000000 / ((gMainRPM / 60.0f) * 6400);
      printf("step delay1: %d\n", stepDelay);

      // 2 is the distance in mm / revolution eg the thread steepness
      uint32_t stepDelay2 = stepDelay * (2/cmd.wireDiameter);
      printf("step delay2: %d\n", stepDelay2);

      gCoilLengthSteps = (6400/2) * cmd.coilLength;
      printf("coil length steps: %d\n", gCoilLengthSteps);
      gCnt1 = gCoilLengthSteps;

      printf("direction: %d\n", cmd.direction);
      gpio_set_level(DIR2_PIN, cmd.direction);
      gpio_set_level(STATUS_PIN, cmd.direction);

      // calc step delay based on RPM and timer freq
      timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, stepDelay);
      timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, stepDelay2);

      //timer_start(TIMER_GROUP_0, TIMER_0);
      //timer_start(TIMER_GROUP_0, TIMER_1);
    }

    printf("gCnt1: %d\n", gCnt1);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}


void enableGPIO(gpio_num_t pin) {
  gpio_pad_select_gpio(pin);
  gpio_set_direction(pin, static_cast<gpio_mode_t>(GPIO_MODE_INPUT_OUTPUT));
  gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);
  gpio_set_level(pin, 0);
}


static void timer0Handler(void *arg) {

    TIMERG0.int_clr_timers.t0 = 1;
    TIMERG0.hw_timer[0].update=1;
    TIMERG0.hw_timer[0].config.alarm_en = 1;

    if(gMainRPM > 0)
    {
      gpio_set_level(STEP1_PIN, 1);
      ets_delay_us(1);
      gpio_set_level(STEP1_PIN, 0);
    }
}


static void togglePin(gpio_num_t pin)
{
  int level = gpio_get_level(pin);
  level = !level;
  gpio_set_level(pin, level);
}


static void timer1Handler(void *arg) {

    TIMERG0.int_clr_timers.t1 = 1;
    TIMERG0.hw_timer[1].update=1;
    TIMERG0.hw_timer[1].config.alarm_en = 1;

    if(gMainRPM > 0) {

      if(gCnt1 <= 0) {
        gCnt1 = gCoilLengthSteps;
        togglePin(DIR2_PIN);
        togglePin(STATUS_PIN);
      }
      
      gpio_set_level(STEP2_PIN, 1);
      ets_delay_us(1);
      gpio_set_level(STEP2_PIN, 0);
      gCnt1--;
    }
}


esp_err_t configTimer(timer_group_t tgrp, timer_idx_t tmr, void (*fn)(void*), void * arg)
{
  esp_err_t res;

  timer_config_t timerCfg = { true, // alarm enable
    false, // counter enable
    TIMER_INTR_LEVEL,
    TIMER_COUNT_UP,
    true, // auto reload
    2 }; // divider

  if((res = timer_init(tgrp, tmr, &timerCfg)) == ESP_OK) {
    printf("TIMER initialized!\n");
  } else {
    printf("TIMER not initialized!\n");
    return res;
  }

  timer_set_alarm_value(tgrp, tmr, DELAY_BETWEEN_STEPS);
  timer_set_counter_value(tgrp, tmr, 0);

  timer_enable_intr(tgrp, tmr);

  timer_isr_handle_t *handle = 0;
  if((res = timer_isr_register(tgrp, tmr, fn, arg, 0, handle)) == ESP_OK) {
    printf("TIMER isr registered!!! :)\n");
  } else {
    printf("TIMER isr not registered!\n");
    return res;
  }

  if((res = timer_start(tgrp, tmr)) == ESP_OK) {
    printf("TIMER started!\n");
  } else {
    printf("TIMER not started!\n");
    return res;
  }
  return ESP_OK;
}

extern "C" void app_main()
{
  esp_err_t err = nvs_flash_init();

  gMutex = xSemaphoreCreateMutex();

  gCmdQueue = xQueueCreate(5, sizeof(sCommand));
  gNetQueue = xQueueCreate(5, sizeof(sCommandReply));

  enableGPIO(LED_PIN);
  enableGPIO(STATUS_PIN);
  enableGPIO(STEP1_PIN);
  enableGPIO(DIR1_PIN);
  enableGPIO(STEP2_PIN);
  enableGPIO(DIR2_PIN);

  if(configTimer(TIMER_GROUP_0, TIMER_0, timer0Handler, nullptr) == ESP_OK) {
    printf("timer0 initialized!\n");
  }

  if(configTimer(TIMER_GROUP_0, TIMER_1, timer1Handler, nullptr) == ESP_OK) {
    printf("timer1 initialized!\n");
  }

  gpio_set_level(DIR1_PIN, 1);
  gpio_set_level(DIR2_PIN, 1);

  pwmInit();

  wifi_station_init();

  std::cout << "Starting WiFi task..." << std::endl;
  cFreeRTOS::startTask(wifiTask, "WiFi task");

  pwmSet(16);

  cFreeRTOS::startTask(mainTask, "Main task");
}
