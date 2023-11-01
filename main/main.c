/* LVGL Example project
 *
 * Basic project to test LVGL on ESP32 based projects.
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_freertos_hooks.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "freertos/queue.h"

/* Littlevgl specific */
#include "lvgl/lvgl.h"
#include "lvgl_helpers.h"


#include "c1.c"
#include "c2.c"
#include "earth.c"
#include "man1.c"
#include "man2.c"
#include "man3.c"
#include "man4.c"
#include "man5.c"
#include "man6.c"
#include "man11.c"

#include "watch_bg.c"
#include "hour.c"
#include "minute.c"
#include "second.c"
#include "pups_01_r.c"

#include "pcf8563.h"
#include "CST816S.h"


static lv_style_t style;
lv_obj_t * img;
lv_obj_t * img0;

lv_obj_t * lvMinute;
lv_obj_t * lvHour;
lv_obj_t * lvSecond ;

uint8_t cur_time_h;
uint8_t cur_time_m;
uint8_t cur_time_s;


/*********************
 *      DEFINES
 *********************/
#define TAG "demo"
#define LV_TICK_PERIOD_MS 1

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void lv_tick_task(void *arg);
static void guiTask(void *pvParameter);
static void create_demo_application(void);
static void getClock(void *pvParameter);
static void touchpad(void *pvParameter);



static void lv_periodic_task(void *arg) {
    (void) arg;



}


static void update_time(void *arg)
{
//    get_time();

	cur_time_s++;
printf("time setting\n");
   lv_img_set_angle(  lvHour, cur_time_h*30*10);
    lv_img_set_angle(  lvMinute, cur_time_m*6*10);
    lv_img_set_angle(  lvSecond, cur_time_s*6*10);
}


void demo_c1_()
{
		//��� ���������
		img0 = lv_img_create(lv_scr_act(), NULL);
//	    lv_img_set_src(img0, &c1);
	    lv_img_set_src(img0, &pups_01_r);
	    lv_obj_align(img0, NULL, LV_ALIGN_CENTER, 0, 0);
}



void watch1()
{
	lv_style_init(&style);
    img = lv_img_create(lv_scr_act(),NULL);
    lv_img_set_src(img, &pups_01_r);

    lv_obj_align(img, NULL, LV_ALIGN_CENTER, 0, 0);
   /* lvHour = lv_img_create(img,NULL);
    lv_img_set_src( lvHour, &hour);
    lv_obj_align(  lvHour, img,LV_ALIGN_CENTER, 0, 0);
    lv_img_set_angle(lvHour, 900);

    lvMinute = lv_img_create(img,NULL);
    lv_img_set_src( lvMinute, &minute);
    lv_img_set_zoom(lvMinute, 300);
    lv_obj_align(  lvMinute, img,LV_ALIGN_CENTER, 0, 0);


    lvSecond = lv_img_create(img,NULL);
    lv_img_set_src( lvSecond, &second);
    lv_img_set_zoom(lvSecond, 350);
    lv_obj_align(  lvSecond, img,LV_ALIGN_CENTER, 0, 0);*/


    lv_task_create(update_time, 1000, LV_TASK_PRIO_LOW, NULL);
}





/**********************
 *   APPLICATION MAIN
 **********************/
void app_main() {

    /* If you want to use a task to create the graphic, you NEED to create a Pinned task
     * Otherwise there can be problem such as memory corruption and so on.
     * NOTE: When not using Wi-Fi nor Bluetooth you can pin the guiTask to core 0 */
    xTaskCreatePinnedToCore(guiTask, "gui", 4096*2, NULL, 0, NULL, 1);
    xTaskCreate(getClock, "getClock", 1024*4, NULL, 2, NULL);
    xTaskCreate(touchpad, "touchpad", 1024*4, NULL, 2, NULL);
}

#define CONFIG_SDA_GPIO (26)
#define CONFIG_SCL_GPIO (25)
xQueueHandle interruptQueue;
bool flagTouchIpt = 0;
i2c_dev_t gdev;
static void IRAM_ATTR touchInterruptHandler(void *args)
{
    int pinNumber = (int)args;
    flagTouchIpt = 1;
   // xQueueSendFromISR(interputQueue, &pinNumber, NULL);
   // printf("touchInterrupt\n");
}

void touchpad(void *pvParameters)
{

  gpio_set_direction(32, GPIO_MODE_INPUT);
  gpio_pulldown_en(32);
  gpio_pullup_dis(32);
  gpio_set_intr_type(32, GPIO_INTR_POSEDGE);




  gpio_install_isr_service(0);
  gpio_isr_handler_add(32,touchInterruptHandler,(void*)32);

  i2c_dev_t dev;
  	if (cst816S_init_desc(&dev, I2C_NUM_0, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO) != ESP_OK) {
  		ESP_LOGE(pcTaskGetName(0), "Could not init device descriptor. touch");
  		//while (1) { vTaskDelay(1); }
  	}

  	CST816SStart(&dev,  33, 32, 0x15);


  while (1) {
    if(flagTouchIpt)
    {

      uint8_t data[8];
      uint8_t i = 0;
      CST816SReadTouch(&dev);
     uint16_t x = CST816SGetX();
      uint16_t y = CST816SGetY();
      uint8_t type = CST816SGetTouchType();
      //while(i<8)
     // {
	//data[i] =  CST816SGetToucData(i);
	//printf("Data = %u\n", (uint16_t)data[i++]);
     // }
	printf("TOuch ipt, x = %u, y = %u, type = %u\n", x, y , type);
      flagTouchIpt = 0;
    }
      /* Delay 1 tick (assumes FreeRTOS tick is 10ms */
      vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void getClock(void *pvParameters)
{
	// Initialize RTC
	//i2c_dev_t dev;
	//if (pcf8563_init_desc(&dev, I2C_NUM_0, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO) != ESP_OK) {
	//	ESP_LOGE(pcTaskGetName(0), "Could not init device descriptor.");
		while (1) { vTaskDelay(1); }
	//}

	//gdev = dev;

	// Initialise the xLastWakeTime variable with the current time.
	TickType_t xLastWakeTime = xTaskGetTickCount();

	// Get RTC date and time
	/*while (1) {
		struct tm rtcinfo;

		if (pcf8563_get_time(&dev, &rtcinfo) != ESP_OK) {
			ESP_LOGE(pcTaskGetName(0), "Could not get time.");
			while (1) { vTaskDelay(1); }
		}

		ESP_LOGI(pcTaskGetName(0), "%04d-%02d-%02d %02d:%02d:%02d",
			rtcinfo.tm_year, rtcinfo.tm_mon + 1,
			rtcinfo.tm_mday, rtcinfo.tm_hour, rtcinfo.tm_min, rtcinfo.tm_sec);
	vTaskDelayUntil(&xLastWakeTime, 1000);
	}*/
}

/* Creates a semaphore to handle concurrent call to lvgl stuff
 * If you wish to call *any* lvgl function from other threads/tasks
 * you should lock on the very same semaphore! */
SemaphoreHandle_t xGuiSemaphore;

static void guiTask(void *pvParameter) {

    (void) pvParameter;
    xGuiSemaphore = xSemaphoreCreateMutex();

    lv_init();

    /* Initialize SPI or I2C bus used by the drivers */
    lvgl_driver_init();

    /* Use double buffered when not working with monochrome displays */
    static lv_color_t buf1[DISP_BUF_SIZE];
#ifndef CONFIG_LVGL_TFT_DISPLAY_MONOCHROME
    static lv_color_t buf2[DISP_BUF_SIZE];
#endif

    static lv_disp_buf_t disp_buf;
    uint32_t size_in_px = DISP_BUF_SIZE;

    /* Initialize the working buffer depending on the selected display */

    lv_disp_buf_init(&disp_buf, buf1, buf2, size_in_px);
 //   lv_disp_buf_init(&disp_buf, buf1, NULL, size_in_px);


    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.flush_cb = disp_driver_flush;

    /* When using a monochrome display we need to register the callbacks:
     * - rounder_cb
     * - set_px_cb */

    disp_drv.buffer = &disp_buf;
    lv_disp_drv_register(&disp_drv);

    /* Register an input device when enabled on the menuconfig */
//#if CONFIG_LVGL_TOUCH_CONTROLLER != TOUCH_CONTROLLER_NONE
    lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.read_cb = touch_driver_read;
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    lv_indev_drv_register(&indev_drv);
//#endif

    /* Create and start a periodic timer interrupt to call lv_tick_inc */
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &lv_tick_task,
        .name = "periodic_gui"
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));

    /* Create the demo application */
    create_demo_application();

    while (1) {
        /* Delay 1 tick (assumes FreeRTOS tick is 10ms */
        vTaskDelay(pdMS_TO_TICKS(10));

        /* Try to take the semaphore, call lvgl related function on success */
        if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
            lv_task_handler();
            xSemaphoreGive(xGuiSemaphore);
       }
    }

    /* A task should NEVER return */
    vTaskDelete(NULL);
}

static void create_demo_application(void)
{

	watch1(); 	//demo ���������/����������
//	demo_c1_();	//����

}



static void lv_tick_task(void *arg) {
    (void) arg;
    lv_tick_inc(LV_TICK_PERIOD_MS);
}
