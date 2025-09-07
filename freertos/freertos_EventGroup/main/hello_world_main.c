#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define TIMER_DIVIDER         80    // 80 分频 (APB_CLK=80MHz/80=1MHz, 每tick=1us)
#define TIMER_INTERVAL_SEC    1     // 定时 1 秒
#define TIMER_GROUP           TIMER_GROUP_0
#define TIMER_INDEX           TIMER_0
#define TIMER_ALARM_TIME      (TIMER_INTERVAL_SEC * 1000000) // 1秒对应的us

#define  QUEUE_LEN    2   /* 队列的长度，最大可包含多少个消息 */
#define  QUEUE_SIZE   20   /* 队列中每个消息大小（字节） */

#define GPIO_INPUT_IO    12    // 使用 GPIO12 作为外部中断引脚 (请按实际硬件修改)
#define ESP_INTR_FLAG_DEFAULT 0
#define DEBOUNCE_MS 50

#define TIMER_EVENT_BIT (1 << 0) // 位0 代表定时器事件
#define BUTTON_EVENT_BIT (1 << 1) // 位1 代表按键事件

QueueHandle_t timer_queue_handle = NULL;
QueueHandle_t button_queue_handle = NULL;
static EventGroupHandle_t Event_Handle =NULL;

static void recive_task(void* pvParameters);

// 定时器中断服务程序
static bool IRAM_ATTR timer_isr_callback(void *args)
{
    // ISR中不能调用printf，使用简单操作，比如设置标志位或xQueueSend
    static uint32_t val = 0;
    char str[20];
    sprintf(str,"val1 %d",(int)val++);
    xQueueSendFromISR(timer_queue_handle,&str,NULL);
    xEventGroupSetBitsFromISR(Event_Handle,TIMER_EVENT_BIT,NULL);
    return pdFALSE; // 返回true表示需要触发高优先级任务切换
}
// ISR 回调函数 (运行在中断上下文)
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    int gpio_num = (int)arg;
    xQueueSendFromISR(button_queue_handle,&gpio_num,NULL);
    xEventGroupSetBitsFromISR(Event_Handle,BUTTON_EVENT_BIT,NULL);

    // 注意：这里不能调用 printf，会卡死
    // 建议使用队列/任务通知传递事件
}

void app_main(void)
{
    timer_queue_handle = xQueueCreate((UBaseType_t)QUEUE_LEN,(UBaseType_t)QUEUE_SIZE);
    button_queue_handle = xQueueCreate((UBaseType_t)QUEUE_LEN,(UBaseType_t)QUEUE_SIZE);

    if (NULL != timer_queue_handle && NULL != button_queue_handle)
    {
        ESP_LOGI("queue","creat ok");
    }
    Event_Handle = xEventGroupCreate();
    if (NULL != Event_Handle)
    {
        ESP_LOGI("Event_Handle","creat ok");
    }

    // 1. 配置 GPIO
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE; // 下降沿触发 (可选：GPIO_INTR_POSEDGE, GPIO_INTR_ANYEDGE)
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL<<GPIO_INPUT_IO);
    io_conf.pull_up_en = 1;   // 开启上拉 (按键常用)
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);

    // 2. 安装 GPIO ISR 服务
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    // 3. 注册中断处理函数
    gpio_isr_handler_add(GPIO_INPUT_IO, gpio_isr_handler, (void*) GPIO_INPUT_IO);

    ESP_LOGI("GPIO", "GPIO interrupt example init done.");

    // 配置定时器
    timer_config_t config = {
        .divider = TIMER_DIVIDER,        // 分频
        .counter_dir = TIMER_COUNT_UP,   // 向上计数
        .counter_en = TIMER_PAUSE,       // 初始暂停
        .alarm_en = TIMER_ALARM_EN,      // 开启alarm
        .auto_reload = true,             // 自动重载
    };
    timer_init(TIMER_GROUP, TIMER_INDEX, &config);

    // 设置初始计数值
    timer_set_counter_value(TIMER_GROUP, TIMER_INDEX, 0);

    // 设置alarm
    timer_set_alarm_value(TIMER_GROUP, TIMER_INDEX, TIMER_ALARM_TIME);

    // 注册中断回调
    timer_isr_callback_add(TIMER_GROUP, TIMER_INDEX, timer_isr_callback, NULL, 0);

    xTaskCreate(recive_task, "recive_task", 4096, NULL, 5, NULL);
    // 启动定时器
    timer_start(TIMER_GROUP, TIMER_INDEX);
}

static void recive_task(void* pvParameters)
{
    BaseType_t ret = true;
    BaseType_t event;
    uint32_t rec_val;
    char str[20];
    while (1)
    {
        event = xEventGroupWaitBits(Event_Handle,TIMER_EVENT_BIT|BUTTON_EVENT_BIT,true,false,portMAX_DELAY);
        if (event & TIMER_EVENT_BIT)
        {
            ret = xQueueReceive(timer_queue_handle,&str,portMAX_DELAY);
            if (ret == true)
                ESP_LOGI("timer","recive data %s",str);
            else
                ESP_LOGI("timer","recive ng");
        }
        if (event & BUTTON_EVENT_BIT)
        {
            int gpio_num;
            int last_state = 1;
            TickType_t last_time = 0;
            ret = xQueueReceive(button_queue_handle,&gpio_num,portMAX_DELAY);
            if (ret == true)
            {
                int level = gpio_get_level(gpio_num);
                TickType_t now = xTaskGetTickCount();

                if(level != last_state && now - last_time > pdMS_TO_TICKS(DEBOUNCE_MS)) {
                    ESP_LOGI("button", "Key pressed/released GPIO[%d], level=%d", gpio_num, level);
                    last_state = level;
                    last_time = now;
                    }
            }
            else
                ESP_LOGI("button","recive ng");
        }
    }
}
