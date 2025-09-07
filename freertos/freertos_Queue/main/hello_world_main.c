#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/timer.h"
#include "esp_log.h"

#define TIMER_DIVIDER         80    // 80 分频 (APB_CLK=80MHz/80=1MHz, 每tick=1us)
#define TIMER_INTERVAL_SEC    1     // 定时 1 秒
#define TIMER_GROUP           TIMER_GROUP_0
#define TIMER_INDEX           TIMER_0
#define TIMER_ALARM_TIME      (TIMER_INTERVAL_SEC * 1000000) // 1秒对应的us

#define  QUEUE_LEN    2   /* 队列的长度，最大可包含多少个消息 */
#define  QUEUE_SIZE   40   /* 队列中每个消息大小（字节） */

QueueHandle_t test_queue_handle = NULL;


static void recive_task(void* pvParameters);

// 定时器中断服务程序
static bool IRAM_ATTR timer_isr_callback(void *args)
{
    // ISR中不能调用printf，使用简单操作，比如设置标志位或xQueueSend
    static uint32_t val = 0;
    char str[20];
    sprintf(str,"val %d",(int)val);
    xQueueSendFromISR(test_queue_handle,&str,NULL);
    return pdFALSE; // 返回true表示需要触发高优先级任务切换
}

void app_main(void)
{
    test_queue_handle = xQueueCreate((UBaseType_t)QUEUE_LEN,(UBaseType_t)QUEUE_SIZE);
    if (NULL != test_queue_handle)
    {
        ESP_LOGI("queue","creat ok");
    }

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

    xTaskCreate(recive_task, "recive_task", 2048, NULL, 5, NULL);
    // 启动定时器
    timer_start(TIMER_GROUP, TIMER_INDEX);
}

static void recive_task(void* pvParameters)
{
    BaseType_t ret = true;
    uint32_t rec_val;
    char str[20];
    while (1)
    {
        ret = xQueueReceive(test_queue_handle,&str,portMAX_DELAY);
        if (ret == true)
            ESP_LOGI("queue","recive data %s",str);
        else
            ESP_LOGI("queue","recive ng");
    }
}
