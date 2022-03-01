// #include <stdio.h>                   // printf()
// #include <math.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"           // for task and timing
// #include "driver/ledc.h"

// #define LEDC_DEFAULT_CLK         0

// unsigned long IRAM_ATTR millis()
// {
//     return (unsigned long) (esp_timer_get_time() / 1000ULL);
// }
// class LedChannel {
// public:
//     LedChannel(uint8_t chan, uint8_t pin)
//     : chan(chan), pin(pin), freq(5000), resolution(0), group(0), timer(0) {
//     }

//     bool Setup(double freq, uint8_t resolution) {
//         this->freq = freq;
//         this->resolution = resolution;
//         this->group=(this->chan/8);
//         this->timer=((this->chan/2)%4);

//         ledc_timer_config_t ledc_timer = {
//             .speed_mode       = (ledc_mode_t) this->group,
//             .duty_resolution  = (ledc_timer_bit_t) this->resolution,
//             .timer_num        = (ledc_timer_t) this->timer,
//             .freq_hz          = (uint32_t) this->freq,
//             .clk_cfg          = (ledc_clk_cfg_t) LEDC_DEFAULT_CLK
//         };

//         ledc_timer_config(&ledc_timer);

//         uint8_t channel=(this->chan%8);

//         ledc_channel_config_t ledc_channel = {
//             .gpio_num       = (int) this->pin,
//             .speed_mode     = (ledc_mode_t) this->group,
//             .channel        = (ledc_channel_t) channel,
//             .intr_type      = (ledc_intr_type_t) LEDC_INTR_DISABLE,
//             .timer_sel      = (ledc_timer_t) this->timer,
//             .duty           = 0,
//             .hpoint         = 0,
//             .flags = {
//                 .output_invert = 0
//             }
//         };
//         ledc_channel_config(&ledc_channel);

//         return true;
//     }

//     bool SetDuty(uint32_t duty) {
//         uint32_t max_duty = (1 << this->resolution) - 1;
//         printf("%p\n", this);

//         if(duty == max_duty){
//             duty = max_duty + 1;
//         }

//         uint8_t channel=(chan%8);

//         ledc_set_duty((ledc_mode_t) this->group, (ledc_channel_t) channel, duty);
//         ledc_update_duty((ledc_mode_t) this->group, (ledc_channel_t) channel);
//         return true;
//     }

// private:
//     uint8_t chan;
//     uint8_t pin;
//     double freq;
//     uint8_t resolution;
//     uint8_t group;
//     uint8_t timer;

// };

// // Using static polymorphism through the use of the C++ CRTP pattern to:
// //  * Reduces memory footprint no vtable since nothing is virtual
// // Read more on the topic:
// //      https://en.wikipedia.org/wiki/Curiously_recurring_template_pattern
// //      https://fjrg76.wordpress.com/2018/05/23/objectifying-task-creation-in-freertos-ii/
// //      https://www.fluentcpp.com/2017/05/16/what-the-crtp-brings-to-code/
// template<typename T>
// class ThreadX {
// public:
//     ThreadX()
//     {
//     }

//     void Start(unsigned _stackDepth, UBaseType_t _priority, const char* _name = "" ) {
//         xTaskCreate( task, _name, _stackDepth, this, _priority, &this->taskHandle);
//     }

//     TaskHandle_t GetHandle()
//     {
//         return this->taskHandle;
//     }

//     void Main()
//     {
//         static_cast<T&>(*this).Main();
//     }

// private:
//     static void task( void* _params )
//     {
//         ThreadX* p = static_cast<ThreadX*>( _params );
//         p->Main();
//     }

//     TaskHandle_t taskHandle;
// };

// class LedFaderX : public ThreadX<LedFaderX>
// {
// public:
//     LedFaderX(unsigned _stackDepth, UBaseType_t _priority, const char* _name, LedChannel *_ledChannel, uint32_t _ticks = 1 )
//     : ledChannel(_ledChannel)
//     , ticks( _ticks )
//     {
//         this->Start(_stackDepth, _priority, _name);
//     }

//     void Main()
//     {
//         float val = 0;
//         while( 1 )
//         {
//             val = (exp(sin(millis()/2000.0*M_PI)) - 0.36787944)*108.0;
//             printf("%f\n", val);
//             this->ledChannel->SetDuty((uint32_t) val);
//             vTaskDelay(this->ticks);
//         }
//     }
// private:
//     LedChannel* ledChannel;
//     uint32_t ticks;
// };

// extern "C" void app_main()
// {
//     printf("Hello Breathe LEDC!\n");

//     LedChannel *led1 = new LedChannel(0, 5);
//     led1->Setup(5000, 8);
//     printf("%p\n", led1);

//     printf("Start LEDC Breathing\n");
//     LedFaderX lf{ 8196, 2, "ledx", led1, 1 };
//     printf("LEDC Task running\n");
// }

#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
 
#define ESP_INTR_FLAG_DEFAULT 0
 
#define BLINK_LED (gpio_num_t) 5
#define GPIO_INPUT_IO_0 (gpio_num_t) 12
int buttonCount = 0;
int i = 0;
 
SemaphoreHandle_t xSemaphore = NULL;
 
TaskHandle_t printVariableTask = NULL;
 
void printVariable(void *pvParameter) {
 
    int a = (int) pvParameter;
    while (1) {
 
        printf("A is a: %d \n", a++);
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}
// interrupt service routine, called when the button is pressed
void IRAM_ATTR button_isr_handler(void* arg) {
 
    // notify the button task
    xSemaphoreGiveFromISR(xSemaphore, NULL);
 
}
// task that will react to button clicks
void button_task(void* arg) {
 
    // infinite loop
    for(;;) {
 
        // wait for the notification from the ISR
        if(xSemaphoreTake(xSemaphore,portMAX_DELAY) == pdTRUE) {
            //int buttonState = gpio_get_level(GPIO_INPUT_IO_0);
            printf("wokeup %d\n", gpio_get_level(GPIO_INPUT_IO_0));
 
            while(gpio_get_level(GPIO_INPUT_IO_0) == 0) {
                buttonCount++;
                printf("GPIO_INPUT_IO_0 %d\n", 1);
                printf("Button pressed! %d \n", i++);
                gpio_set_level(BLINK_LED, 1);
                vTaskDelay(10 / portTICK_RATE_MS);
                }
            gpio_set_level(BLINK_LED, 0);
        }
    }
}

#define NUM_OUPUT_PINS  34
#define PIN_DAC1        25
#define PIN_DAC2        26

#define LOW               0x0
#define HIGH              0x1

//GPIO FUNCTIONS
#define INPUT             0x01
#define OUTPUT            0x02
#define PULLUP            0x04
#define INPUT_PULLUP      0x05
#define PULLDOWN          0x08
#define INPUT_PULLDOWN    0x09
#define OPEN_DRAIN        0x10
#define OUTPUT_OPEN_DRAIN 0x12
#define SPECIAL           0xF0
#define FUNCTION_1        0x00
#define FUNCTION_2        0x20
#define FUNCTION_3        0x40
#define FUNCTION_4        0x60
#define FUNCTION_5        0x80
#define FUNCTION_6        0xA0
#define ANALOG            0xC0

void configurePullup(uint8_t pin, gpio_mode_t mode) {
    gpio_config_t conf = {
		    .pin_bit_mask = (1ULL<<pin),			/*!< GPIO pin: set with bit mask, each bit maps to a GPIO */
		    .mode = GPIO_MODE_DISABLE,              /*!< GPIO mode: set input/output mode                     */
		    .pull_up_en = GPIO_PULLUP_DISABLE,      /*!< GPIO pull-up                                         */
		    .pull_down_en = GPIO_PULLDOWN_DISABLE,  /*!< GPIO pull-down                                       */
		    .intr_type = GPIO_INTR_DISABLE      	/*!< GPIO interrupt type                                  */
	};
	if (mode < 0x20) {//io
		conf.mode = (gpio_mode_t) (mode & (INPUT | OUTPUT));
		if (mode & OPEN_DRAIN) {
			//conf.mode |= ((gpio_mode_t) GPIO_MODE_DEF_OD);
		}
		if (mode & PULLUP) {
			conf.pull_up_en = GPIO_PULLUP_ENABLE;
		}
		if (mode & PULLDOWN) {
			conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
		}
	}
	if(gpio_config(&conf) != ESP_OK)
    {
        return;
    }
}
 
extern "C" void app_main()
{
    // create the binary semaphore
    xSemaphore = xSemaphoreCreateBinary();
 
    // configure button and led pins as GPIO pins
    gpio_pad_select_gpio(GPIO_INPUT_IO_0);
    gpio_pad_select_gpio(BLINK_LED);
 
    // set the correct direction
    gpio_set_direction(GPIO_INPUT_IO_0, GPIO_MODE_INPUT);
    gpio_set_direction(BLINK_LED, GPIO_MODE_OUTPUT);
    //gpio_set_level(BLINK_LED, 1);
    configurePullup(GPIO_INPUT_IO_0, (gpio_mode_t) INPUT_PULLUP);

    // enable interrupt on falling (1->0) edge for button pin
    gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_NEGEDGE);
 
    // start the task that will handle the button
    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);
 
    // install ISR service with default configuration
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
 
    // attach the interrupt service routine
    gpio_isr_handler_add(GPIO_INPUT_IO_0, button_isr_handler, NULL);
 
    int pass = 25;
    xTaskCreate(&printVariable, "printVariable", 2048, (void*) pass, 5, &printVariableTask);
 
}