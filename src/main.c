
//https://academy.nordicsemi.com/courses/nrf-connect-sdk-fundamentals/lessons/lesson-1-nrf-connect-sdk-introduction/

/*
Md. Khairul Alam
Date: 25 August, 2024
*/

#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <hal/nrf_gpio.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

#define ADC_NODE DT_NODELABEL(adc)
static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);

#define ADC_RESOLUTION 10
#define ADC_CHANNEL    0
#define ADC_PORT       SAADC_CH_PSELN_PSELN_AnalogInput0  //AIN0
#define ADC_REFERENCE  ADC_REF_INTERNAL                   //0.6V
#define ADC_GAIN       ADC_GAIN_1_5                       //ADC_REFERENCE*5

struct adc_channel_cfg chl0_cfg = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQ_TIME_DEFAULT,
	.channel_id = ADC_CHANNEL,
#ifdef CONFIG_ADC_NRFX_SAADC
	.input_positive = ADC_PORT
#endif
};

int16_t sample_buffer[1];

struct adc_sequence sequence = {
	/* individual channels will be added below */
	.channels = BIT(ADC_CHANNEL),
	.buffer = sample_buffer,
	/* buffer size in bytes, not number of samples */
	.buffer_size = sizeof(sample_buffer),
	.resolution = ADC_RESOLUTION
};


#define LED_GREEN NRF_GPIO_PIN_MAP(1, 10)
#define SPEED_INCREASE_PIN NRF_GPIO_PIN_MAP(1, 11)
#define SPEED_DECREASE_PIN NRF_GPIO_PIN_MAP(1, 12)

#define LIMIT_BOTTOM NRF_GPIO_PIN_MAP(1, 3)
#define LIMIT_TOP NRF_GPIO_PIN_MAP(1, 4)


/*Function Prototype*/
void initialize_adc(void);
int read_adc(void);



int main(void)
{
	nrf_gpio_cfg_output(LED_GREEN);
	nrf_gpio_cfg_output(SPEED_INCREASE_PIN);
	nrf_gpio_cfg_output(SPEED_DECREASE_PIN);

	nrf_gpio_cfg_input(LIMIT_BOTTOM, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(LIMIT_TOP, NRF_GPIO_PIN_PULLUP);
 
    initialize_adc();

	int change_margin = 20; //determinds the sensitivity of the sensor, find it after trial and error
	int count = 1800;       //reduce false trigger before a resonable amount of bending
	
	while (1) {
		
		int32_t flex_value = read_adc();

		while(flex_value>1800){

			if((flex_value-count)>change_margin){
				int delay_count = flex_value-count;  
				nrf_gpio_pin_set(SPEED_INCREASE_PIN);
				k_msleep(delay_count*1.7); //depends on the speed of the linear actuator
				//printk("Delay count positive: %d.\n", delay_count);
				nrf_gpio_pin_clear(SPEED_INCREASE_PIN);
				count += delay_count; 
				printk("Count: %d.\n", count);
			}

			else if((flex_value-count)<-change_margin){
				int delay_count = abs(flex_value-count);   //only value is count
				nrf_gpio_pin_set(SPEED_DECREASE_PIN);
				k_msleep(delay_count*1.7);
				//printk("Delay count negative: %d.\n", delay_count);
				nrf_gpio_pin_clear(SPEED_DECREASE_PIN);
				count -= delay_count; 
			}

			flex_value = read_adc();

		}

	}
	return 0;
}



void initialize_adc(void){
    int err;

	if(!device_is_ready(adc_dev)){
		printk("adc_dev not ready\n");
		return;
	}

	err = adc_channel_setup(adc_dev, &chl0_cfg);
	if(err != 0){
		printk("adc_channel_setup failed with error %d.\n, err");
		return;
	}
}


int read_adc(void){
    int err = adc_read(adc_dev, &sequence);
	if(err != 0){
		printk("ADC reading failed with error %d.\n", err);
		return;
	}

	int32_t mv_value = sample_buffer[0];
	//printk("ADC-raw: %d mV\n", mv_value);
	int32_t adc_vref = adc_ref_internal(adc_dev);
	adc_raw_to_millivolts(adc_vref, ADC_GAIN, ADC_RESOLUTION, &mv_value);
	printk("ADC Sensor Reading: %d mV\n", mv_value); 
	//k_msleep(SLEEP_TIME_MS);
	return mv_value;   
}