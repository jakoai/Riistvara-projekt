/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

// DESCRIPTION:
// This example contains minimal code to make ESP32-S2 based device
// recognizable by USB-host devices as a USB Serial Device printing output from
// the application.

#include <stdio.h>
#include <stdlib.h>
#include <sys/reent.h>
#include "esp_log.h"
#include "esp_vfs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "tusb_console.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include <led_strip.h>
#include <math.h>
#include <driver/i2c.h>
#include "TPS65988.h"
#include "BQ2573.h"

#define SDA_PIN 35
#define SCL_PIN 36

#define BUTTON_BACK     GPIO_NUM_0
#define BUTTON_INC      GPIO_NUM_38
#define BUTTON_DEC      GPIO_NUM_37
#define BUTTON_ENTER    GPIO_NUM_39
#define BUTTON_EXT1     GPIO_NUM_40
#define BUTTON_EXT2     GPIO_NUM_41

#define LED_STRIP_LEN 16
#define LED_TYPE LED_STRIP_WS2812
#define LED_GPIO GPIO_NUM_1

enum RGB_LED_POSITION {
    RBG_LED_TOP_RIGHT,
    RBG_LED_RIGHT_UPPER,
    RGB_LED_BTN_EXT2,
    RGB_LED_DISP_BKG2,
    RGB_LED_DISP_BKG1,
    RBG_LED_TOP_LEFT,
    RBG_LED_LEFT_UPPER,
    RGB_LED_BTN_EXT1,
    RBG_LED_LEFT_LOWER,
    RBG_LED_BOTTOM_LEFT,
    RGB_LED_BTN_BACK,
    RGB_LED_BTN_DEC,
    RGB_LED_BTN_INC,
    RGB_LED_BTN_ENTER,
    RBG_LED_BOTTOM_RIGHT,
    RBG_LED_RIGHT_LOWER
};

static const char *TAG = "example";

static const rgb_t colors[] = {
    { .r = 0x0f, .g = 0x0f, .b = 0x0f },
    { .r = 0x00, .g = 0x00, .b = 0x2f },
    { .r = 0x00, .g = 0x2f, .b = 0x00 },
    { .r = 0x2f, .g = 0x00, .b = 0x00 },
    { .r = 0x00, .g = 0x00, .b = 0x00 },
};

#define COLORS_TOTAL (sizeof(colors) / sizeof(rgb_t))


struct HSV
{
	double H;
	double S;
	double V;
};

rgb_t HSVToRGB(struct HSV hsv) {
	double r = 0, g = 0, b = 0;

	if (hsv.S == 0)
	{
		r = hsv.V;
		g = hsv.V;
		b = hsv.V;
	}
	else
	{
		int i;
		double f, p, q, t;

		if (hsv.H == 360)
			hsv.H = 0;
		else
			hsv.H = hsv.H / 60;

		i = (int)trunc(hsv.H);
		f = hsv.H - i;

		p = hsv.V * (1.0 - hsv.S);
		q = hsv.V * (1.0 - (hsv.S * f));
		t = hsv.V * (1.0 - (hsv.S * (1.0 - f)));

		switch (i)
		{
		case 0:
			r = hsv.V;
			g = t;
			b = p;
			break;

		case 1:
			r = q;
			g = hsv.V;
			b = p;
			break;

		case 2:
			r = p;
			g = hsv.V;
			b = t;
			break;

		case 3:
			r = p;
			g = q;
			b = hsv.V;
			break;

		case 4:
			r = t;
			g = p;
			b = hsv.V;
			break;

		default:
			r = hsv.V;
			g = p;
			b = q;
			break;
		}

	}

	rgb_t rgb;
	rgb.r = r * 255;
	rgb.g = g * 255;
	rgb.b = b * 255;

	return rgb;
}

struct HSV rainbow = { 0.0, 1.0, 0.1 };
void test(void *pvParameters)
{
    led_strip_t strip = {
        .type = LED_TYPE,
        .length = LED_STRIP_LEN,
        .gpio = LED_GPIO,
        .buf = NULL,
#ifdef LED_STRIP_BRIGHTNESS
        .brightness = 255,
#endif
    };
    ESP_ERROR_CHECK(led_strip_init(&strip));
    size_t c = 0;

    while (1)
    {
        ESP_ERROR_CHECK(led_strip_fill(&strip, 0, strip.length, HSVToRGB(rainbow)));
        led_strip_set_pixel(&strip, RGB_LED_BTN_BACK, gpio_get_level(BUTTON_BACK) ? colors[2]:colors[1]);
        led_strip_set_pixel(&strip, RGB_LED_BTN_INC, gpio_get_level(BUTTON_INC) ? colors[2]:colors[1]);
        led_strip_set_pixel(&strip, RGB_LED_BTN_DEC, gpio_get_level(BUTTON_DEC) ? colors[2]:colors[1]);
        led_strip_set_pixel(&strip, RGB_LED_BTN_ENTER, gpio_get_level(BUTTON_ENTER) ? colors[2]:colors[1]);
        led_strip_set_pixel(&strip, RGB_LED_BTN_EXT1, gpio_get_level(BUTTON_EXT1) ? colors[2]:colors[1]);
        led_strip_set_pixel(&strip, RGB_LED_BTN_EXT2, gpio_get_level(BUTTON_EXT2) ? colors[2]:colors[1]);
        ESP_ERROR_CHECK(led_strip_flush(&strip));
        if (++rainbow.H>360) rainbow.H = 0;

        vTaskDelay(pdMS_TO_TICKS(10));

        if (++c >= COLORS_TOTAL)
            c = 0;
    }
}


void i2c_init(){
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = SDA_PIN;
    conf.scl_io_num = SCL_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    i2c_param_config(I2C_NUM_0, &conf);

    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    ESP_LOGI(TAG, "I2C installed");
}

void i2c_scan(){
    esp_err_t res;
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
    printf("00:         ");
    for (uint8_t i = 3; i < 0x78; i++)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
        i2c_master_stop(cmd);

        res = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
        if (i % 16 == 0)
            printf("\n%.2x:", i);
        if (res == 0)
            printf(" %.2x", i);
        else
            printf(" --");
        i2c_cmd_link_delete(cmd);
    }
    printf("\n\n");
}


void app_main(void)
{
    /* Setting TinyUSB up */
    ESP_LOGI(TAG, "USB initialization");

    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false, // In the most cases you need to use a `false` value
        .configuration_descriptor = NULL,
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t acm_cfg = { 0 }; // the configuration uses default values
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));

    ESP_LOGI(TAG, "USB initialization DONE");

    gpio_num_t shift_in_out_latch = GPIO_NUM_45;
    gpio_num_t shift_out_clk = GPIO_NUM_46;
    gpio_num_t shift_out_data = GPIO_NUM_3;
    gpio_num_t shift_out_clr = GPIO_NUM_16;
    gpio_set_direction(shift_out_clr, GPIO_MODE_OUTPUT);
    gpio_set_level(shift_out_clr, 0);
    gpio_set_direction(shift_in_out_latch, GPIO_MODE_OUTPUT);
    gpio_set_direction(shift_out_clk, GPIO_MODE_OUTPUT);
    gpio_set_direction(shift_out_data, GPIO_MODE_OUTPUT);
    gpio_set_direction(shift_out_clr, GPIO_MODE_OUTPUT);
    gpio_set_level(shift_out_clk, 0);
    gpio_set_level(shift_out_data, 0);
    gpio_set_level(shift_out_clr, 1);
    uint8_t shift_out_state = 0b10000000; //enable 5V supply
    for (int8_t i = 7; i >= 0; i--){
        gpio_set_level(shift_out_data, shift_out_state&(1<<i));
        gpio_set_level(shift_out_clk, 1);
        gpio_set_level(shift_out_clk, 0);
    }
    gpio_set_level(shift_in_out_latch, 1);
    gpio_set_level(shift_in_out_latch, 0);

    ESP_LOGI(TAG, "Shift out initialization DONE");

    led_strip_install();
    xTaskCreate(test, "test", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
    ESP_LOGI(TAG, "LED strip initialization DONE");


    gpio_set_direction(BUTTON_BACK, GPIO_MODE_INPUT);
    gpio_set_direction(BUTTON_INC, GPIO_MODE_INPUT);
    gpio_set_direction(BUTTON_DEC, GPIO_MODE_INPUT);
    gpio_set_direction(BUTTON_ENTER, GPIO_MODE_INPUT);
    gpio_set_direction(BUTTON_EXT1, GPIO_MODE_INPUT);
    gpio_set_direction(BUTTON_EXT2, GPIO_MODE_INPUT);

    i2c_init();
    i2c_scan();

    /*uint8_t reg_size;
    uint8_t buf[4] = {0};
    ESP_ERROR_CHECK(tps65988_i2c_read(TPS65988_ADDR_VERSION, buf, sizeof(buf), &reg_size));

    ESP_LOGI(TAG, "version, reg size %d", reg_size);
    for (int i = 0; i<sizeof(buf);i++){
        ESP_LOGI(TAG, "%d", buf[i]);
    };*/


    TPS65988_GLOBAL_SYSTEM_CONF_T tps_global_conf = {0};
    tps65988_set_pp_vbus_map(&tps_global_conf, TPS65988_POWER_PATH_EXT1, TPS65988_POWER_PATH_MAP_VBUS1);
    tps65988_set_pp_vbus_map(&tps_global_conf, TPS65988_POWER_PATH_INT1, TPS65988_POWER_PATH_MAP_VBUS1);
    tps65988_set_pp_vbus_map(&tps_global_conf, TPS65988_POWER_PATH_EXT2, TPS65988_POWER_PATH_MAP_VBUS2);
    tps65988_set_pp_vbus_map(&tps_global_conf, TPS65988_POWER_PATH_INT2, TPS65988_POWER_PATH_MAP_VBUS2);
    
    tps65988_set_pp_sw_conf(&tps_global_conf, TPS65988_POWER_PATH_INT1, TPS65988_PP_SW_CONF_SOURCE);
    tps65988_set_pp_sw_conf(&tps_global_conf, TPS65988_POWER_PATH_EXT1, TPS65988_PP_SW_CONF_SINK);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x6B << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, 0x01, 1 /* expect ack */);
    i2c_master_write_byte(cmd, 0b01100111, 1);
    i2c_master_stop(cmd);

    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x6B << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, 0x3A, 1 /* expect ack */);
    i2c_master_write_byte(cmd, 0b00000011, 1);
    i2c_master_write_byte(cmd, 0b11000000, 1);
    i2c_master_stop(cmd);

    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);

    /*struct A {
        uint8_t adc_conv;
        uint8_t adc_en;
    } a;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x6B << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, 0x3A, 1);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x6B << 1) | I2C_MASTER_READ, 1);
    i2c_master_read(cmd, (uint8_t *)&a, sizeof(a), I2C_MASTER_ACK);
    i2c_master_stop(cmd);

    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);

    ESP_LOGI(TAG, "vsys %d mV", a.adc_conv);
    ESP_LOGI(TAG, "bat %d mV", a.adc_en);*/
    /*cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x20 << 1) | I2C_MASTER_WRITE, 1;
    i2c_master_write_byte(cmd, TPS65988_ADDR_GLOBAL_SYSTEM_CONF, 1);
    i2c_master_write_byte(cmd, sizeof(tps_global_conf), 1);
    i2c_master_write(cmd, &tps_global_conf, sizeof(tps_global_conf), 1);
    i2c_master_stop(cmd);

    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 100 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);
    */
    //ESP_ERROR_CHECK(tps65988_i2c_write(TPS65988_ADDR_GLOBAL_SYSTEM_CONF, &tps_global_conf, sizeof(tps_global_conf)));

    uint8_t power_status[16] = {0};
    while (1) {
        //ESP_LOGI(TAG, "log -> UART");
        //vTaskDelay(1000 / portTICK_PERIOD_MS);
        //fprintf(stdout, "example: print -> stdout\n");
        //vTaskDelay(1000 / portTICK_PERIOD_MS);
        //fprintf(stderr, "example: print -> stderr\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        /*ESP_ERROR_CHECK(tps65988_i2c_read(TPS65988_ADDR_UID, power_status, sizeof(power_status), &reg_size));

        ESP_LOGI(TAG, "power status, reg size %d", reg_size);
        for (int i = 0; i<sizeof(power_status);i++){
            ESP_LOGI(TAG, "%d", buf[i]);
        };*/


        struct P {
            uint8_t vsys;
            uint8_t bat;
        } p;
        assert(sizeof(p) == 2);
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (0x6B << 1) | I2C_MASTER_WRITE, 1);
        i2c_master_write_byte(cmd, 0x2C, 1);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (0x6B << 1) | I2C_MASTER_READ, 1);
        i2c_master_read(cmd, (uint8_t *)&p, sizeof(p), I2C_MASTER_ACK);
        i2c_master_stop(cmd);

        ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS));
        i2c_cmd_link_delete(cmd);

        
       //uint8_t d[] = {0x2C};
       //i2c_master_write_to_device(I2C_NUM_0, 0x6B, d, sizeof(d), 1000 / portTICK_PERIOD_MS);
       //i2c_master_read_from_device(I2C_NUM_0, 0x6B, (uint8_t *)&p, sizeof(p), 1000 / portTICK_PERIOD_MS);


        ESP_LOGI(TAG, "vsys %d mV", ((uint16_t)p.vsys)<<6);
        ESP_LOGI(TAG, "bat %d mV", ((uint16_t)p.bat)<<6);

        uint8_t bq_mfr;
        bq2573_get_mfr_id(&bq_mfr);
        ESP_LOGI(TAG, "BQ MFR 0x%X", bq_mfr);
        

        /*esp_tusb_init_console(TINYUSB_CDC_ACM_0); // log to usb
        ESP_LOGI(TAG, "log -> USB");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        fprintf(stdout, "example: print -> stdout\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        fprintf(stderr, "example: print -> stderr\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        esp_tusb_deinit_console(TINYUSB_CDC_ACM_0); // log to uart*/
    }
}
