#ifndef BQ2573_H
#define BQ2573_H

#include "esp_err.h"

#define BQ2573_DEVICE_ID_SYSTEM             0x6B

#define BQ2573_ADDR_CHARGE_OPTION_0         0x00
#define BQ2573_ADDR_CHARGE_CURRENT          0x02
#define BQ2573_ADDR_CHARGE_VOLTAGE          0x04
#define BQ2573_ADDR_OTG_VOLTAGE             0x06
#define BQ2573_ADDR_OTG_CURRENT             0x08
#define BQ2573_ADDR_INPUT_VOLTAGE           0x0A
#define BQ2573_ADDR_IIN_HOST                0x0E
#define BQ2573_ADDR_CHARGER_STATUS          0x20
#define BQ2573_ADDR_PROCHOT_STATUS          0x22
#define BQ2573_ADDR_IIN_DMP                 0x24
#define BQ2573_ADDR_ADC_PSYS                0x26
#define BQ2573_ADDR_ADC_VBUS                0x27
#define BQ2573_ADDR_ADC_IBAT                0x28
#define BQ2573_ADDR_ADC_IIN_CMPIN           0x2A
#define BQ2573_ADDR_ADC_VSYS_VBAT           0x2C
#define BQ2573_ADDR_ADC_MFR_ID              0x2E //should return always 0x40
#define BQ2573_ADDR_ADC_DEVICE_ID           0x2F //should return 0xD6
#define BQ2573_ADDR_CHARGE_OPTION_1         0x30
#define BQ2573_ADDR_CHARGE_OPTION_2         0x32
#define BQ2573_ADDR_CHARGE_OPTION_3         0x34
#define BQ2573_ADDR_PROCHOT_OPTION_0        0x36
#define BQ2573_ADDR_PROCHOT_OPTION_1        0x38
#define BQ2573_ADDR_ADC_OPTION              0x3A
#define BQ2573_ADDR_CHARGE_OPTION_4         0x3C
#define BQ2573_ADDR_VMIN_ACTIVE_PROTECTION  0x3E

esp_err_t bq2573_get_adc_vbus(uint8_t *ret_id);
esp_err_t bq2573_get_adc_vbus(uint8_t *ret_id);
esp_err_t bq2573_get_adc_ichrg(uint8_t *ret_id);
esp_err_t bq2573_get_adc_idchrg(uint8_t *ret_id);
esp_err_t bq2573_get_adc_iinc(uint8_t *ret_id);
esp_err_t bq2573_get_adc_cmpin(uint8_t *ret_id);
esp_err_t bq2573_get_adc_vsys(uint8_t *ret_id);
esp_err_t bq2573_get_adc_vbat(uint8_t *ret_id);

esp_err_t bq2573_get_mfr_id(uint8_t *ret_id);
esp_err_t bq2573_get_device_id(uint8_t *ret_id);


#endif