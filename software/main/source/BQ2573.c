#include "BQ2573.h"
#include "driver/i2c.h"

static esp_err_t bq2573_i2c_read(uint8_t reg, uint8_t *buf, uint8_t len){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BQ2573_DEVICE_ID_SYSTEM << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, reg, 1);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BQ2573_DEVICE_ID_SYSTEM << 1) | I2C_MASTER_READ, 1);
    i2c_master_read(cmd, buf, len, I2C_MASTER_ACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

esp_err_t bq2573_get_mfr_id(uint8_t *ret_id){
    return bq2573_i2c_read(BQ2573_ADDR_ADC_MFR_ID, ret_id, 1);
}

esp_err_t bq2573_get_device_id(uint8_t *ret_id){
    return bq2573_i2c_read(BQ2573_ADDR_ADC_DEVICE_ID, ret_id, 1);
}