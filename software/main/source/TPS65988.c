#include "TPS65988.h"
#include "driver/i2c.h"
#include <string.h>
#include "esp_log.h"

static const char *TAG = "TPS65988";

esp_err_t tps65988_set_spi_read_only(TPS65988_GLOBAL_SYSTEM_CONF_T *conf, uint8_t state){
    return ESP_OK;
}

esp_err_t tps65988_set_i2c_timeout(TPS65988_GLOBAL_SYSTEM_CONF_T *conf, enum TPS65988_I2C_TIMEOUT timeout) {
    return ESP_OK;
}

esp_err_t tps65988_set_multiport_sink_non_overlap_time(TPS65988_GLOBAL_SYSTEM_CONF_T *conf, enum TPS65988_MULTIPORT_SINK_NON_OVERLAP_TIME time){
    return ESP_OK;
}

esp_err_t tps65988_set_multiport_policy(TPS65988_GLOBAL_SYSTEM_CONF_T *conf, enum TPS65988_MULTIPORT_POLICY policy, uint8_t value){
    return ESP_OK;
}

esp_err_t tps65988_set_tbt_controller_type(TPS65988_GLOBAL_SYSTEM_CONF_T *conf, enum TPS65988_TBT_CONTROLLER_TYPE type){
    return ESP_OK;
}

esp_err_t tps65988_set_i2c_master(TPS65988_GLOBAL_SYSTEM_CONF_T *conf, enum TPS65988_I2C_MASTER enable_ports){
    return ESP_OK;
}

esp_err_t tps65988_set_pp_int_occ_timeout(TPS65988_GLOBAL_SYSTEM_CONF_T *conf, enum TPS65988_POWER_PATH pp, uint8_t en, uint8_t timeout){
    return ESP_OK;
}

esp_err_t tps65988_set_pp_vbus_map(TPS65988_GLOBAL_SYSTEM_CONF_T *conf, enum TPS65988_POWER_PATH pp, enum TPS65988_POWER_PATH_VBUS_MAP vbus){
    conf->power_path_map = ((conf->power_path_map)&~(1<<pp))|(vbus<<pp);
    return ESP_OK;
}

esp_err_t tps65988_set_pp_sw_conf(TPS65988_GLOBAL_SYSTEM_CONF_T *conf, enum TPS65988_POWER_PATH pp, enum TPS65988_PP_SW_CONF pp_conf){
    uint32_t mask = 6;
    uint32_t new = pp_conf;
    switch (pp){
        case TPS65988_POWER_PATH_INT1:
            mask <<= 0;
            pp_conf <<= 0;
            break;
        case TPS65988_POWER_PATH_INT2:
            mask <<= 3;
            pp_conf <<= 3;
            break;
        case TPS65988_POWER_PATH_EXT1:
            mask <<= 16;
            pp_conf <<= 16;
            break;
        case TPS65988_POWER_PATH_EXT2:
            mask <<= 19;
            pp_conf <<= 19;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }
    conf->pp_config = ((conf->pp_config)&(~mask))|new;
    return ESP_OK;
}

esp_err_t tps65988_i2c_write(uint8_t reg, uint8_t *data, size_t len){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TPS65988_DEVICE_ID_SYSTEM << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
    i2c_master_write_byte(cmd, reg, 1);
    i2c_master_write_byte(cmd, len, 1);
    i2c_master_write(cmd, data, len, 1);
    i2c_master_stop(cmd);

    esp_err_t res = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return res;
}
esp_err_t tps65988_i2c_read(uint8_t reg, uint8_t *buf, size_t buf_len, uint8_t *reg_len){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TPS65988_DEVICE_ID_SYSTEM << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, reg, 1 /* expect ack */);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TPS65988_DEVICE_ID_SYSTEM << 1) | I2C_MASTER_READ, 1);
    uint8_t reg_size = 0;
    i2c_master_read_byte(cmd, &reg_size, I2C_MASTER_ACK);

    esp_err_t res = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (res != ESP_OK){
        return res;
    }

    if (reg_size != buf_len){
        ESP_LOGW(TAG, "buf size (%d) doesn't match actual reg size (%d)", buf_len, reg_size);
    }

    uint8_t reg_data[reg_size];
    cmd = i2c_cmd_link_create();
    i2c_master_read(cmd, reg_data, reg_size, I2C_MASTER_ACK);
    i2c_master_stop(cmd);

    res = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    memcpy(buf, reg_data, buf_len);
    return res;
}