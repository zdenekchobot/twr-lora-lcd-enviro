#include <application.h>
#include <at.h>

#define SEND_DATA_INTERVAL (15 * 60 * 1000)
#define MEASURE_INTERVAL (15 * 1000)
#define MEASURE_INTERVAL_CO2 (15 * 1000)
#define MEASURE_INTERVAL_VOC (15 * 1000)

/*
TOWER configuration:
    CORE module NR (temperature, acceleration, security chip)
    LCD module - LCD, 2x button, 6x RGB mini LED, gesture & proximity sensor
    LoRa module
    CO2 module
    H  umidity tag
    VOC-LP tag
    Mini battery module
*/

// LED instance
twr_led_t ledr;
twr_led_t ledg;
twr_led_t ledb;
// Button instance
twr_button_t button_left;
twr_button_t button_right;
// Lora instance
twr_cmwx1zzabz_t lora;
// Accelerometer instance
twr_lis2dh12_t lis2dh12;
twr_dice_t dice;
twr_dice_face_t dice_face = TWR_DICE_FACE_1;
// Thermometer instance
twr_tmp112_t tmp112;
// Humidity tag instance
twr_tag_humidity_t humi;
//VOC tag instance
twr_tag_voc_lp_t voc_tag;
// GFX pointer
twr_gfx_t *pgfx;

TWR_DATA_STREAM_FLOAT_BUFFER(sm_voltage_buffer, 8)
TWR_DATA_STREAM_FLOAT_BUFFER(sm_temperature_buffer, (SEND_DATA_INTERVAL / MEASURE_INTERVAL))
TWR_DATA_STREAM_FLOAT_BUFFER(sm_humidity_buffer, (SEND_DATA_INTERVAL / MEASURE_INTERVAL))
TWR_DATA_STREAM_FLOAT_BUFFER(sm_co2_buffer, (SEND_DATA_INTERVAL / MEASURE_INTERVAL_CO2))
TWR_DATA_STREAM_INT_BUFFER(sm_voc_buffer, (SEND_DATA_INTERVAL / MEASURE_INTERVAL_VOC))
TWR_DATA_STREAM_INT_BUFFER(sm_orientation_buffer, 3)

twr_data_stream_t sm_voltage;
twr_data_stream_t sm_temperature;
twr_data_stream_t sm_humidity;
twr_data_stream_t sm_co2;
twr_data_stream_t sm_voc;
twr_data_stream_t sm_orientation;

twr_scheduler_task_id_t battery_measure_task_id;

enum
{
    HEADER_BOOT = 0x00,
    HEADER_UPDATE = 0x01,
    HEADER_BUTTON_CLICK = 0x02,
    HEADER_BUTTON_HOLD = 0x03,

} header = HEADER_BOOT;

void button_event_handler(twr_button_t *self, twr_button_event_t event, void *event_param)
{
    if (event == TWR_BUTTON_EVENT_CLICK && event_param == 0)
    {
        header = HEADER_BUTTON_CLICK;

        twr_scheduler_plan_now(0);
    }
    else if (event == TWR_BUTTON_EVENT_HOLD)
    {
        header = HEADER_BUTTON_HOLD;

        twr_scheduler_plan_now(0);
    }
    else if (event == TWR_BUTTON_EVENT_CLICK && (int)event_param == 1)
    {
        twr_led_set_mode(&ledr, TWR_LED_MODE_TOGGLE);
    }
}

void lcd_event_handler(twr_module_lcd_event_t event, void *event_param)
{
}

void tmp112_event_handler(twr_tmp112_t *self, twr_tmp112_event_t event, void *event_param)
{
    float value = NAN;
    if (event == TWR_TMP112_EVENT_UPDATE)
    {
        twr_tmp112_get_temperature_celsius(self, &value);
        twr_log_debug("CORE: Temperature: %.1f °C", value);
    }
}

void tag_humidity_event_handler(twr_tag_humidity_t *self, twr_tag_humidity_event_t event, void *event_param)
{
    float temperature = NAN;
    float humidity = NAN;
    if (event == TWR_TAG_HUMIDITY_EVENT_UPDATE)
    {
        if (twr_tag_humidity_get_humidity_percentage(self, &humidity))
        {
            twr_data_stream_feed(&sm_humidity, &humidity);
            twr_log_debug("HUMIDITY TAG: Humidity: %.0f %%", humidity);
        }
        else
        {
            twr_log_debug("HUMIDITY TAG: Invalid humidity value");
        }

        if (twr_tag_humidity_get_temperature_celsius(self, &temperature))
        {
            twr_data_stream_feed(&sm_temperature, &temperature);
            twr_log_debug("HUMIDITY TAG: Temperature: %.1f °C", temperature);
        }
        else
        {
            twr_log_debug("HUMIDITY TAG: Temperature value invalid");
        }
    }
}

void voc_tag_event_handler(twr_tag_voc_lp_t *self, twr_tag_voc_lp_event_t event, void *event_param)
{
    uint16_t value;
    /*
    if (twr_tag_voc_lp_measure(self))
        twr_log_debug("VOC measure OK");
    else
        twr_log_debug("VOC measure error");
    */
    if (event == TWR_TAG_VOC_LP_EVENT_UPDATE)
    {
        twr_log_debug("VOC MODULE get ppb");
        if (twr_tag_voc_lp_get_tvoc_ppb(self, &value))
        {
            int retyped_value = (int)value;
            twr_data_stream_feed(&sm_voc, &retyped_value);
            twr_log_debug("VOC MODULE: VOC: %i ppb", value);
        }
        else
        {
            twr_log_debug("VOC MODULE: Invalid VOC value");
        }
    }
    // float = twr_tag_voc_lp_set_compensation()
}

void battery_event_handler(twr_module_battery_event_t event, void *event_param)
{
    if (event == TWR_MODULE_BATTERY_EVENT_UPDATE)
    {
        float value = NAN;
        if (twr_module_battery_get_voltage(&value))
        {
            twr_data_stream_feed(&sm_voltage, &value);
            twr_log_debug("BATTERY MODULE: Voltage %.1f V", value);
        }
        else
        {
            twr_log_debug("BATTERY MODULE: Invalid voltage value");
        }
    }
}

void co2_module_event_handler(twr_module_co2_event_t event, void *event_param)
{
    if (event == TWR_MODULE_CO2_EVENT_UPDATE)
    {
        float value = NAN;
        if (twr_module_co2_get_concentration_ppm(&value))
        {
            twr_data_stream_feed(&sm_co2, &value);
            twr_log_debug("CO2 MODULE: CO2: %.1f ppm", value);
        }
        else
        {
            twr_log_debug("CO2 MODULE: CO2 measure failed");
        }
    }
}

void lis2dh12_event_handler(twr_lis2dh12_t *self, twr_lis2dh12_event_t event, void *event_param)
{
    if (event == TWR_LIS2DH12_EVENT_UPDATE)
    {
        twr_lis2dh12_result_g_t g;

        if (twr_lis2dh12_get_result_g(self, &g))
        {
            twr_dice_feed_vectors(&dice, g.x_axis, g.y_axis, g.z_axis);

            int orientation = (int)twr_dice_get_face(&dice);

            twr_data_stream_feed(&sm_orientation, &orientation);
        }
    }
}

void battery_measure_task(void *param)
{
    if (!twr_module_battery_measure())
    {
        twr_scheduler_plan_current_now();
    }
}

void lora_callback(twr_cmwx1zzabz_t *self, twr_cmwx1zzabz_event_t event, void *event_param)
{
    if (event == TWR_CMWX1ZZABZ_EVENT_ERROR)
    {
        twr_led_set_mode(&ledg, TWR_LED_MODE_BLINK_FAST);
    }
    else if (event == TWR_CMWX1ZZABZ_EVENT_SEND_MESSAGE_START)
    {
        twr_led_set_mode(&ledg, TWR_LED_MODE_ON);

        twr_scheduler_plan_relative(battery_measure_task_id, 20);
    }
    else if (event == TWR_CMWX1ZZABZ_EVENT_SEND_MESSAGE_DONE)
    {
        twr_led_set_mode(&ledg, TWR_LED_MODE_OFF);
    }
    else if (event == TWR_CMWX1ZZABZ_EVENT_READY)
    {
        twr_led_set_mode(&ledg, TWR_LED_MODE_OFF);
    }
    else if (event == TWR_CMWX1ZZABZ_EVENT_JOIN_SUCCESS)
    {
        twr_atci_printf("$JOIN_OK");
    }
    else if (event == TWR_CMWX1ZZABZ_EVENT_JOIN_ERROR)
    {
        twr_atci_printf("$JOIN_ERROR");
    }
}

bool at_send(void)
{
    twr_scheduler_plan_now(0);

    return true;
}

bool at_status(void)
{
    float value_avg = NAN;

    static const struct
    {
        twr_data_stream_t *stream;
        const char *name;
        int precision;
    } values[] = {
        {&sm_voltage, "Voltage", 1},
        {&sm_temperature, "Temperature", 1},
        {&sm_humidity, "Humidity", 1},
        {&sm_co2, "CO2", 1},
        {&sm_voc, "VOC", 0},
    };

    for (size_t i = 0; i < sizeof(values) / sizeof(values[0]); i++)
    {
        value_avg = NAN;

        if (twr_data_stream_get_average(values[i].stream, &value_avg))
        {
            twr_atci_printf("$STATUS: \"%s\",%.*f\n", values[i].name, values[i].precision, value_avg);
        }
        else
        {
            twr_atci_printf("$STATUS: \"%s\",\n", values[i].name);
        }
    }

    int orientation;

    if (twr_data_stream_get_median(&sm_orientation, &orientation))
    {
        twr_atci_printf("$STATUS: \"Orientation\",%d\n", orientation);
    }
    else
    {
        twr_atci_printf("$STATUS: \"Orientation\",\n", orientation);
    }

    return true;
}

void application_init(void)
{
    // Initialize logging
    twr_log_init(TWR_LOG_LEVEL_DUMP, TWR_LOG_TIMESTAMP_ABS);
 
    twr_data_stream_init(&sm_voltage, 1, &sm_voltage_buffer);
    twr_data_stream_init(&sm_temperature, 1, &sm_temperature_buffer);
    twr_data_stream_init(&sm_humidity, 1, &sm_humidity_buffer);
    twr_data_stream_init(&sm_co2, 1, &sm_co2_buffer);
    twr_data_stream_init(&sm_voc, 1, &sm_voc_buffer);
    twr_data_stream_init(&sm_orientation, 1, &sm_orientation_buffer);


    // Initialize LED
    const twr_led_driver_t *driver = twr_module_lcd_get_led_driver();
    twr_led_init_virtual(&ledg, TWR_MODULE_LCD_LED_GREEN, driver, 1);
    twr_led_init_virtual(&ledr, TWR_MODULE_LCD_LED_RED, driver, 1);
    twr_led_init_virtual(&ledb, TWR_MODULE_LCD_LED_BLUE, driver, 1);
    // twr_led_set_mode(&led, TWR_LED_MODE_ON);

    // Initialize button
    const twr_button_driver_t *lcdButtonDriver = twr_module_lcd_get_button_driver();
    twr_button_init_virtual(&button_left, 0, lcdButtonDriver, 0);
    twr_button_init_virtual(&button_right, 1, lcdButtonDriver, 0);
    twr_button_set_event_handler(&button_left, button_event_handler, (int *)0);
    twr_button_set_event_handler(&button_right, button_event_handler, (int *)1);

    // Initialize LCD module
    twr_module_lcd_init();
    twr_module_lcd_set_event_handler(lcd_event_handler, NULL);
    pgfx = twr_module_lcd_get_gfx();

    // Initialize TMP112 - thermometer on Core board
    twr_tmp112_init(&tmp112, TWR_I2C_I2C0, 0x49);
    // twr_tmp112_set_update_interval(&tmp112, MEASURE_INTERVAL);
    twr_tmp112_set_event_handler(&tmp112, tmp112_event_handler, NULL);

    //Initialize humidity tag
    twr_tag_humidity_init(&humi, TWR_TAG_HUMIDITY_REVISION_R3, TWR_I2C_I2C0, 0x40);
    twr_tag_humidity_set_event_handler(&humi, tag_humidity_event_handler, NULL);
    twr_tag_humidity_set_update_interval(&humi, MEASURE_INTERVAL);

    // Initialize VOC tag
    twr_log_debug("VOC_LP tag init started");
    twr_tag_voc_lp_init(&voc_tag, TWR_I2C_I2C0);
    twr_tag_voc_lp_set_event_handler(&voc_tag, voc_tag_event_handler, NULL);
    twr_tag_voc_lp_set_update_interval(&voc_tag, MEASURE_INTERVAL_VOC);
    twr_log_debug("VOC_LP tag init finished");

    // Initialize battery
    twr_module_battery_init();
    twr_module_battery_set_event_handler(battery_event_handler, NULL);
    twr_module_battery_set_threshold_levels((4 * 1.7), (4 * 1.6));
    battery_measure_task_id = twr_scheduler_register(battery_measure_task, NULL, 2020);

    // Initialize accelerometer
    twr_dice_init(&dice, TWR_DICE_FACE_UNKNOWN);
    twr_lis2dh12_init(&lis2dh12, TWR_I2C_I2C0, 0x19);
    twr_lis2dh12_set_resolution(&lis2dh12, TWR_LIS2DH12_RESOLUTION_8BIT);
    twr_lis2dh12_set_event_handler(&lis2dh12, lis2dh12_event_handler, NULL);
    twr_lis2dh12_set_update_interval(&lis2dh12, MEASURE_INTERVAL);

    // Initialize lora module
    twr_cmwx1zzabz_init(&lora, TWR_UART_UART1);
    twr_cmwx1zzabz_set_event_handler(&lora, lora_callback, NULL);
    twr_cmwx1zzabz_set_class(&lora, TWR_CMWX1ZZABZ_CONFIG_CLASS_A);

    //Initialize CO2 module
    twr_module_co2_init();
    twr_module_co2_set_event_handler(co2_module_event_handler, NULL);
    twr_module_co2_set_update_interval(MEASURE_INTERVAL_CO2);

    // Initialize AT command interface
    at_init(&ledg, &lora);
    static const twr_atci_command_t commands[] = {
        AT_LORA_COMMANDS,
        {"$SEND", at_send, NULL, NULL, NULL, "Immediately send packet"},
        {"$STATUS", at_status, NULL, NULL, NULL, "Show status"},
        AT_LED_COMMANDS,
        TWR_ATCI_COMMAND_CLAC,
        TWR_ATCI_COMMAND_HELP};
    twr_atci_init(commands, TWR_ATCI_COMMANDS_LENGTH(commands));

    twr_scheduler_plan_current_relative(10 * 1000);
}

void application_task(void)
{
    if (!twr_cmwx1zzabz_is_ready(&lora))
    {
        twr_scheduler_plan_current_relative(100);

        return;
    }

    static uint8_t buffer[10];

    memset(buffer, 0xff, sizeof(buffer));

    buffer[0] = header;

    float voltage_avg = NAN;

    twr_data_stream_get_average(&sm_voltage, &voltage_avg);

    if (!isnan(voltage_avg))
    {
        buffer[1] = ceil(voltage_avg * 10.f);
    }

    int orientation;

    if (twr_data_stream_get_median(&sm_orientation, &orientation))
    {
        buffer[2] = orientation;
    }

    float temperature_avg = NAN;

    twr_data_stream_get_average(&sm_temperature, &temperature_avg);

    if (!isnan(temperature_avg))
    {
        int16_t temperature_i16 = (int16_t)(temperature_avg * 10.f);

        buffer[3] = temperature_i16 >> 8;
        buffer[4] = temperature_i16;
    }

    float humidity_avg = NAN;

    twr_data_stream_get_average(&sm_humidity, &humidity_avg);

    if (!isnan(humidity_avg))
    {
        buffer[5] = humidity_avg * 2;
    }

    float co2_avg = NAN;

    twr_data_stream_get_average(&sm_co2, &co2_avg);

    if (!isnan(co2_avg))
    {
        if (co2_avg > 65534)
        {
            co2_avg = 65534;
        }

        uint16_t value = (uint16_t)co2_avg;
        buffer[6] = value >> 8;
        buffer[7] = value;
    }

    float voc_avg = NAN;

    twr_data_stream_get_average(&sm_voc, &voc_avg);

    if (!isnan(voc_avg))
    {
        uint16_t value = voc_avg / 2.f;
        buffer[8] = value >> 8;
        buffer[9] = value;
    }

    twr_cmwx1zzabz_send_message(&lora, buffer, sizeof(buffer));

    static char tmp[sizeof(buffer) * 2 + 1];
    for (size_t i = 0; i < sizeof(buffer); i++)
    {
        sprintf(tmp + i * 2, "%02x", buffer[i]);
    }

    twr_atci_printf("$SEND: %s\n", tmp);

    header = HEADER_UPDATE;

    twr_scheduler_plan_current_relative(SEND_DATA_INTERVAL);
}
