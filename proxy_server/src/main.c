/* Copyright (c) 2010 - 2018, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
*
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "nrf_drv_lpcomp.h"
#include "nrf_drv_timer.h"
#include <stdint.h>
#include <string.h>
/* HAL */
#include "app_timer.h"
#include "boards.h"
#include "simple_hal.h"

/* ADC*/
#include "app_util_platform.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_lpcomp.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_timer.h"
#include "nrf_pwr_mgmt.h"

/* Core */
#include "access_config.h"
#include "ble_conn_params.h"
#include "ble_hci.h"
#include "device_state_manager.h"
#include "mesh_adv.h"
#include "mesh_config.h"
#include "mesh_opt_gatt.h"
#include "mesh_stack.h"
#include "net_state.h"
#include "nrf_mesh.h"
#include "nrf_mesh_configure.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "proxy.h"
#include "sdk_config.h"

/* Provisioning and configuration */
#include "mesh_app_utils.h"
#include "mesh_provisionee.h"
#include "mesh_softdevice_init.h"

/* Models */
#include "generic_on_off_client.h"
#include "generic_on_off_server.h"

/* Logging and RTT */
#include "log.h"
#include "rtt_input.h"

/* Example specific includes */
#include "app_config.h"
#include "example_common.h"
#include "nrf_mesh_config_examples.h"
//#include "light_switch_example_common.h"
#include "simple_pwm.h"


#include "nrf_power.h"
#include "nrf_bootloader_info.h"
#include "ble_dfu.h"


#define solar_panel 4
#define SAMPLES_IN_BUFFER 1
#define sample 250
#define vol_sample 250
#define led_on 40
#define led_off 760
#define LED_PIN_NUMBER 2 //(BSP_LED_0)
APP_TIMER_DEF(Led_Blink);
uint32_t Led_blink_intervel;

APP_TIMER_DEF(send_data);
uint32_t send_data_intervel = APP_TIMER_TICKS(3200);

APP_TIMER_DEF(Server_timeout);
uint32_t Server_timeout_intervel = APP_TIMER_TICKS(30 * 1000);

APP_TIMER_DEF(solarpanel_timer_id);
uint32_t SOLAR_STATE_MEAS_INTERVAL = APP_TIMER_TICKS(10000);

extern unsigned int Received_data;
extern bool mesh_node_reset;

unsigned int node_id;

static generic_on_off_server_t m_server;
static generic_on_off_client_t m_client;
static bool m_device_provisioned;
static bool m_led_flag = false;
static bool m_on_off_button_flag = true;

// TODO: Hands on 2.1 - Change the DEVICE_NAME to something unique
#define DEVICE_NAME "SOLAR_RPM"

#define MIN_CONN_INTERVAL MSEC_TO_UNITS(25, UNIT_1_25_MS)  /**< Minimum acceptable connection interval. was 250 */
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(100, UNIT_1_25_MS) /**< Maximum acceptable connection interval. was 1000 */
#define GROUP_MSG_REPEAT_COUNT (1)
#define SLAVE_LATENCY 0                                     /**< Slave latency. */
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(4000, UNIT_10_MS)    /**< Connection supervisory timeout (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(100) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called. */
#define NEXT_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(2000) /**< Time between each call to sd_ble_gap_conn_param_update after the first call. */
#define MAX_CONN_PARAMS_UPDATE_COUNT 3                      /**< Number of attempts before giving up the connection parameter negotiation. */

float adc = 0;
uint32_t BUTTON_STATE_MEAS_INTERVAL;

static nrf_saadc_value_t m_buffer_pool[2][SAMPLES_IN_BUFFER];
uint8_t i = 1;
uint8_t vol_sam = 1;
uint8_t solar_flag = 0;
uint8_t start_readings = 0;
char buf[6];
unsigned long int temp_var;
uint16_t adc_buff[sample];
float voltage_sample[vol_sample];
static uint32_t m_adc_evt_counter;
volatile uint8_t state = 1;
float dustDensity = 0, val = 0, voMeasured = 0;
char str[10];
void adc_comp();
void stock_mode();


static void buttonless_dfu_sdh_state_observer(nrf_sdh_state_evt_t state, void * p_context)
{
    if (state == NRF_SDH_EVT_STATE_DISABLED)
    {
        // Softdevice was disabled before going into reset. Inform bootloader to skip CRC on next boot.
        nrf_power_gpregret2_set(BOOTLOADER_DFU_SKIP_CRC);

        //Go to system off.
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    }
}

/* nrf_sdh state observer. */
//NRF_SDH_STATE_OBSERVER(m_buttonless_dfu_state_obs, 0) =
//{
//    .handler = buttonless_dfu_sdh_state_observer,
//};

// YOUR_JOB: Update this code if you want to do anything given a DFU event (optional).
/**@brief Function for handling dfu events from the Buttonless Secure DFU service
 *
 * @param[in]   event   Event from the Buttonless Secure DFU service.
 */
static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event)
{
    switch (event)
    {
        case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
             __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,"Device is preparing to enter bootloader mode.");
            // YOUR_JOB: Disconnect all bonded devices that currently are connected.
            //           This is required to receive a service changed indication
            //           on bootup after a successful (or aborted) Device Firmware Update.
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER:
            // YOUR_JOB: Write app-specific unwritten data to FLASH, control finalization of this
            //           by delaying reset by reporting false in app_shutdown_handler
             __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,"Device will enter bootloader mode.");
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
             __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,"Request to enter bootloader mode failed asynchroneously.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            break;

        case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,"Request to send a response to client failed.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            APP_ERROR_CHECK(false);
            break;

        default:
           __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,"Unknown event from ble_dfu_buttonless.");
            break;
    }
}




static void services_init(void)
{
    uint32_t                  err_code;
  
    ble_dfu_buttonless_init_t dfus_init = {0};

    // Initialize Queued Write Module.
  

    // Initialize the async SVCI interface to bootloader.
    err_code = ble_dfu_buttonless_async_svci_init();
    //APP_ERROR_CHECK(err_code);

    dfus_init.evt_handler = ble_dfu_evt_handler;

    err_code = ble_dfu_buttonless_init(&dfus_init);


}



void saadc_callback(nrf_drv_saadc_evt_t const *p_event)
{

    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);
        if (i <= sample)
        {
            adc_buff[i] = p_event->data.done.p_buffer[0];
            i++;
            if (i == sample + 1)
            {
                int adc_temp = 0;
                for (int j = 1; j <= sample; j++)
                {
                    adc_temp = adc_temp + adc_buff[j];
                }
                adc_temp = adc_temp / (sample);
                adc_temp = adc_temp;
                adc = ((((adc_temp)*3.3) / 16384) * 1.068) + 0.1;

                // adc = ((((adc_temp) * 3.3)/16384)*6.2334);
                //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "-----  ADC %d -----\n", adc_temp);
                voltage_sample[vol_sam] = adc; //- 0.053;
                i = 0;
                vol_sam++;
                if (vol_sam == vol_sample + 1)
                {
                    float vol_temp = 0;
                    for (int j = 1; j <= vol_sample; j++)
                    {
                        vol_temp = vol_temp + voltage_sample[j];
                    }
                    vol_temp = vol_temp / (vol_sample);
                    temp_var = (float)vol_temp * 1000;
                    buf[5] = 0x00;
                    buf[4] = temp_var % 10;
                    buf[4] |= 0x30;
                    temp_var /= 10;
                    buf[3] = temp_var % 10;
                    buf[3] |= 0x30;
                    temp_var /= 10;
                    buf[2] = temp_var % 10;
                    buf[2] |= 0x30;
                    temp_var /= 10;
                    buf[1] = '.';
                    buf[0] = temp_var % 10;
                    buf[0] |= 0x30;
                    temp_var /= 10;
                    // //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "-----  ADC %d -----\n", adc_temp);
                   // __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Voltage %s\n", buf);
                   if (mesh_node_reset == true)
                    { mesh_node_reset = false;
                    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Voltage %s\n", buf);
        //  app_timer_stop(send_data);
                      }
                    
                    vol_sam = 0;
                    if (vol_temp < 2.25)
                    {
                       adc_comp();
                    }
                }
            }
        }
    }
}



void saadc_init(void)

{

    //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "INIT ADC");
    ret_code_t err_code;

    nrf_drv_saadc_config_t saadc_config = NRF_DRV_SAADC_DEFAULT_CONFIG;
    saadc_config.resolution = NRF_SAADC_RESOLUTION_14BIT;

    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN3);

    //  channel_config.gain = NRF_SAADC_GAIN1_4;
    channel_config.gain = NRF_SAADC_GAIN1_4;
    channel_config.reference = 3.3;
    //  channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL;

    err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}

static bool m_device_provisioned;

static void gap_params_init(void);
static void conn_params_init(void);

static void on_sd_evt(uint32_t sd_evt, void *p_context)
{
    (void)nrf_mesh_on_sd_evt(sd_evt);
}

NRF_SDH_SOC_OBSERVER(mesh_observer, NRF_SDH_BLE_STACK_OBSERVER_PRIO, on_sd_evt, NULL);

static bool on_off_server_get_cb(const generic_on_off_server_t *p_server)
{
    return m_led_flag;
}

static bool on_off_server_set_cb(const generic_on_off_server_t *p_server, unsigned int value)
{
    // TODO: Hands on 2.3 - After initializing the PWM library in main(), change this function to use the PWM Driver instead of the hal_led_.. functions
    //                      Try to make the LED's fade in and out when the callback occurs, rather than having it set/cleared immediately
    uint32_t err_code;

 //  __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Got SET received data : %u  value :%u \n", Received_data, value);
    if (Received_data > 0 && value != 0)
    {

        app_timer_stop(Server_timeout);
        app_timer_stop(Led_Blink);

        if (Received_data < node_id && Received_data != node_id)
        {
            app_timer_stop(send_data);
        }
        hal_led_pin_set(LED_PIN_NUMBER, 0);
        uint8_t status = hal_led_pin_get(LED_PIN_NUMBER);
        if (status == 1)
        {
            Led_blink_intervel = APP_TIMER_TICKS(led_off);
        }
        else
        {
            Led_blink_intervel = APP_TIMER_TICKS(led_on);
        }
        app_timer_start(Led_Blink, Led_blink_intervel, NULL);
        app_timer_start(Server_timeout, Server_timeout_intervel, NULL);
        Received_data = 0;
    }
    else if (value == 0)
    {
        stock_mode();
    }

    return value;
}

static void node_reset(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Node reset  -----\n");
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_RESET);

    /* This function may return if there are ongoing flash operations. */
    mesh_stack_device_reset();
}

static void config_server_evt_cb(const config_server_evt_t *p_evt)
{
    if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET)
    {
        node_reset();
    }
}

static bool client_publication_configured(void)
{
    dsm_handle_t pub_addr_handle;

    if (access_model_publish_address_get(m_client.model_handle, &pub_addr_handle) == NRF_SUCCESS)
    {
        if (pub_addr_handle == DSM_HANDLE_INVALID)
        {
            return false;
        }
    }
    else
    {
        return false;
    }

    return true;
}

static void button_event_handler(uint32_t button_number)
{
    //   __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);

    if (client_publication_configured())
    {
        uint32_t status = NRF_SUCCESS;
        switch (button_number)
        {

        case 0:
        case 1:
            status = generic_on_off_client_set(&m_client, node_id);
            break;
        /* Initiate node reset */
        case 3:
            /* Clear all the states to reset the node. */
            if (mesh_stack_is_device_provisioned())
            {
                (void)proxy_stop();
                mesh_stack_config_clear();
                node_reset();
            }
            else
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "The device is unprovisioned. Resetting has no effect.\n");
            }
            break;

        default:
            break;
        }
    }
}

static void on_conn_params_evt(ble_conn_params_evt_t *p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(p_evt->conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
    else if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_SUCCEEDED)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully updated connection parameters\n");
    }
}

static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void provisioning_complete_cb(void)
{
    
     if (!m_device_provisioned)
    {
    
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned\n");

    /* Restores the application parameters after switching from the Provisioning service to the Proxy  */
    gap_params_init();
    conn_params_init();
     services_init();
    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x \n", node_address.address_start);

    node_id = node_address.address_start;

    hal_led_mask_set(LEDS_MASK, false);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_PROV);

    }
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Disconnected -----\n");
}

static void client_status_cb(const generic_on_off_client_t *p_self, generic_on_off_status_t status, uint16_t src)
{

   // __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "server status received \n");
    switch (status)
    {
    case GENERIC_ON_OFF_STATUS_ON:
        m_on_off_button_flag = 1;
      //  __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "set m_on_off_button_flag to 1 \n");
        break;

    case GENERIC_ON_OFF_STATUS_OFF:
        m_on_off_button_flag = 0;
     //   __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "set m_on_off_button_flag to 0 \n");
        break;

    case GENERIC_ON_OFF_STATUS_ERROR_NO_REPLY:
     //   __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "GENERIC_ON_OFF_STATUS_ERROR_NO_REPLY \n");
        break;

    case GENERIC_ON_OFF_STATUS_CANCELLED:
    default:
     //   __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Unknown status \n");
        break;
    }
}
static void client_publish_timeout_cb(access_model_handle_t handle, void *p_self)
{
 //   __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Acknowledged send timedout\n");
}

static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");
    m_server.get_cb = on_off_server_get_cb;
    m_server.set_cb = on_off_server_set_cb;
    ERROR_CHECK(generic_on_off_server_init(&m_server, 0));
    ERROR_CHECK(access_model_subscription_list_alloc(m_server.model_handle));
    //Initialize client on model 1
    m_client.status_cb = client_status_cb;
    m_client.timeout_cb = client_publish_timeout_cb;
    ERROR_CHECK(generic_on_off_client_init(&m_client, 1));
    ERROR_CHECK(access_model_subscription_list_alloc(m_client.model_handle));
}

static void mesh_init(void)
{
    uint8_t dev_uuid[NRF_MESH_UUID_SIZE];
    uint8_t node_uuid_prefix[NODE_UUID_PREFIX_LEN] = SERVER_NODE_UUID_PREFIX;

    ERROR_CHECK(mesh_app_uuid_gen(dev_uuid, node_uuid_prefix, NODE_UUID_PREFIX_LEN));
    mesh_stack_init_params_t init_params =
        {
            .core.irq_priority = NRF_MESH_IRQ_PRIORITY_LOWEST,
            .core.lfclksrc = DEV_BOARD_LF_CLK_CFG,
            .core.p_uuid = dev_uuid,
            .models.models_init_cb = models_init_cb,
            .models.config_server_cb = config_server_evt_cb};
    ERROR_CHECK(mesh_stack_init(&init_params, &m_device_provisioned));
}

static void gap_params_init(void)
{
    uint32_t err_code;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
        (const uint8_t *)DEVICE_NAME,
        strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);
}

static void conn_params_init(void)
{
    uint32_t err_code;
    ble_conn_params_init_t cp_init;
    ble_gap_conn_params_t gap_conn_params;

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));
    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);

    memset(&cp_init, 0, sizeof(cp_init));
    cp_init.p_conn_params = &gap_conn_params;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail = false;
    cp_init.evt_handler = on_conn_params_evt;
    cp_init.error_handler = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

static void initialize(void)
{

    ERROR_CHECK(app_timer_init());
    //hal_leds_init();

#if BUTTON_BOARD
    // ERROR_CHECK(hal_buttons_init(button_event_handler));
#endif
    uint32_t err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);
#if defined S140 // todo remove that after S140 priority fixing
    softdevice_irq_priority_checker();
#endif

    uint32_t ram_start = 0;
    /* Set the default configuration (as defined through sdk_config.h). */
    err_code = nrf_sdh_ble_default_cfg_set(MESH_SOFTDEVICE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    gap_params_init();
    conn_params_init();

    mesh_init();
}

static void start(void)
{
    //rtt_input_enable(app_rtt_input_handler, RTT_INPUT_POLL_PERIOD_MS);
    ERROR_CHECK(mesh_stack_start());

    if (!m_device_provisioned)
    {
        static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
        mesh_provisionee_start_params_t prov_start_params =
            {
                .p_static_data = static_auth_data,
                .prov_complete_cb = provisioning_complete_cb,
                .p_device_uri = NULL};
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    }

    const uint8_t *p_uuid = nrf_mesh_configure_device_uuid_get();
    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "Device UUID ", p_uuid, NRF_MESH_UUID_SIZE);
}

static void Led_blink_cb(void *p_context)
{

    hal_led_pin_set(LED_PIN_NUMBER, !hal_led_pin_get(LED_PIN_NUMBER));
    uint8_t status = hal_led_pin_get(LED_PIN_NUMBER);
    // //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Setting GPIO value: %d\n", status)
    if (status == 1)
    {
        Led_blink_intervel = APP_TIMER_TICKS(led_off);
    }
    else
    {
        Led_blink_intervel = APP_TIMER_TICKS(led_on);
        start_readings = 1;
    }
    app_timer_start(Led_Blink, Led_blink_intervel, NULL);
}

static void send_data_cb(void *p_context)
{
    button_event_handler(1);
}

static void server_timeout_cb(void *p_context)
{

    app_timer_start(send_data, send_data_intervel, NULL);
}

static void solar_inttrupt()
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Sleep-----\n");
    nrf_gpio_cfg_default(LED_PIN_NUMBER);
    nrf_gpio_cfg_sense_input(solar_panel, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_LOW); // Added this
    //NRF_POWER->TASKS_LOWPWR = 1;
    // NRF_POWER->SYSTEMOFF = POWER_SYSTEMOFF_SYSTEMOFF_Enter;
    sd_power_system_off();
}

void stock_mode()
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- STOCKMODE-----\n");
    nrf_gpio_cfg_default(LED_PIN_NUMBER);
    nrf_gpio_cfg_sense_input(solar_panel, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_HIGH); // Added this
    //NRF_POWER->TASKS_LOWPWR = 1;
    // NRF_POWER->SYSTEMOFF = POWER_SYSTEMOFF_SYSTEMOFF_Enter;
    sd_power_system_off();
}

static void wakeup(nrf_lpcomp_event_t event)
{
    //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- wakeup-----\n");
}

void adc_comp()
{
    nrf_gpio_cfg_default(LED_PIN_NUMBER);
    nrf_gpio_cfg_default(solar_panel);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE adc  -----\n");
    uint32_t err_code;
    nrf_drv_lpcomp_config_t config = NRF_DRV_LPCOMP_DEFAULT_CONFIG;
    config.hal.detection = NRF_LPCOMP_DETECT_UP;
    config.hal.reference = 13;
    config.input = NRF_LPCOMP_INPUT_3;
    err_code = nrf_drv_lpcomp_init(&config, wakeup);
    APP_ERROR_CHECK(err_code);
    nrf_drv_lpcomp_enable();
    while (NRF_LPCOMP->EVENTS_READY == 0);
    NRF_LPCOMP->EVENTS_READY = 0;
    sd_power_system_off();
}

int main(void)
{
    NRF_POWER->DCDCEN = 1;
   __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS | LOG_SRC_BEARER, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
   // __LOG_INIT(LOG_SRC_APP, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Light Switch Proxy Server Demo -----\n");
    initialize();
    services_init();
    execution_start(start);
    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);
    node_id = node_address.address_start;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: %d \n", node_id);
    ret_code_t ret_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(ret_code);
    saadc_init();
    uint8_t int_flag = 0;
    nrf_gpio_cfg_input(solar_panel, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_output(LED_PIN_NUMBER);
    nrf_gpio_pin_write(LED_PIN_NUMBER, 0);
    APP_ERROR_CHECK(app_timer_create(&solarpanel_timer_id, APP_TIMER_MODE_SINGLE_SHOT, solar_inttrupt));
    APP_ERROR_CHECK(app_timer_create(&Led_Blink, APP_TIMER_MODE_SINGLE_SHOT, Led_blink_cb));
    Led_blink_intervel = APP_TIMER_TICKS(1000);
    app_timer_start(Led_Blink, Led_blink_intervel, NULL);
    APP_ERROR_CHECK(app_timer_create(&send_data, APP_TIMER_MODE_REPEATED, send_data_cb));
    app_timer_start(send_data, send_data_intervel, NULL);
    APP_ERROR_CHECK(app_timer_create(&Server_timeout, APP_TIMER_MODE_REPEATED, server_timeout_cb));
    ///app_timer_start(Server_timeout, Server_timeout_intervel, NULL);

    for (;;)
    {
        (void)sd_app_evt_wait();
        nrf_pwr_mgmt_run();
        if (start_readings == 1)
        {
            nrf_drv_saadc_sample();
        }
        uint8_t sp_status = nrf_gpio_pin_read(solar_panel);
        if (sp_status == 0 && solar_flag == 1)
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Intrrupt cleared-----\n");
            app_timer_stop(solarpanel_timer_id);
            solar_flag = 0;
        }
        if (sp_status == 1 && solar_flag == 0)
        {

            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "-----Solar Intrrupt occured-----\n");
            app_timer_start(solarpanel_timer_id, SOLAR_STATE_MEAS_INTERVAL, NULL);
            solar_flag = 1;
        }

        if (mesh_node_reset == true)
        {
        //  app_timer_stop(send_data);
        }
    }
}