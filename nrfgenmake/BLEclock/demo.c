#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_drv_config.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"
#include "nrf_drv_gpiote.h" 
#include "nrf_drv_twi.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_bas.h"
#include "ble_dis.h"
#include "boards.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "device_manager.h"
#include "pstorage.h"

#define MANUFACTURER_NAME "Signal Labs"
#define APP_TIMER_OP_QUEUE_SIZE 3
#define APP_TIMER_PRESCALER 0 
#define BLE_UUID_OUR_BASE_UUID {0x23, 0xD1, 0x13, 0xEF, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}
#define BLE_UUID_OUR_SERVICE 0x152C
#define CLOCK_READ_UUID 0x3907
#define CLOCK_READ_INDX 0
#define CLOCK_WRITE_UUID 0x3909
#define CLOCK_WRITE_INDX 1
#define NUMBER_OUR_ATTRIBUTES 2

typedef struct {
  uint8_t second;   // when read, this is BCD representation
  uint8_t minute;   // also BCD
  uint8_t hour;     // hours in BCD
  uint8_t weekday;  // range is 1-7
  uint8_t day;      // range is 1-31, BCD
  uint8_t month;    // BCD of 1-12, but high order bit is century 
  uint8_t year;     // again, BCD, but year (minus 1900 or 2000)
  } DS3231clock_t; 
typedef struct {
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  } clock_read_t;
static clock_read_t clock;  
static DS3231clock_t setclock;  
const uint16_t our_attribute_uuid_list[NUMBER_OUR_ATTRIBUTES] = {CLOCK_READ_UUID,CLOCK_WRITE_UUID};
const uint8_t our_attribute_lengths[NUMBER_OUR_ATTRIBUTES] = {sizeof(clock),sizeof(setclock)};
uint8_t* our_attribute_pointers[NUMBER_OUR_ATTRIBUTES] = {(uint8_t*)&clock,(uint8_t*)&setclock};
typedef struct {
  ble_uuid_t uuid;
  ble_gatts_char_md_t char_md;
  ble_gatts_attr_md_t cccd_md;
  ble_gatts_attr_md_t attr_md;
  ble_gatts_attr_t  attr_char_value;
  } charvar_t; 
typedef struct {
  uint16_t conn_handle;
  uint16_t service_handle; 
  ble_gatts_char_handles_t char_handles[NUMBER_OUR_ATTRIBUTES];
  } ble_os_t;
static charvar_t our_attributes[NUMBER_OUR_ATTRIBUTES];
static ble_os_t m_our_service;
static dm_application_instance_t m_app_handle;
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;   
bool connected = false;
const uint8_t leds_list[5] = {18,19,20,21,22};
APP_TIMER_DEF(timer_id);
static ble_uuid_t m_adv_uuids[2] = {{BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE},
                                    {BLE_UUID_OUR_SERVICE, BLE_UUID_TYPE_BLE}};
static const nrf_drv_twi_t m_twi_port = NRF_DRV_TWI_INSTANCE(0);
uint8_t twi_buffer[8];

static void gpioteHandler(nrf_drv_gpiote_pin_t p, nrf_gpiote_polarity_t d);
static void gpioteInit() {
  nrf_drv_gpiote_in_config_t config16 = GPIOTE_CONFIG_IN_SENSE_HITOLO(false);
  nrf_drv_gpiote_in_config_t config17 = GPIOTE_CONFIG_IN_SENSE_HITOLO(false);
  config16.pull = NRF_GPIO_PIN_PULLUP;
  config17.pull = NRF_GPIO_PIN_PULLUP;
  if (!nrf_drv_gpiote_is_init()) nrf_drv_gpiote_init();
  nrf_drv_gpiote_in_uninit(16);
  nrf_drv_gpiote_in_init(16,&config16,gpioteHandler);
  nrf_drv_gpiote_in_event_enable(16,true);
  nrf_drv_gpiote_in_uninit(17);
  nrf_drv_gpiote_in_init(17,&config17,gpioteHandler);
  nrf_drv_gpiote_in_event_enable(17,true);
  }
static void gpioInit() {
  uint8_t i;
  for (i=0;i<5;i++) nrf_gpio_cfg_output(leds_list[i]);
  }
static void gpioOn(uint8_t pin) {
  nrf_gpio_pin_set(pin);
  }
static void gpioOff(uint8_t pin) {
  nrf_gpio_pin_clear(pin);
  }
static void gpioToggle(uint8_t pin) {
  nrf_gpio_pin_toggle(pin);
  }

void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context) {
  DS3231clock_t * t;
  switch (p_event->type) {
    case NRF_DRV_TWI_RX_DONE:
      t = (DS3231clock_t *)twi_buffer;
      clock.month =  10*((t->month>>4)&0x7)+(t->month&0xf); 
      clock.day =    10*(t->day>>4) + (t->day&0xf);
      clock.hour =   10*((t->hour>>4)&0x3)+(t->hour&0xf); 
      clock.minute = 10*(t->minute>>4)+(t->minute&0xf);
      break;
    case NRF_DRV_TWI_TX_DONE:
      nrf_drv_twi_rx(&m_twi_port,0xD0>>1,twi_buffer,sizeof(DS3231clock_t),false);
      break;
    default:
      break;
    }
  }
void DS3231_read_curtime() {
  twi_buffer[0] = 0;   // register address on DS3231 chip
  nrf_drv_twi_tx(&m_twi_port,0xD0>>1,twi_buffer,1,true);
  }

void twi_init() {
  // twi_uninit(); 
  const nrf_drv_twi_config_t twi_config = {
    .scl = 0x01, .sda = 0x00,
    .frequency = NRF_TWI_FREQ_400K,
    .interrupt_priority = APP_IRQ_PRIORITY_HIGH
    };
  nrf_drv_twi_init(&m_twi_port,&twi_config,twi_handler,NULL);
  nrf_drv_twi_enable(&m_twi_port);
  }
void twi_uninit() {
  nrf_drv_twi_disable(&m_twi_port);
  nrf_drv_twi_uninit(&m_twi_port);
  }

static void sleep_mode_enter(void) {
  sd_power_system_off(); // <--- note timer is not running :(
  }

static void timer_handler(void * p_context) {
  UNUSED_PARAMETER(p_context);
  // clock += 1;
  // gpioToggle(22);
  if (m_our_service.conn_handle != BLE_CONN_HANDLE_INVALID) {
    uint16_t len = 4;
    ble_gatts_hvx_params_t hvx_params;
    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.handle = m_our_service.char_handles[CLOCK_READ_INDX].value_handle;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = 0;
    hvx_params.p_len = &len;
    hvx_params.p_data = (uint8_t*)&clock;  
    sd_ble_gatts_hvx(m_our_service.conn_handle,&hvx_params);
    }   
  if (connected) return;
  // if (clock < 30) return;
  // sleep_mode_enter();
  }

static void timers_init(void) {
  APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
  app_timer_create(&timer_id, APP_TIMER_MODE_REPEATED, timer_handler);
  }
static void timers_start(void) {
  app_timer_start(timer_id,APP_TIMER_TICKS(1000,APP_TIMER_PRESCALER),NULL);
  }
static void gap_params_init(void) {
  ble_gap_conn_params_t   gap_conn_params;
  ble_gap_conn_sec_mode_t sec_mode;
  memset(&gap_conn_params, 0, sizeof(gap_conn_params));
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
  sd_ble_gap_device_name_set(&sec_mode,(const uint8_t *)"Signal Labs",strlen("Signal Labs"));
  sd_ble_gap_appearance_set(BLE_APPEARANCE_UNKNOWN);
  gap_conn_params.min_conn_interval = MSEC_TO_UNITS(100, UNIT_1_25_MS); // MIN_CONN_INTERVAL
  gap_conn_params.max_conn_interval = MSEC_TO_UNITS(200, UNIT_1_25_MS); // MAX_CONN_INTERVAL;
  gap_conn_params.slave_latency     = 0; // SLAVE_LATENCY;
  gap_conn_params.conn_sup_timeout  = MSEC_TO_UNITS(4000, UNIT_10_MS);  // CONN_SUP_TIMEOUT;
  sd_ble_gap_ppcp_set(&gap_conn_params);
  }

static void local_service() {
  ble_uuid_t     service_uuid;
  ble_uuid128_t  base_uuid = {BLE_UUID_OUR_BASE_UUID};
  uint8_t        i;
  memset(&our_attributes, 0, sizeof(our_attributes));

  service_uuid.uuid = BLE_UUID_OUR_SERVICE;
  sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
  sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                           &service_uuid,
                           &m_our_service.service_handle);
  for (i=0;i<NUMBER_OUR_ATTRIBUTES;i++) {
    our_attributes[i].uuid.uuid = our_attribute_uuid_list[i];
    sd_ble_uuid_vs_add(&base_uuid, &our_attributes[i].uuid.type);
    our_attributes[i].char_md.char_props.read = 1;
    our_attributes[i].char_md.char_props.write = 1;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&our_attributes[i].cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&our_attributes[i].cccd_md.write_perm);
    our_attributes[i].cccd_md.vloc = BLE_GATTS_VLOC_STACK;    
    our_attributes[i].char_md.p_cccd_md = &our_attributes[i].cccd_md;
    our_attributes[i].char_md.char_props.notify = 1;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&our_attributes[i].attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&our_attributes[i].attr_md.write_perm);
    our_attributes[i].attr_char_value.p_uuid = &our_attributes[i].uuid;
    our_attributes[i].attr_char_value.p_attr_md = &our_attributes[i].attr_md;
    our_attributes[i].attr_md.vloc = BLE_GATTS_VLOC_STACK;   
    our_attributes[i].attr_char_value.max_len = our_attribute_lengths[i];
    our_attributes[i].attr_char_value.init_len = our_attribute_lengths[i];
    our_attributes[i].attr_char_value.p_value = our_attribute_pointers[i]; // WAS (uint8_t*)&clock;
    sd_ble_gatts_characteristic_add(m_our_service.service_handle,
                &our_attributes[i].char_md,
                &our_attributes[i].attr_char_value,
                &m_our_service.char_handles[i]);
    }
  }

static void services_init(void) {
  ble_dis_init_t dis_init;
  memset(&dis_init, 0, sizeof(dis_init));
  ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);
  ble_dis_init(&dis_init);
  local_service();
  }

static void on_conn_params_evt(ble_conn_params_evt_t * p_evt) {
  if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
    sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
    }
  }

static void conn_params_error_handler(uint32_t nrf_error) {
  APP_ERROR_HANDLER(nrf_error);
  }

static void conn_params_init(void) {
  ble_conn_params_init_t cp_init;
  memset(&cp_init, 0, sizeof(cp_init));

  cp_init.p_conn_params                  = NULL;
  cp_init.first_conn_params_update_delay = APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER); 
  cp_init.next_conn_params_update_delay  = APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER);
  cp_init.max_conn_params_update_count   = 3; // MAX_CONN_PARAMS_UPDATE_COUNT;
  cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
  cp_init.disconnect_on_fail             = false;
  cp_init.evt_handler                    = on_conn_params_evt;
  cp_init.error_handler                  = conn_params_error_handler;

  ble_conn_params_init(&cp_init);
  }

static void on_adv_evt(ble_adv_evt_t ble_adv_evt) {
  switch (ble_adv_evt) {
    case BLE_ADV_EVT_FAST:
            break;
    case BLE_ADV_EVT_SLOW:
            break;
    case BLE_ADV_EVT_IDLE:
            // gpioToggle(19);
            sleep_mode_enter();
            break;
    default: break;
    }
  }

static void on_ble_gatt_write(ble_evt_t * p_ble_evt) {
  uint32_t data_buffer;
  ble_gatts_value_t rx_data;
  rx_data.len = sizeof(uint32_t);
  rx_data.offset = 0;
  rx_data.p_value = (uint8_t*)&data_buffer;
  if (p_ble_evt->evt.gatts_evt.params.write.handle == m_our_service.char_handles[CLOCK_WRITE_INDX].value_handle) {
    sd_ble_gatts_value_get(m_our_service.conn_handle,m_our_service.char_handles[CLOCK_WRITE_INDX].value_handle,&rx_data);
    // ############ SET CLOCK HERE FROM data_buffer
    // gpioToggle(21);
    }
  } 

static void on_ble_evt(ble_evt_t * p_ble_evt) {
  switch (p_ble_evt->header.evt_id) {
    case BLE_GAP_EVT_CONNECTED:
      // gpioOn(18);
      m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
      connected = true;
      break;

    case BLE_GAP_EVT_DISCONNECTED:
      // gpioOff(18);
      m_conn_handle = BLE_CONN_HANDLE_INVALID;
      connected = false;
      break;

    case BLE_GATTS_EVT_WRITE:
      on_ble_gatt_write(p_ble_evt);
      break;

    default:
      break;
    }
  }

static void ble_evt_dispatch(ble_evt_t * p_ble_evt) {
  dm_ble_evt_handler(p_ble_evt);
  ble_conn_params_on_ble_evt(p_ble_evt);
  on_ble_evt(p_ble_evt);
  ble_advertising_on_ble_evt(p_ble_evt);
  }

static void sys_evt_dispatch(uint32_t sys_evt) {
  pstorage_sys_event_handler(sys_evt);
  ble_advertising_on_sys_evt(sys_evt);
  }

static void softdevice_init() {   
  uint8_t ison = false;
  sd_softdevice_is_enabled(&ison);
  if (ison) return;
  SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_SYNTH_250_PPM, false);
  }

static void ble_stack_init(void) {
  ble_enable_params_t ble_enable_params;
  memset(&ble_enable_params, 0, sizeof(ble_enable_params));
  ble_enable_params.gatts_enable_params.service_changed = 1;
  sd_ble_enable(&ble_enable_params);
  softdevice_ble_evt_handler_set(ble_evt_dispatch);
  softdevice_sys_evt_handler_set(sys_evt_dispatch);
  }

static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,
                                           dm_event_t const  * p_event,
                                           ret_code_t        event_result) {
  return NRF_SUCCESS;
  }

static void device_manager_init(bool erase_bonds) {
  dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
  dm_application_param_t register_param;

  pstorage_init();
  dm_init(&init_param);

  memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));
  register_param.sec_param.bond         = 1;  // SEC_PARAM_BOND;
  register_param.sec_param.mitm         = 0;  // SEC_PARAM_MITM;
  register_param.sec_param.io_caps      = BLE_GAP_IO_CAPS_NONE; 
  register_param.sec_param.oob          = 0;  // SEC_PARAM_OOB;
  register_param.sec_param.min_key_size = 7;  // SEC_PARAM_MIN_KEY_SIZE;
  register_param.sec_param.max_key_size = 16; // SEC_PARAM_MAX_KEY_SIZE;
  register_param.evt_handler            = device_manager_evt_handler;
  register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;
  dm_register(&m_app_handle, &register_param);
  }

static void advertising_init(void) {
  ble_advdata_t advdata;
  ble_adv_modes_config_t options = {0};
  ble_advdata_t srdata;
  memset(&advdata, 0, sizeof(advdata));
  memset(&srdata, 0, sizeof(srdata));
  advdata.name_type               = BLE_ADVDATA_FULL_NAME;
  advdata.include_appearance      = true;
  advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
  advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
  advdata.uuids_complete.p_uuids  = m_adv_uuids;
  options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
  options.ble_adv_fast_interval = 300; // APP_ADV_INTERVAL; (units are 6.24us, so 0x4000 is 10.25s)
  options.ble_adv_fast_timeout  = 30;  // APP_ADV_TIMEOUT_IN_SECONDS;
  srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
  srdata.uuids_complete.p_uuids = m_adv_uuids;
  ble_advertising_init(&advdata, &srdata, &options, on_adv_evt, NULL);
  }

static void power_manage(void) {
  sd_app_evt_wait();
  }

static void init_all() {
  softdevice_init();
  gpioInit();
  gpioteInit();
  twi_init();
  timers_init();
  ble_stack_init();
  device_manager_init(true);
  gap_params_init();
  advertising_init();
  services_init();
  conn_params_init();
  timers_start();
  ble_advertising_start(BLE_ADV_MODE_FAST);
  }

int main(void) {
  init_all();
  for (;;) { power_manage(); }
  }

static void gpioteHandler(nrf_drv_gpiote_pin_t p, nrf_gpiote_polarity_t d) {
  uint8_t ison = false;
  if (p != 16) return;
  sd_softdevice_is_enabled(&ison);
  if (ison) {
    advertising_init();
    ble_advertising_start(BLE_ADV_MODE_FAST);
    }
  else init_all();
  gpioToggle(20);
  DS3231_read_curtime();
  }
