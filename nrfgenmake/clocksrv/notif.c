#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdm.h"
#include "app_error.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "sensorsim.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "bsp_btn_ble.h"
#include "nrf_sdh_soc.h"
#include "bsp_btn_ble.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "fds.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_lesc.h"
#include "nrf_ble_qwr.h"
#include "ble_conn_state.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE
#define SCHED_QUEUE_SIZE 10

#define APP_ADV_INTERVAL 300 
#define APP_ADV_DURATION 18000 
#define APP_BLE_CONN_CFG_TAG 1
#define APP_BLE_OBSERVER_PRIO 3 

#define MIN_CONN_INTERVAL MSEC_TO_UNITS(400, UNIT_1_25_MS)
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(650, UNIT_1_25_MS)
#define SLAVE_LATENCY 0 
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(4000, UNIT_10_MS)
#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(5000)
#define NEXT_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(30000)
#define MAX_CONN_PARAMS_UPDATE_COUNT 3
#define LESC_DEBUG_MODE 0

#define SEC_PARAM_BOND 1
#define SEC_PARAM_MITM 0
#define SEC_PARAM_LESC 1
#define SEC_PARAM_KEYPRESS 0
#define SEC_PARAM_IO_CAPABILITIES BLE_GAP_IO_CAPS_NONE
#define SEC_PARAM_OOB 0 
#define SEC_PARAM_MIN_KEY_SIZE 7
#define SEC_PARAM_MAX_KEY_SIZE 16

#define DEAD_BEEF 0xDEADBEEF 

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

/*
#define OPCODE_LENGTH 1                                                             
#define HANDLE_LENGTH 2
#define MAX_HRM_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH)
#define INITIAL_VALUE_HRM 0
#define HRM_FLAG_MASK_HR_VALUE_16BIT (0x01 << 0)
#define HRM_FLAG_MASK_SENSOR_CONTACT_DETECTED (0x01 << 1) 
#define HRM_FLAG_MASK_SENSOR_CONTACT_SUPPORTED (0x01 << 2) 
#define HRM_FLAG_MASK_EXPENDED_ENERGY_INCLUDED (0x01 << 3) 
#define HRM_FLAG_MASK_RR_INTERVAL_INCLUDED (0x01 << 4)
*/


// basic GATT connection/handler struct for this module
typedef enum {
    BLE_NOTIF_EVT_NOTIFICATION_ENABLED,  
    BLE_NOTIF_EVT_NOTIFICATION_DISABLED 
    } ble_notif_evt_type_t;
typedef enum {
    CHAR_MODE_READ,
    CHAR_MODE_WRITE
    } app_char_type_t;	   
typedef struct {
    ble_notif_evt_type_t evt_type;  
    } ble_notif_evt_t;
typedef struct notif_s notif_t;
typedef void (*notif_evt_handler_t) (notif_t * p_notif, ble_notif_evt_t * p_evt);
//typedef void (*nrf_ble_amtc_evt_handler_t) (struct nrf_ble_amtc_t * p_ctx, nrf_ble_amtc_evt_t    * p_evt);

struct notif_s {
   notif_evt_handler_t evt_handler;
   ble_gatts_char_handles_t clock_handle; 
   ble_add_char_params_t clock_params;
   ble_gatts_char_handles_t setclock_handle; 
   ble_add_char_params_t setclock_params;
   ble_uuid_t service_uuid;
   ble_uuid128_t base_uuid128;
   uint16_t conn_handle;
   uint16_t service_handle;
   uint32_t clock;
   uint32_t setclock;
   uint16_t max_notif_len;
   uint8_t uuid_type;
   bool notifyset;
   uint16_t debug1;
   };
static notif_t m_notif;

// DEVICE_INFORMATION_SERVICE from ble_srv_common.h
//static ble_uuid_t m_adv_uuids[2] = {{BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE},
                                    // {BLE_UUID_OUR_SERVICE, BLE_UUID_TYPE_VENDOR_BEGIN}};
static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}};
const uint16_t our_attribute_uuid_list[NUMBER_OUR_ATTRIBUTES] = {CLOCK_READ_UUID,CLOCK_WRITE_UUID};
const uint8_t our_attribute_lengths[NUMBER_OUR_ATTRIBUTES] = 
  {sizeof(m_notif.clock),sizeof(m_notif.setclock)};
uint8_t* our_attribute_pointers[NUMBER_OUR_ATTRIBUTES] = 
  {(uint8_t*)&m_notif.clock,(uint8_t*)&m_notif.setclock};
typedef struct {
  ble_uuid_t uuid;
  ble_gatts_char_md_t char_md;
  ble_gatts_attr_md_t cccd_md;
  ble_gatts_attr_md_t attr_md;
  ble_gatts_attr_t  attr_char_value;
  } charvar_t; 
// static charvar_t our_attributes[NUMBER_OUR_ATTRIBUTES];
// static dm_application_instance_t m_app_handle;


//   uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;   
//   ble_uuid128_t m_base_uuid128 = {BLE_UUID_OUR_BASE_UUID};
// NRF_SDH_BLE_OBSERVER(m_notif,APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
// see https://devzone.nordicsemi.com/f/nordic-q-a/24292/sdk-14---how-to-set-up-a-ble-observer-for-custom-service

bool connected = false;

static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];

static ble_gap_adv_data_t m_adv_data = {
    .adv_data = {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
        },
    .scan_rsp_data = {
        .p_data = NULL,
        .len    = 0
        }
    };

APP_TIMER_DEF(timer_id);
NRF_BLE_GATT_DEF(m_gatt);
NRF_BLE_QWR_DEF(m_qwr); 
BLE_ADVERTISING_DEF(m_advertising);  

static void sleep_mode_enter(void) {
  sd_power_system_off(); // <--- note timer is not running :(
  }


void blesetclock(void) {
  ret_code_t err_code;
  ble_gatts_value_t gatts_value;
  memset(&gatts_value,0,sizeof(gatts_value));
  gatts_value.len = sizeof(m_notif.clock);
  gatts_value.offset = 0;
  gatts_value.p_value = (uint8_t*)&m_notif.clock;
  err_code = sd_ble_gatts_value_set(
	m_notif.conn_handle,
	m_notif.clock_handle.value_handle,
	&gatts_value);
  return;
  APP_ERROR_CHECK(err_code);
  }

static void updateclock(void) {
  ret_code_t err_code;
  uint16_t len = 4;
  ble_gatts_hvx_params_t hvx_params;
  blesetclock();
  if (m_notif.conn_handle == BLE_CONN_HANDLE_INVALID) return;
  if (!m_notif.notifyset) return;
  memset(&hvx_params, 0, sizeof(hvx_params));
  hvx_params.handle = m_notif.clock_handle.value_handle;
  hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
  hvx_params.offset = 0;
  hvx_params.p_len = &len;
  hvx_params.p_data = (uint8_t*)&m_notif.clock;
  err_code = sd_ble_gatts_hvx(m_notif.conn_handle,&hvx_params);
  if (err_code != NRF_SUCCESS)
	  NRF_LOG_INFO("update clock error %d",err_code);
  }

void debug(void * parameter, uint16_t size) {
  NRF_LOG_INFO("%d versus ( %d %d %d %d %d %d )",m_notif.debug1,
      m_notif.clock_handle.value_handle,
      m_notif.clock_handle.user_desc_handle,
      m_notif.clock_handle.cccd_handle,
      m_notif.clock_handle.sccd_handle);
  NRF_LOG_INFO("extra %d %d",m_notif.conn_handle,m_notif.service_handle);
  NRF_LOG_FLUSH();
  }

static void on_ble_gatt_write(const ble_evt_t * p_ble_evt) {
  uint32_t data_buffer;
  ble_gatts_value_t rx_data;
  ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
  bsp_board_led_on(2);
  m_notif.debug1 = p_evt_write->handle;
  app_sched_event_put(NULL,1,&debug);
  if (    p_evt_write->handle == m_notif.clock_handle.cccd_handle
       && p_evt_write->len == 2 ) {
     if (ble_srv_is_notification_enabled(p_evt_write->data)) { 
        bsp_board_led_on(3);
	m_notif.notifyset = true;
        // not sure what to do here; maybe set some flag?
	return;
        }	  
     return;   /// different write op???
     }
  rx_data.len = sizeof(uint32_t);
  rx_data.offset = 0;
  rx_data.p_value = (uint8_t*)&data_buffer;
  if (p_ble_evt->evt.gatts_evt.params.write.handle == 
	m_notif.setclock_handle.value_handle) {
           sd_ble_gatts_value_get(
	      m_notif.conn_handle,
	      m_notif.setclock_handle.value_handle,
	      &rx_data);
	NRF_LOG_INFO("gatt write");   
        }
  }

/*
void notification_send() {
  ret_code_t err_code;
  uint16_t payload_len = sizeof(m_notif.clock);
  ble_gatts_hvx_params_t const hvx_param = {
     .type   = BLE_GATT_HVX_NOTIFICATION,
     .handle = m_notif.clock_handle.value_handle,
     .p_data = (uint8_t*)&m_notif.clock,
     .p_len  = &payload_len
     };
  err_code = sd_ble_gatts_hvx(m_notif.conn_handle,&hvx_param);
  APP_ERROR_CHECK(err_code);
  }
void schednotify(void * parameter, uint16_t size) {
  if (m_notif.notifyset) notification_send();
  }
*/

static void timer_handler(void * p_context) {
  UNUSED_PARAMETER(p_context);
  m_notif.clock++;
  updateclock();
  // app_sched_event_put(NULL,1,&schednotify);
  }

void bsp_event_handler(bsp_event_t event);
static void buttons_leds_init(bool * p_erase_bonds) {
    ret_code_t err_code;
    bsp_event_t startup_event;
    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);
    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);
    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
    }

static void log_init(void) {
  ret_code_t err_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(err_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  }

static void timers_init(void) {
  ret_code_t err_code;
  err_code = app_timer_init();
  APP_ERROR_CHECK(err_code);

  err_code = app_timer_create(&timer_id, 
		  APP_TIMER_MODE_REPEATED, timer_handler);
  APP_ERROR_CHECK(err_code);
  }
static void timers_start(void) {
  app_timer_start(timer_id,APP_TIMER_TICKS(1000),NULL);
  }
static void gap_params_init(void) {
  ble_gap_conn_params_t   gap_conn_params;
  ble_gap_conn_sec_mode_t sec_mode;
  memset(&gap_conn_params, 0, sizeof(gap_conn_params));
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
  sd_ble_gap_device_name_set(&sec_mode,(const uint8_t *)
		  "Signal Labs",strlen("Signal Labs"));
  sd_ble_gap_appearance_set(BLE_APPEARANCE_UNKNOWN);
  gap_conn_params.min_conn_interval = MSEC_TO_UNITS(100, UNIT_1_25_MS); 
  gap_conn_params.max_conn_interval = MSEC_TO_UNITS(200, UNIT_1_25_MS);
  gap_conn_params.slave_latency     = 0; 
  gap_conn_params.conn_sup_timeout  = MSEC_TO_UNITS(4000, UNIT_10_MS); 
  sd_ble_gap_ppcp_set(&gap_conn_params);
  }

// handle a gatt event
void ble_notif_on_gatt_evt(notif_t * p_notif, nrf_ble_gatt_evt_t const * p_gatt_evt) {
  if ((p_notif->conn_handle == p_gatt_evt->conn_handle)
        && (p_gatt_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)) {
      p_notif->max_notif_len = p_gatt_evt->params.att_mtu_effective;
      }
  }
static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, 
		nrf_ble_gatt_evt_t const * p_evt) {
  switch (p_evt->evt_id) {
	  case NRF_BLE_GATT_EVT_ATT_MTU_UPDATED: {
               NRF_LOG_INFO("GATT ATT MTU on connection 0x%x changed to %d.",
                   p_evt->conn_handle,
                   p_evt->params.att_mtu_effective);
	       } break;

          case NRF_BLE_GATT_EVT_DATA_LENGTH_UPDATED: {
               NRF_LOG_INFO("Data length updated to %u bytes.", 
		   p_evt->params.data_length);
               } break;
    }
  }

static void gatt_init(void) {
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt,gatt_evt_handler);
    APP_ERROR_CHECK(err_code);
    }

static void nrf_qwr_error_handler(uint32_t nrf_error) {
    APP_ERROR_HANDLER(nrf_error);
    }

/*
// add a particular characteristic
static void charac_add(
		uint16_t char_uuid_number,
		uint16_t char_len,
		uint8_t * p_init_value,
		ble_gatts_char_handles_t * p_handles,
		ble_add_char_params_t * p_params,
                app_char_type_t mode) {
  ret_code_t err_code;
  memset(p_params, 0, sizeof(ble_add_char_params_t));
  p_params->uuid = char_uuid_number;
  p_params->uuid_type = m_notif.service_uuid.type;
  p_params->max_len = char_len; // or NRF_SDH_BLE_GATT_MAX_MTU_SIZE;
  p_params->p_init_value = p_init_value;
  NRF_LOG_INFO("setting data pointer %p -> %x %x %x %x",p_init_value,
		  *p_init_value,*(p_init_value+1),*(p_init_value+2),*(p_init_value+3));
  switch (mode) {
      case CHAR_MODE_READ:
        p_params->char_props.read = 1;
        p_params->read_access = SEC_OPEN;
        break;
      case CHAR_MODE_WRITE:
	p_params->char_props.write = 1;
        p_params->write_access = SEC_OPEN;
        break;
      default:
        // p_params->char_props.notify = 1;
        // p_params->cccd_write_access = SEC_OPEN;
        break;
      }
  // see ble_srv_common.c for the following
  err_code = characteristic_add(m_notif.service_handle,
		  p_params,
		  p_handles);
  APP_ERROR_CHECK(err_code);
  }
*/

/*
static void local_service_add() {
  ret_code_t err_code;
  uint8_t local_uuid;
  ble_uuid128_t base_uuid128 = {BLE_UUID_OUR_BASE_UUID};
  memset(&m_notif,0,sizeof(m_notif));
  m_notif.conn_handle = BLE_CONN_HANDLE_INVALID;   
  err_code = sd_ble_uuid_vs_add(&base_uuid128, &local_uuid);
  APP_ERROR_CHECK(err_code);  // this assigns into the type field (an index)

  m_notif.service_uuid.type = local_uuid;
  m_notif.service_uuid.type = 2;
  m_notif.service_uuid.uuid = BLE_UUID_OUR_SERVICE;

  m_notif.conn_handle = BLE_CONN_HANDLE_INVALID;
  err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, 
		 &m_notif.service_uuid, 
		 &m_notif.service_handle);
  // presumably that assigned into the service_handle
  APP_ERROR_CHECK(err_code);
  }
*/

/*
static void local_service_add() {
  ret_code_t err_code;
  ble_uuid_t ble_uuid;
  ble_add_char_params_t add_char_params;

  ble_gatts_char_md_t char_md;
  ble_gatts_attr_md_t cccd_md;
  ble_gatts_attr_t attr_char_value;
  ble_gatts_attr_md_t attr_md;
  ble_uuid_t char_uuid;

  memset(&m_notif,0,sizeof(m_notif));
  memset(&cccd_md, 0, sizeof(cccd_md));
  memset(&char_md, 0, sizeof(char_md));
  memset(&attr_md, 0, sizeof(attr_md));

  cccd_md.vloc = BLE_GATTS_VLOC_STACK;
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

  char_md.char_props.read   = 1;
  char_md.char_props.notify = 1;
  char_md.p_char_user_desc  = NULL;
  char_md.p_char_pf         = NULL;
  char_md.p_user_desc_md    = NULL;
  char_md.p_cccd_md         = &cccd_md;
  char_md.p_sccd_md         = NULL;

  m_notif.conn_handle = BLE_CONN_HANDLE_INVALID;

  char_uuid.type = uuid_type;
  char_uuid.uuid = LOCAL_CHAR_UUID;

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);

  attr_md.vloc    = BLE_GATTS_VLOC_STACK;
  attr_md.rd_auth = 0;
  attr_md.wr_auth = 0;
  attr_md.vlen    = 0;
  memset(&attr_char_value, 0, sizeof(attr_char_value));

  attr_char_value.p_uuid    = &char_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len  = APP_CFG_CHAR_LEN;
  attr_char_value.init_offs = 0;
  attr_char_value.max_len   = APP_CFG_CHAR_LEN;
  attr_char_value.p_value   = m_char_value;

  err_code = sd_ble_gatts_characteristic_add(m_service_handle,
                    &char_md,
                    &attr_char_value,
                    &m_char_handles);
  APP_ERROR_CHECK(err_code);

  // Add service
  BLE_UUID_BLE_ASSIGN(ble_uuid,BLE_UUID_OUR_SERVICE);
  err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                    &ble_uuid,
                    &m_notif.service_handle);
  APP_ERROR_CHECK(err_code);

  // add clock characteristic - read only 
  memset(&add_char_params, 0, sizeof(add_char_params));
  add_char_params.uuid = CLOCK_READ_UUID;
  add_char_params.max_len = sizeof(m_notif.clock);
  add_char_params.init_len = sizeof(m_notif.clock);
  add_char_params.p_init_value = (uint8_t*)&m_notif.clock;
  add_char_params.is_var_len = false;
  add_char_params.char_props.read = 1;  // there are many others, see ble_gatt.h
  add_char_params.char_props.write = 0; 
  add_char_params.char_props.notify = 0;  
  add_char_params.read_access = SEC_OPEN;
  add_char_params.cccd_write_access = SEC_OPEN;
  err_code = characteristic_add(m_notif.service_handle, 
		  &add_char_params, 
		  &(m_notif.notif_handles));
  APP_ERROR_CHECK(err_code);
  }	
*/

static void local_service_add() {
  ret_code_t err_code;
  ble_uuid_t ble_uuid;
  ble_uuid128_t base_uuid = {BLE_UUID_OUR_BASE_UUID};

  memset(&m_notif,0,sizeof(m_notif));
  m_notif.conn_handle = BLE_CONN_HANDLE_INVALID;   

  err_code = sd_ble_uuid_vs_add(&base_uuid, &m_notif.uuid_type);
  APP_ERROR_CHECK(err_code);
  ble_uuid.type = m_notif.uuid_type;
  ble_uuid.uuid = BLE_UUID_OUR_SERVICE;

  // this adds a primary service and sets a service handle 
  err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &m_notif.service_handle);
  APP_ERROR_CHECK(err_code);
  }

static void local_clock_add() {
  // add the clock characteristic
  ble_gatts_char_md_t char_md;
  ble_gatts_attr_md_t cccd_md;
  ble_gatts_attr_t    attr_char_value;
  ble_uuid_t          ble_uuid;
  ble_gatts_attr_md_t attr_md;

  memset(&char_md, 0, sizeof(char_md));
  memset(&cccd_md, 0, sizeof(cccd_md));
  memset(&attr_char_value, 0, sizeof(attr_char_value));

  // cccd md
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
  cccd_md.vloc = BLE_GATTS_VLOC_STACK;

  // char md
  char_md.char_props.read   = 1;
  char_md.char_props.notify = 1;
  // char_md.p_char_user_desc  = NULL;
  char_md.p_char_user_desc  = (uint8_t*) "Read Clock";
  char_md.char_user_desc_max_size = 10;
  char_md.char_user_desc_size = 10;
  char_md.p_char_pf         = NULL;
  char_md.p_user_desc_md    = NULL;
  char_md.p_cccd_md         = &cccd_md;  // how CCCD is connected
  char_md.p_sccd_md         = NULL;


  // attr md
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
  attr_md.vloc       = BLE_GATTS_VLOC_STACK;
  attr_md.rd_auth    = 0;
  attr_md.wr_auth    = 0;
  attr_md.vlen       = 0;

  attr_char_value.p_uuid       = &ble_uuid;  // the UUID and type
  attr_char_value.p_attr_md    = &attr_md;   // how attr_md is connected
  attr_char_value.init_len     = sizeof(m_notif.clock);
  attr_char_value.init_offs    = 0;
  attr_char_value.max_len      = sizeof(m_notif.clock);
  attr_char_value.p_value      = (uint8_t*)&m_notif.clock;

  ble_uuid.type = m_notif.uuid_type;
  ble_uuid.uuid = CLOCK_READ_UUID;

  sd_ble_gatts_characteristic_add(
              m_notif.service_handle, 
	      &char_md,          // which connects to cccd_md
              &attr_char_value,  // which connects to attr_md 
              &m_notif.clock_handle);
  }

static void local_setclock_add() {
  // add the clock characteristic
  ble_gatts_char_md_t char_md;
  ble_gatts_attr_t    attr_char_value;
  ble_uuid_t          ble_uuid;
  ble_gatts_attr_md_t attr_md;

  memset(&char_md, 0, sizeof(char_md));
  memset(&attr_char_value, 0, sizeof(attr_char_value));

  // char md
  char_md.char_props.read   = 1;
  char_md.char_props.write  = 1;
  // char_md.p_char_user_desc  = NULL;
  char_md.p_char_user_desc  = (uint8_t*) "Set Clock";
  char_md.char_user_desc_max_size = 9;
  char_md.char_user_desc_size = 9;
  char_md.p_char_pf         = NULL;
  char_md.p_user_desc_md    = NULL;
  char_md.p_cccd_md         = NULL; 
  char_md.p_sccd_md         = NULL;

  // attr md
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
  attr_md.vloc       = BLE_GATTS_VLOC_STACK;
  attr_md.rd_auth    = 0;
  attr_md.wr_auth    = 0;
  attr_md.vlen       = 0;

  attr_char_value.p_uuid       = &ble_uuid;  // the UUID and type
  attr_char_value.p_attr_md    = &attr_md;   // how attr_md is connected
  attr_char_value.init_len     = sizeof(m_notif.setclock);
  attr_char_value.init_offs    = 0;
  attr_char_value.max_len      = sizeof(m_notif.setclock);
  attr_char_value.p_value      = (uint8_t*)&m_notif.setclock;

  ble_uuid.type = m_notif.uuid_type;
  ble_uuid.uuid = CLOCK_WRITE_UUID;

  sd_ble_gatts_characteristic_add(
              m_notif.service_handle, 
	      &char_md,         
              &attr_char_value,  // which connects to attr_md 
              &m_notif.setclock_handle);
  }

/*
static void local_service() {
  ble_uuid_t     service_uuid;
  ble_uuid128_t  base_uuid = {BLE_UUID_OUR_BASE_UUID};
  uint8_t        i;
  memset(&our_attributes, 0, sizeof(our_attributes));

  service_uuid.uuid = BLE_UUID_OUR_SERVICE;
  sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
  sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                           &service_uuid,
                           &m_.service_handle);
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
    our_attributes[i].attr_char_value.p_value = our_attribute_pointers[i]; 
    sd_ble_gatts_characteristic_add(m_our_service.service_handle,
                &our_attributes[i].char_md,
                &our_attributes[i].attr_char_value,
                &m_our_service.char_handles[i]);
    }
  }
*/

static void local_init(void) {
  local_service_add();
  local_clock_add();
  local_setclock_add();
  /*
  NRF_LOG_INFO("Service handle created.");
  charac_add(CLOCK_READ_UUID,
             sizeof(m_notif.clock),
	     (uint8_t *)&m_notif.clock,
	     &m_notif.clock_handle,
	     &m_notif.clock_params,
             CHAR_MODE_READ);
  charac_add(CLOCK_WRITE_UUID,
             sizeof(m_notif.setclock),
	     (uint8_t *)&m_notif.setclock,
	     &m_notif.setclock_handle,
	     &m_notif.setclock_params,
             CHAR_MODE_WRITE);
  NRF_LOG_INFO("Characteristic handles created.");
  */
  }

static void services_init(void) {
  ret_code_t err_code;
  ble_dis_init_t dis_init;
  memset(&dis_init, 0, sizeof(dis_init));
  ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);
  dis_init.dis_char_rd_sec = SEC_OPEN;
  nrf_ble_qwr_init_t qwr_init = {0};
  qwr_init.error_handler = nrf_qwr_error_handler;
  err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
  APP_ERROR_CHECK(err_code);

  err_code = ble_dis_init(&dis_init);
  APP_ERROR_CHECK(err_code);

  local_init();
  }

static void delete_bonds(void) {
  ret_code_t err_code;
  NRF_LOG_INFO("Erase bonds!");
  err_code = pm_peers_delete();
  APP_ERROR_CHECK(err_code);
  }

/*
void advertising_start(bool erase_bonds) {
  if (erase_bonds == true) {
        delete_bonds();
        }
    else {
        ret_code_t err_code;
        err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
        }
  }
*/

static void advertising_start(bool erase_bonds) {
  if (erase_bonds == true) { delete_bonds(); }
  else {
    ret_code_t err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);
    }
  }

static void pm_evt_handler(pm_evt_t const * p_evt) {
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id) {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(false);
            break;
        default:
            break;
        }
    }
static void peer_manager_init(void) {
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;
    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));
    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;
    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);
    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
    }


static void on_conn_params_evt(ble_conn_params_evt_t * p_evt) {
  if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
    sd_ble_gap_disconnect(m_notif.conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
    }
  }

static void conn_params_error_handler(uint32_t nrf_error) {
  APP_ERROR_HANDLER(nrf_error);
  }

/*
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
*/

static void conn_params_init(void) {
  ret_code_t             err_code;
  ble_conn_params_init_t cp_init;

  memset(&cp_init, 0, sizeof(cp_init));

  cp_init.p_conn_params                  = NULL;
  cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
  cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
  cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
  // cp_init.start_on_notify_cccd_handle    = m_notif.notif_handles.cccd_handle;
  cp_init.disconnect_on_fail             = false;
  cp_init.evt_handler                    = on_conn_params_evt;
  cp_init.error_handler                  = conn_params_error_handler;
  err_code = ble_conn_params_init(&cp_init);
  APP_ERROR_CHECK(err_code);
  }

/*
static void on_adv_evt(ble_adv_evt_t ble_adv_evt) {
  switch (ble_adv_evt) {
    case BLE_ADV_EVT_FAST:
            break;
    case BLE_ADV_EVT_SLOW:
            break;
    case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
    default: break;
    }
  }
*/

/*
static void on_ble_gatt_write(ble_evt_t * p_ble_evt) {
  uint32_t data_buffer;
  ble_gatts_value_t rx_data;
  rx_data.len = sizeof(uint32_t);
  rx_data.offset = 0;
  rx_data.p_value = (uint8_t*)&data_buffer;
  if (p_ble_evt->evt.gatts_evt.params.write.handle == m_our_service.char_handles[CLOCK_WRITE_INDX].value_handle) {
    sd_ble_gatts_value_get(m_our_service.conn_handle,m_our_service.char_handles[CLOCK_WRITE_INDX].value_handle,&rx_data);
    }
  } 
*/

/*
static void on_ble_evt(ble_evt_t * p_ble_evt) {
  switch (p_ble_evt->header.evt_id) {
    case BLE_GAP_EVT_CONNECTED:
      m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
      connected = true;
      break;

    case BLE_GAP_EVT_DISCONNECTED:
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
*/

/*
static void char_notify(void) {
  ret_code_t err_code;
  ble_gatts_hvx_params_t hvx_params;
  uint16_t len = 4;  
  if (m_our_service.conn_handle != BLE_CONN_HANDLE_INVALID) {
    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.handle = 
	    m_our_service.char_handles[CLOCK_READ_INDX].value_handle;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = 0;
    hvx_params.p_len = &len;
    hvx_params.p_data = (uint8_t*)&m_notif.clock;  
    err_code = sd_ble_gatts_hvx(m_our_service.conn_handle,&hvx_params);
    APP_ERROR_CHECK(err_code);
    }   
  }

static void on_read(ble_evt_t * p_ble_evt) {
    ble_gatts_evt_read_t const * p_evt_read = 
	   &p_ble_evt->evt.gatts_evt.params.read; 
    if ((p_evt_read->handle == m_char_handles.cccd_handle)
        & (p_evt_read->len == 4)) {
        char_notify();
	}
    }
*/

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context) {
    ret_code_t err_code;
    NRF_LOG_INFO("got ble event %d",p_ble_evt->header.evt_id);

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_notif.conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, 
			    m_notif.conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected, reason %d.",
                          p_ble_evt->evt.gap_evt.params.disconnected.reason);
            m_notif.conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
            } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            NRF_LOG_DEBUG("BLE_GAP_EVT_SEC_PARAMS_REQUEST");
            break;

        case BLE_GAP_EVT_AUTH_KEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_AUTH_KEY_REQUEST");
            break;

        case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_LESC_DHKEY_REQUEST");
            break;

        case BLE_GAP_EVT_AUTH_STATUS:
             NRF_LOG_INFO("BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
                          p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
                          p_ble_evt->evt.gap_evt.params.auth_status.bonded,
                          p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
            break;

	case BLE_GATTS_EVT_WRITE:
            on_ble_gatt_write(p_ble_evt);
	    break;    

        default:
            // No implementation needed.
            break;
	}
   }

static void softdevice_init() {   
  ret_code_t err_code;
  err_code = nrf_sdh_enable_request();
  APP_ERROR_CHECK(err_code);
  }

/*
static void ble_stack_init(void) {
  ret_code_t err_code;
  // Configure the BLE stack using the default settings.
  // Fetch the start address of the application RAM.
  uint32_t ram_start = 0;
  err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
  APP_ERROR_CHECK(err_code);
  // Enable BLE stack.
  err_code = nrf_sdh_ble_enable(&ram_start);
  APP_ERROR_CHECK(err_code);
  // Register a handler for BLE events.
  NRF_SDH_BLE_OBSERVER(m_notif, 
		  APP_BLE_OBSERVER_PRIO, 
		  ble_evt_dispatch, 
		  NULL);
  }
*/
   
static void ble_stack_init(void) {
  ret_code_t err_code;

    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_notif,APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
    }

/*
static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,
                                           dm_event_t const  * p_event,
                                           ret_code_t        event_result) {
  return NRF_SUCCESS;
  }

static void device_manager_init(bool erase_bonds) {
  dm_init_param_t init_param = {.clear_persistent_data = erase_bonds};
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
*/

static void advertising_data_set(void) {
    ret_code_t ret;

    ble_gap_adv_params_t const adv_params = {
       .properties    = {
          .type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED,
          },
       .p_peer_addr   = NULL,
       .filter_policy = BLE_GAP_ADV_FP_ANY,
       .interval      = APP_ADV_INTERVAL,
       .duration      = 0,
       .primary_phy   = BLE_GAP_PHY_1MBPS, // Must be changed to connect in long range. (BLE_GAP_PHY_CODED)
       .secondary_phy = BLE_GAP_PHY_1MBPS,
       };

    ble_advdata_t const adv_data = {
       .name_type          = BLE_ADVDATA_FULL_NAME,
       .flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE,
       .include_appearance = true, 
       .uuids_complete     = {1,m_adv_uuids}
       };

    ret = ble_advdata_encode(&adv_data, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(ret);
    ret = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);
    APP_ERROR_CHECK(ret);
    }

/*
static void advertising_init(void) {
    ret_code_t             err_code;
    ble_advertising_init_t init;
    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    // notice how this init call connects m_advertising declared top
    // with the ble_advertising_init structure just filled in above
    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
    }
*/

void bsp_event_handler(bsp_event_t event) {
    ret_code_t err_code;
    switch (event) {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;
        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_notif.conn_handle,
                          BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE) {
                APP_ERROR_CHECK(err_code);
                }
            break;
        case BSP_EVENT_WHITELIST_OFF:
            if (m_notif.conn_handle == BLE_CONN_HANDLE_INVALID) {
                err_code = ble_advertising_restart_without_whitelist(
				&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE) {
                    APP_ERROR_CHECK(err_code);
                    }
                }
	case BSP_EVENT_KEY_2:
            NRF_LOG_INFO("Key_2 Pressed");
            //    app_sched_event_put(NULL,1,&motetx);
            break;
        default:
            break;
        }
    }

static void power_management_init(void) {
  ret_code_t err_code;
  err_code = nrf_pwr_mgmt_init();
  APP_ERROR_CHECK(err_code);
  }


static void power_manage(void) {
  // sd_app_evt_wait();
  ret_code_t err_code = nrf_ble_lesc_request_handler();
  APP_ERROR_CHECK(err_code);
  if (NRF_LOG_PROCESS() == false) nrf_pwr_mgmt_run();
  }

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name) {
  app_error_handler(DEAD_BEEF, line_num, p_file_name);
  }

static void init_all() {
  bool erase_bonds;
  log_init();
  timers_init();
  buttons_leds_init(&erase_bonds);
  APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
  power_management_init();
  softdevice_init();
  ble_stack_init();
  NRF_LOG_INFO("stack initialized");
  // device_manager_init(true);
  gap_params_init();
  gatt_init();
  memset(&m_notif, 0, sizeof(m_notif));
  services_init();
  advertising_data_set();
  // device_manager_init(true);
  conn_params_init();
  peer_manager_init();
  timers_start();
  advertising_start(true);
  NRF_LOG_INFO("advertising launched");
  }

int main(void) {
  init_all();
  for (;;) { 
    power_manage(); 
    app_sched_execute();
    }
  }
