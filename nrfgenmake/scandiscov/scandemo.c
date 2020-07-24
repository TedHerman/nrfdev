/*
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "assert.h"
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "app_error.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "ble_db_discovery.h"
#include "ble_dis_c.h"
#include "ble_dis.h"
#include "fds.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_scan.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
*/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "assert.h"
#include "nordic_common.h"
#include "app_timer.h"
#include "app_scheduler.h"
#include "app_error.h"
#include "app_util.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "ble_dis.h"
#include "ble_db_discovery.h"
#include "ble_dis_c.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "fds.h"
#include "nrf_fstorage.h"
#include "ble_conn_state.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_ble_scan.h"
#include "nrf_ble_gatt.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_fstorage.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


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

#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE
#define SCHED_QUEUE_SIZE 10

// for bonding, copied from app_rscs
#define SEC_PARAM_BOND              1  /**< Perform bonding. */
#define SEC_PARAM_MITM              0  
#define SEC_PARAM_LESC              0 
#define SEC_PARAM_KEYPRESS          0 
#define SEC_PARAM_IO_CAPABILITIES   BLE_GAP_IO_CAPS_NONE 
#define SEC_PARAM_OOB               0    
#define SEC_PARAM_MIN_KEY_SIZE      7                                  
#define SEC_PARAM_MAX_KEY_SIZE      16                                 

#define APP_BLE_CONN_CFG_TAG    1                                       
#define APP_BLE_OBSERVER_PRIO   3
#define APP_SOC_OBSERVER_PRIO   1
#define SCAN_INTERVAL 180 // scan interval in units of 0.625 millisecond
#define SCAN_WINDOW 90    // scan window in units of 0.625 millisecond
#define SCAN_TIMEOUT 0    // 0 means continuous 
#define DATA_LENGTH_MAX 244   // empirically determined max char size
#define MEMCHUNKS_POOL 720  // total memory is 720*244 
#define BALANCE_UUID_PRIMARY 0x8b09

#define CONN_INTERVAL_DEFAULT (uint16_t)(MSEC_TO_UNITS(7.5, UNIT_1_25_MS)) // at central side
#define CONN_INTERVAL_MIN (uint16_t)(MSEC_TO_UNITS(7.5, UNIT_1_25_MS))  // in units of 1.25 ms. */
#define CONN_INTERVAL_MAX (uint16_t)(MSEC_TO_UNITS(500, UNIT_1_25_MS)) 
#define CONN_SUP_TIMEOUT (uint16_t)(MSEC_TO_UNITS(4000,  UNIT_10_MS)) // 4 sec timeout 
#define SLAVE_LATENCY 0

typedef struct {
  uint16_t att_mtu;      
  uint16_t conn_interval;  
  ble_gap_phys_t  phys;          
  uint8_t data_len;     
  bool conn_evt_len_ext_enabled;  
  } test_params_t;
static test_params_t m_test_params = {
  .att_mtu = NRF_SDH_BLE_GATT_MAX_MTU_SIZE,
  .data_len = NRF_SDH_BLE_GAP_DATA_LENGTH,
  .conn_interval = CONN_INTERVAL_DEFAULT,
  .conn_evt_len_ext_enabled = true,
  // .phys.tx_phys = BLE_GAP_PHY_2MBPS | BLE_GAP_PHY_1MBPS | BLE_GAP_PHY_CODED,
  // .phys.rx_phys = BLE_GAP_PHY_2MBPS | BLE_GAP_PHY_1MBPS | BLE_GAP_PHY_CODED
  .phys.tx_phys = BLE_GAP_PHY_1MBPS,
  .phys.rx_phys = BLE_GAP_PHY_1MBPS
   };

/*******************************************************************
 * application main data structure, initially should be zeroed out 
 */ 
typedef struct balapp_s balapp_t;
struct balapp_s {
  ble_gap_addr_t peer_addr;
  uint16_t conn_handle;
  // NOTE: ble_gatts_char_handles_t is defined in ble_gatts.h, 
  // and contains handles: value_handle, cccd_handle, and a couple
  // more that we are not interested in; here we have one such 
  // group for each characteristic in the Balance BPS primary service
  // Note: here [I],[N],[W] are for Indicate, Notify and Write
  ble_gatts_char_handles_t serv8a91; // the Measurement service [I]
  ble_gatts_char_handles_t serv8a92; // the Intermediate Meas service [N]
  ble_gatts_char_handles_t serv8a81; // the Download (ie command) servicei [W]
  ble_gatts_char_handles_t serv8a82; // the Upload service [I]
  // REMARK: on the server side, the "ble_add_char_params_t" is used
  // to add characteristics to a service, fields in ble_srv_common.h
  // BELOW are flags (hopefully initially false by zero) meaning that the
  // structs above are not yet established; should all be True after 
  // the discovery phase is complete
  bool got8a91;
  bool got8a92;
  bool got8a81;
  bool got8a82;
  uint8_t buffer[64];  // general purpose read/write buffer
  uint8_t index;       // index into the buffer
  };
balapp_t m_balapp;
// define Balance BPS Monitor's primary UUID for comparison
ble_uuid_t balance_uuid = {
  .type = BLE_UUID_TYPE_BLE,
  .uuid = BALANCE_UUID_PRIMARY 
  };

// for debugging/info
static char const * const m_dis_char_names[] = {
  "Manufacturer Name String",
  "Model Number String     ",
  "Serial Number String    ",
  "Hardware Revision String",
  "Firmware Revision String",
  "Software Revision String",
  "System ID",
  "IEEE 11073-20601 Regulatory Certification Data List",
  "PnP ID"
  };

NRF_BLE_SCAN_DEF(m_scan);
NRF_BLE_GATT_DEF(m_gatt);
BLE_DIS_C_DEF(m_ble_dis_c);
BLE_DB_DISCOVERY_DEF(m_db_disc);
APP_TIMER_DEF(timer_id);
APP_TIMER_DEF(kickoff_id);

/*

NRF_BLE_GATT_DEF(m_gatt);
BLE_DISC_C_DEF(m_ble_dis_c);  
BLE_DB_DISCOVERY_DEF(m_db_disc);
NRF_BLE_SCAN_DEF(m_scan);
APP_TIMER_DEF(timer_id);
APP_TIMER_DEF(kickoff_id);
*/

// not really used for a scan parameter, but needed to connect
static const ble_gap_scan_params_t m_scan_param = {
  .active         = 0,    // Passive scanning.
  .filter_policy  = BLE_GAP_SCAN_FP_ACCEPT_ALL, // Do not use whitelist.
  .interval       = (uint16_t)SCAN_INTERVAL, // Scan interval.
  .window         = (uint16_t)SCAN_WINDOW,   // Scan window.
  .timeout        = 0,    // Never stop scanning unless explicit asked to.
  .scan_phys      = BLE_GAP_PHY_AUTO            // Automatic PHY selection.
   };

// Connection parameters requested for connection.
static ble_gap_conn_params_t m_conn_param = {
  .min_conn_interval = CONN_INTERVAL_MIN,  
  .max_conn_interval = CONN_INTERVAL_MAX, 
  .slave_latency     = SLAVE_LATENCY,      
  .conn_sup_timeout  = CONN_SUP_TIMEOUT  
  };

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name) {
  app_error_handler(0xDEADBEEF, line_num, p_file_name);
  }

// Scanning: init, start, parse
uint32_t adv_report_parse(uint8_t type, 
		ble_data_t * p_advdata, 
		ble_data_t * p_typedata) {
  // this parser extracts name, service, etc, from an advertisement payload
  // argument "type" says what kind of field to extract
  uint32_t index = 0;
  uint8_t * p_data;
  p_data = p_advdata->p_data;
  while (index < p_advdata->len) {
    uint8_t field_length = p_data[index];
    uint8_t field_type = p_data[index+1];
    if (field_type == type) {
      p_typedata->p_data = &p_data[index+2];
      p_typedata->len = field_length-1;
      return NRF_SUCCESS;
      }
    index += field_length+1;
    }
  return NRF_ERROR_NOT_FOUND;
  }
void scan_init() { // use simple form, no filters
  ret_code_t ret;
  ret = nrf_ble_scan_init(&m_scan, NULL, NULL);
  APP_ERROR_CHECK(ret);
  }
void scan_start() {
  ret_code_t ret;
  ret = nrf_ble_scan_start(&m_scan);
  APP_ERROR_CHECK(ret);
  ret = bsp_indication_set(BSP_INDICATE_SCANNING);
  APP_ERROR_CHECK(ret);
  }
void connect_task(void * parameter, uint16_t size) {
  ret_code_t err_code;
  m_balapp.conn_handle = BLE_CONN_HANDLE_INVALID; 
  err_code = sd_ble_gap_connect(&m_balapp.peer_addr,
      &m_scan_param,
      &m_conn_param,
      APP_BLE_CONN_CFG_TAG);
  if (err_code != NRF_SUCCESS) 
    NRF_LOG_INFO("sd_ble_gap_connect failed 0x%x",err_code);
  }
void scan_inspect(ble_gap_evt_adv_report_t const * p_adv) {
  // scrutinze an advertisement, returning ??? if it is a device worth
  // connecting to and investigating further
  /*** Note, for the adv_report, the library the header says one thing, 
   * the documentation says otherwise -- documentation says it has seven fields, 
   * peer_addr, direct_addr, rssi, then an aggregate byte
   * for scan_rsp, type, and dlen (5 bits), followed by up to
   * 31 bytes of data -- and if scan_rsp is 1, ignore this report
   * BUT one header file (ble_gap.h) has this instead:
   * 1. type field, two bytes, it is a bit-defined aggregate 
   *    uint16_t connectable: 1
   *    uint16_t scannable: 1
   *    uint16_t directed: 1
   *    uint16_t scan_response: 1
   *    uint16_t extended_pdu: 1
   *    uint16_t status: 2 
   *    uint16_t reserved: 9
   * 2. peer address (ble_gap_addr_t) 
   * 3. direct address (ble_gap_addr_t)
   * 4. primary_phy, a byte
   * 5. secondary phy, a byte
   * 6. tx_power, a byte
   * 7. rssi, a byte 
   * 8. ch_index, a byte for channel index of this report 
   * 9. set_id, a byte
   * 10. data_id, two bytes but really 12 bits
   * 11. data, which is a ble_data_t (len and pointer)
  *****/
	    // uint8_t* q = p_adv->data.p_data;
	    // uint8_t  n = p_adv->data.len;
  ret_code_t err_code;
  ble_data_t inspectName; // ble_data_t has just two fields, 
  ble_data_t inspectServ; //   len and p_data (pointer)
  uint16_t service_code = 0;
  // unclear we should be getting scan response packet (more advanced)
  if (p_adv->type.scan_response == 1) return;
  // for debugging, let's ignore the non-connectable devices
  if (p_adv->type.connectable == 0) return; 
  NRF_LOG_INFO("adv evt rssi %d channel %d",p_adv->rssi,p_adv->ch_index);
  NRF_LOG_INFO("scannable %d directed %d resp %d",
			    p_adv->type.scannable,
			    p_adv->type.directed,
                            p_adv->type.scan_response
			    );
  err_code = adv_report_parse(9,&m_scan.scan_buffer,&inspectName);
  if (err_code == NRF_SUCCESS) { 
    NRF_LOG_INFO("name");
    NRF_LOG_RAW_HEXDUMP_INFO (inspectName.p_data, inspectName.len);
    }
  err_code = adv_report_parse(3,&m_scan.scan_buffer,&inspectServ);
    if (err_code == NRF_SUCCESS && inspectServ.len == 2)  { 
      service_code = *(uint16_t*)inspectServ.p_data;
      NRF_LOG_INFO("service %x vs %x",service_code,BALANCE_UUID_PRIMARY);
      }
  if (service_code == BALANCE_UUID_PRIMARY) {
    memcpy((uint8_t*)&m_balapp.peer_addr,
           (uint8_t*)&p_adv->peer_addr,sizeof(ble_gap_addr_t));
    NRF_LOG_INFO("peer address");
    NRF_LOG_RAW_HEXDUMP_INFO(m_balapp.peer_addr.addr,6);
    app_sched_event_put(NULL,1,&connect_task);
    }
  }

uint32_t ble_dis_c_all_chars_read(void) {
  // determines number of characteristics present
  ret_code_t err_code;
  uint32_t disc_char_num = 0;
  for (ble_dis_c_char_type_t char_type = (ble_dis_c_char_type_t) 0;
         char_type < BLE_DIS_C_CHAR_TYPES_NUM;
         char_type++) {
    err_code = ble_dis_c_read(&m_ble_dis_c, char_type);
    // NRF_ERROR_INVALID_STATE => characteristic is not present 
    if (err_code != NRF_ERROR_INVALID_STATE) {
       APP_ERROR_CHECK(err_code);
       disc_char_num++;
       }
    }
  return disc_char_num;
  }

// for debugging/info, show characteristic names
void ble_dis_c_string_char_log(ble_dis_c_char_type_t char_type,
  ble_dis_c_string_t const * const p_string) {
  char response_data_string[BLE_DIS_C_STRING_MAX_LEN] = {0};
  if (sizeof(response_data_string) > p_string->len) {
    memcpy(response_data_string, p_string->p_data, p_string->len);
    NRF_LOG_INFO("%s: %s",
      m_dis_char_names[char_type],
      nrf_log_push((char *) response_data_string));
      }
    else {
      NRF_LOG_ERROR("String buffer for DIS characteristics is too short.")
      }
  }

// for debugging/info, show vendor and uuid info
void ble_dis_c_system_id_log(ble_dis_sys_id_t const * const p_sys_id) {
  NRF_LOG_INFO("%s:", m_dis_char_names[BLE_DIS_C_SYS_ID]);
  NRF_LOG_INFO(" Manufacturer Identifier: 0x%010X", p_sys_id->manufacturer_id);
  NRF_LOG_INFO(" Organizationally Unique Identifier: 0x%06X", 
		  p_sys_id->organizationally_unique_id);
  }

// for debugging/info, show regulatory certification data
void ble_dis_c_cert_list_log(ble_dis_reg_cert_data_list_t const * const 
		p_cert_list) {
  NRF_LOG_INFO("%s:", m_dis_char_names[BLE_DIS_C_CERT_LIST]);
  NRF_LOG_HEXDUMP_INFO(p_cert_list->p_list, p_cert_list->list_len);
  }

// for debugging/ino, show PnP characteristic data
void ble_dis_c_pnp_id_log(ble_dis_pnp_id_t const * const p_pnp_id) {
  NRF_LOG_INFO("%s:", m_dis_char_names[BLE_DIS_C_PNP_ID]);
  NRF_LOG_INFO(" Vendor ID Source: 0x%02X", p_pnp_id->vendor_id_source);
  NRF_LOG_INFO(" Vendor ID:        0x%04X", p_pnp_id->vendor_id);
  NRF_LOG_INFO(" Product ID:       0x%04X", p_pnp_id->product_id);
  NRF_LOG_INFO(" Product Version:  0x%04X", p_pnp_id->product_version);
  }

void ble_dis_c_evt_handler(ble_dis_c_t * p_ble_dis_c, 
		ble_dis_c_evt_t const * p_ble_dis_evt) {
  ret_code_t err_code;
  static uint32_t disc_chars_num     = 0;
  static uint32_t disc_chars_handled = 0;
  ble_dis_c_evt_read_rsp_t* p_read_rsp; 
  switch (p_ble_dis_evt->evt_type) {
    case BLE_DIS_C_EVT_DISCOVERY_COMPLETE:
      err_code = ble_dis_c_handles_assign(p_ble_dis_c,
                   p_ble_dis_evt->conn_handle,
                   p_ble_dis_evt->params.disc_complete.handles);
      APP_ERROR_CHECK(err_code);
      disc_chars_num = ble_dis_c_all_chars_read();
      disc_chars_handled = 0;
      NRF_LOG_INFO("Device Information Service discovered.");
      break;
    case BLE_DIS_C_EVT_DIS_C_READ_RSP: 
      p_read_rsp = (ble_dis_c_evt_read_rsp_t*)&p_ble_dis_evt->params.read_rsp;
      if ((disc_chars_handled == 0) && (disc_chars_num != 0)) {
        NRF_LOG_INFO("Device Information:");
        switch (p_read_rsp->char_type) {
          case BLE_DIS_C_MANUF_NAME:
          case BLE_DIS_C_MODEL_NUM:
          case BLE_DIS_C_SERIAL_NUM:
          case BLE_DIS_C_HW_REV:
          case BLE_DIS_C_FW_REV:
          case BLE_DIS_C_SW_REV:
            ble_dis_c_string_char_log(p_read_rsp->char_type, 
		  &p_read_rsp->content.string);
            break;
          case BLE_DIS_C_SYS_ID:
            ble_dis_c_system_id_log(&p_read_rsp->content.sys_id);
            break;
          case BLE_DIS_C_CERT_LIST:
            ble_dis_c_cert_list_log(&p_read_rsp->content.cert_list);
            break;
          case BLE_DIS_C_PNP_ID:
            ble_dis_c_pnp_id_log(&p_read_rsp->content.pnp_id);
            break;
          default:
            break;
          }}
      disc_chars_handled++;
      if (disc_chars_handled == disc_chars_num) {
        disc_chars_handled = 0;
        disc_chars_num     = 0;
        }
      break;
    case BLE_DIS_C_EVT_DIS_C_READ_RSP_ERROR:
      NRF_LOG_ERROR("Read request for: %s characteristic failed with gatt_status: 0x%04X.",
       m_dis_char_names[p_ble_dis_evt->params.read_rsp.char_type],
       p_ble_dis_evt->params.read_rsp_err.gatt_status);
       break;
    case BLE_DIS_C_EVT_DISCONNECTED:
       break;
    }
  }

void db_disc_handler(ble_db_discovery_evt_t * p_evt) {
  // Type can be one of these:
  //   BLE_DB_DISCOVERY_COMPLETE, BLE_DB_DISCOVERY_ERROR, 
  //   BLE_DB_DISCOVERY_SRV_NOT_FOUND, BLE_DB_DISCOVERY_AVAILABLE
  // Note: p_evt-> params.discovered_db 
  //   (type is ble_gatt_db_srv_t) is only valid if the discovery 
  //   is complete, and has these fields:
  //       ble_uuid_t srv_uuid  --  UUID of the service
  //       uint8_t char_count   --  number of characteristics 
  //       ble_gattc_handle_range_t handle_range  -- range of handles
  //         (above is a struct with just two 16-bit handles named
  //          start_handle and end_handle) 
  //       ble_gatt_db_char_t       characteristics[BLE_GATT_DB_MAX_CHARS]
  //   the program should iterate for the char_count items in 
  //   characteristics; with each, fields of ble_gatt_db_char_t are:
  //       characteristic -- type is ble_gattc_char_t, see below
  //       then cccd_handle (super important), plus three rarely used:
  //       ext_prop_handle, user_desc_handle, report_ref_handle (prolly n/a)
  //   the ble_gattc_char_t type has these fields
  //       uuid -- type is ble_uuit_t, which has fields type and uuid
  //       char_props -- type is ble_gatt_char_props_t
  //       char_ext_props (1 bit) -- if there are extended properties
  //       handle_decl -- handle of the characteristic declaration
  //       handle_value -- handle of the characteristic value
  NRF_LOG_INFO("discovery event type %x",p_evt->evt_type);
  switch (p_evt->evt_type) {
    case BLE_DB_DISCOVERY_ERROR: 
      NRF_LOG_INFO("discovery event error"); break;
    case BLE_DB_DISCOVERY_SRV_NOT_FOUND: 
      NRF_LOG_INFO("discovery service not found"); break;
    case BLE_DB_DISCOVERY_AVAILABLE:
      NRF_LOG_INFO("discovery service available"); break;
    default: break;
    }
  if (p_evt->evt_type != BLE_DB_DISCOVERY_COMPLETE) return;
  NRF_LOG_INFO("discovery handle %x",p_evt->conn_handle);
  NRF_LOG_INFO("discovery event service uuid %x",
      p_evt->params.discovered_db.srv_uuid.uuid);
  NRF_LOG_INFO("discovery event charac count %d",
      p_evt->params.discovered_db.char_count);
  NRF_LOG_INFO("discovery event handle range 0x%x:0x%x",
      p_evt->params.discovered_db.handle_range.start_handle,
      p_evt->params.discovered_db.handle_range.end_handle);
  for (int i=0; i < p_evt->params.discovered_db.char_count; i++) {
      ble_gatt_db_char_t* q = 
	      // misspelling of characteristics is Nordic's fault
	      &p_evt->params.discovered_db.charateristics[i];
      uint8_t properties = *(uint8_t*)&q->characteristic.char_props;
      NRF_LOG_INFO("uuid 0x%x cccd 0x%x handle 0x%x properties 0x%x",
        q->characteristic.uuid.uuid,
        q->cccd_handle,
        q->characteristic.handle_value,
	properties);
      }
  // ble_dis_c_on_db_disc_evt(&m_ble_dis_c, p_evt);
  }

void soc_evt_handler(uint32_t evt_id, void * p_context) {
  switch (evt_id) {
    case NRF_EVT_FLASH_OPERATION_SUCCESS:
    case NRF_EVT_FLASH_OPERATION_ERROR:
         scan_start();
      break;
    default:
      break;
    }
  }

/*
void on_ble_gap_evt_connected(ble_gap_evt_t const * p_gap_evt) {
  ret_code_t err_code;
  NRF_LOG_INFO("on_ble_gap_evt_connected %x",p_gap_evt->conn_handle);
  m_balapp.conn_handle = p_gap_evt->conn_handle;
  // nrf_ble_scan_stop();
  err_code = ble_db_discovery_start(&m_db_disc, 
		 p_gap_evt->conn_handle);
  // err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr,m_balapp.conn_handle);
  NRF_LOG_INFO("db discovery start got %d",err_code);
  NRF_LOG_FLUSH();
  APP_ERROR_CHECK(err_code);
  }
  */

void timer_handler(void * p_context) {
  UNUSED_PARAMETER(p_context);
  }

void kickoff_handler(void * p_context) {
  }

void timers_init(void) {
  ret_code_t err_code;
  err_code = app_timer_init();
  APP_ERROR_CHECK(err_code);
  err_code = app_timer_create(&timer_id,
                  APP_TIMER_MODE_REPEATED, timer_handler);
  APP_ERROR_CHECK(err_code);
  err_code = app_timer_create(&kickoff_id,
                  // APP_TIMER_MODE_SINGLE_SHOT,
                  APP_TIMER_MODE_REPEATED,
                  kickoff_handler);
  APP_ERROR_CHECK(err_code);
  }
void timers_start(void) {
  app_timer_start(timer_id,APP_TIMER_TICKS(1000),NULL);
  app_timer_start(kickoff_id,APP_TIMER_TICKS(10),NULL);
  }

/*
static void on_ble_gatt_write(const ble_evt_t * p_ble_evt) {
  uint32_t data_buffer;
  ble_gatts_value_t rx_data;
  ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
  m_balapp.debug1 = p_evt_write->handle;
  // app_sched_event_put(NULL,1,&debug);
  if (    p_evt_write->handle == m_balapp.clock_handle.cccd_handle
       && p_evt_write->len == 2 ) {
     if (ble_srv_is_notification_enabled(p_evt_write->data)) {
        bsp_board_led_on(3);
        m_balapp.notifyset = true;
        // not sure what to do here; maybe set some flag?
        return;
        }
     return;   /// different write op???
     }
  rx_data.len = sizeof(uint32_t);
  rx_data.offset = 0;
  rx_data.p_value = (uint8_t*)&data_buffer;
  if (p_ble_evt->evt.gatts_evt.params.write.handle ==
        m_balapp.setclock_handle.value_handle) {
           sd_ble_gatts_value_get(
              m_balapp.conn_handle,
              m_balapp.setclock_handle.value_handle,
              &rx_data);
        NRF_LOG_INFO("gatt write");
        }
  }
*/

/* only used with filter and scan_init has to set up
void scan_evt_handler(scan_evt_t const * p_scan_evt) {
  ret_code_t err_code;
  NRF_LOG_INFO("scan evt handler");
  switch(p_scan_evt->scan_evt_id) {
    case NRF_BLE_SCAN_EVT_CONNECTING_ERROR: {
      err_code = p_scan_evt->params.connecting_err.err_code;
      APP_ERROR_CHECK(err_code);
      } break;
    case NRF_BLE_SCAN_EVT_CONNECTED: {
      ble_gap_evt_connected_t const * p_connected =
             p_scan_evt->params.connected.p_connected;
      // Scan is automatically stopped by the connection.
      NRF_LOG_INFO("Connecting to target %02x%02x%02x%02x%02x%02x",
          p_connected->peer_addr.addr[0],
          p_connected->peer_addr.addr[1],
          p_connected->peer_addr.addr[2],
          p_connected->peer_addr.addr[3],
          p_connected->peer_addr.addr[4],
          p_connected->peer_addr.addr[5]
          );
      } break;
    case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT: {
      NRF_LOG_INFO("Scan timed out.");
      scan_start();
      } break;

    default:
      break;
    }
  }
*/

void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context) {
  ret_code_t err_code;
  ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;
  NRF_LOG_INFO("ble_evt_handler %x",p_ble_evt->header.evt_id);

  switch (p_ble_evt->header.evt_id) {
    case BLE_GAP_EVT_CONNECTED:
      m_balapp.conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
      NRF_LOG_INFO("connected event with handle %x",
                    m_balapp.conn_handle);
      err_code = ble_db_discovery_start(&m_db_disc,m_balapp.conn_handle);
      APP_ERROR_CHECK(err_code);
      err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_GAP_EVT_DISCONNECTED:
      NRF_LOG_INFO("Disconnected. conn_handle: 0x%x, reason: 0x%x",
                         p_gap_evt->conn_handle,
                         p_gap_evt->params.disconnected.reason);
      // on_ble_gap_evt_disconnected(p_gap_evt);
      break;

    case BLE_GAP_EVT_TIMEOUT:
      if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN) {
        NRF_LOG_INFO("Connection Request timed out.");
        }
      break;

    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
      err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle,             BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_GAP_EVT_CONN_PARAM_UPDATE: {
      // m_conn_interval_configured = true;
      NRF_LOG_INFO("Connection interval updated: 0x%x, 0x%x.",
           p_gap_evt->params.conn_param_update.conn_params.min_conn_interval,
           p_gap_evt->params.conn_param_update.conn_params.max_conn_interval);
      } 
      break;

    case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST: {
      ble_gap_conn_params_t params;
      params = p_gap_evt->params.conn_param_update_request.conn_params;
      err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle, &params);
      APP_ERROR_CHECK(err_code);
      NRF_LOG_INFO("Connection interval updated (upon request): 0x%x, 0x%x.",
          p_gap_evt->params.conn_param_update_request.conn_params.min_conn_interval,
          p_gap_evt->params.conn_param_update_request.conn_params.max_conn_interval);
      } 
      break;

    case BLE_GATTS_EVT_SYS_ATTR_MISSING: {
      err_code = sd_ble_gatts_sys_attr_set(p_gap_evt->conn_handle, NULL, 0, 0);
      APP_ERROR_CHECK(err_code);
      } 
      break;

    case BLE_GAP_EVT_PHY_UPDATE: {
      NRF_LOG_INFO("PHY update event.");
      ble_gap_evt_phy_update_t const * p_phy_evt = &p_ble_evt->evt.gap_evt.params.phy_update;
      if (p_phy_evt->status == BLE_HCI_STATUS_CODE_LMP_ERROR_TRANSACTION_COLLISION) {
        // Ignore LL collisions.
        NRF_LOG_DEBUG("LL transaction collision during PHY update.");
        } 
        // on_ble_gap_evt_connected(p_gap_evt);
      } 
      break;

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
      err_code = sd_ble_gap_phy_update(p_gap_evt->conn_handle, &m_test_params.phys);
      APP_ERROR_CHECK(err_code);
      } 
      break;

    case BLE_GATTC_EVT_TIMEOUT:
      // Disconnect on GATT Client timeout event.
      NRF_LOG_DEBUG("GATT Client Timeout.");
      err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
            BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      if (err_code != NRF_SUCCESS && err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
         err_code != NRF_ERROR_INVALID_STATE) {
         APP_ERROR_CHECK(err_code);
         }
      break;

    case BLE_GATTS_EVT_TIMEOUT:
      // Disconnect on GATT Server timeout event.
      NRF_LOG_DEBUG("GATT Server Timeout.");
      err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      if (err_code != NRF_SUCCESS && err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
         err_code != NRF_ERROR_INVALID_STATE) {
         APP_ERROR_CHECK(err_code);
	 }
      break;

    case BLE_GAP_EVT_ADV_REPORT: {
      NRF_LOG_INFO("adv report");
      scan_inspect(&p_gap_evt->params.adv_report);
      } break;

    case BLE_GATTC_EVT_HVX: 
      break;

     // client gets write (to CCCD) from server
     case BLE_GATTS_EVT_WRITE:
       // on_ble_gatt_write(p_ble_evt);
       break;

     case BLE_GATTS_EVT_HVN_TX_COMPLETE:
       // on_tx_complete(p_ble_evt);
       break;
	 
     case BLE_GATTC_EVT_WRITE_RSP:
       // on_write_response(p_ble_evt);
       break;

     case BLE_GATTC_EVT_READ_RSP:
       // on_read_response(p_ble_evt);
       break;

     default:
       break;
     }
  }

void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, 
		nrf_ble_gatt_evt_t const * p_evt) {
  if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED) {
    NRF_LOG_INFO("ATT MTU exchange completed.");
    }
  }

void gatt_init() {
  ret_code_t err_code;
  err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
  APP_ERROR_CHECK(err_code);
  err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, 
		  NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
  APP_ERROR_CHECK(err_code);
  }

void db_discovery_init() {
  ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
  APP_ERROR_CHECK(err_code);
  err_code = ble_db_discovery_evt_register(&balance_uuid);
  APP_ERROR_CHECK(err_code);
  }

void softdevice_init() {
  ret_code_t err_code;
  err_code = nrf_sdh_enable_request();
  APP_ERROR_CHECK(err_code);
  }

void ble_stack_init(void) { 
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
  NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO,
		  ble_evt_handler, NULL);
  NRF_SDH_SOC_OBSERVER(m_soc_observer, APP_SOC_OBSERVER_PRIO,
		  soc_evt_handler, NULL);
  }

void bsp_event_handler(bsp_event_t event) {
  switch (event) {
     case BSP_EVENT_SLEEP:
       nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
       break;
     case BSP_EVENT_DISCONNECT:
       break;
     default:
       break;
     }
  }

/**@brief Function for initializing buttons and leds. */
void buttons_leds_init(void) {
  ret_code_t err_code;
  bsp_event_t startup_event;
  err_code = bsp_init(BSP_INIT_LEDS, bsp_event_handler);
  APP_ERROR_CHECK(err_code);
  err_code = bsp_btn_ble_init(NULL, &startup_event);
  APP_ERROR_CHECK(err_code);
  }

void log_init(void) {
  ret_code_t err_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(err_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  }

void power_management_init(void) {
  ret_code_t err_code;
  err_code = nrf_pwr_mgmt_init();
  APP_ERROR_CHECK(err_code);
  }

void idle_state_handle(void) {
  if (NRF_LOG_PROCESS() == false) {
    nrf_pwr_mgmt_run();
    }
  }

int main(void) {
  log_init();
  timers_init();
  buttons_leds_init();
  APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
  power_management_init();
  softdevice_init();
  ble_stack_init();
  gatt_init();
  timers_start();
  db_discovery_init(); // prepare for UUID discovery upon connect
  scan_init();
  scan_start();
  NRF_LOG_INFO("main loop starts");

  for (;;) {
    bsp_board_led_invert(1);
    idle_state_handle();
    NRF_LOG_FLUSH();
    app_sched_execute();
    }
  }
/*
 * note - ble_gatt.h defines things like BLE_GATT_OP_WRITE_REQ and
 * more, like BLE_GATT_HVX_INDICATION 
 *
static ret_code_t cccd_configure(uint16_t conn_handle, uint16_t handle_cccd, bool enable)
{
    NRF_LOG_DEBUG("Configuring CCCD. CCCD Handle = %d, Connection Handle = %d",
        handle_cccd, conn_handle);

    tx_message_t * p_msg;
    uint16_t       cccd_val = enable ? BLE_GATT_HVX_NOTIFICATION : 0;

    p_msg              = &m_tx_buffer[m_tx_insert_index++];
    m_tx_insert_index &= TX_BUFFER_MASK;

    p_msg->req.write_req.gattc_params.handle   = handle_cccd;
    p_msg->req.write_req.gattc_params.len      = WRITE_MESSAGE_LENGTH;
    p_msg->req.write_req.gattc_params.p_value  = p_msg->req.write_req.gattc_value;
    p_msg->req.write_req.gattc_params.offset   = 0;
    p_msg->req.write_req.gattc_params.write_op = BLE_GATT_OP_WRITE_REQ;
    p_msg->req.write_req.gattc_value[0]        = LSB_16(cccd_val);
    p_msg->req.write_req.gattc_value[1]        = MSB_16(cccd_val);
    p_msg->conn_handle                         = conn_handle;
    p_msg->type                                = WRITE_REQ;

    tx_buffer_process();

    return NRF_SUCCESS;
}

static void tx_buffer_process(void)
{
    if (m_tx_index != m_tx_insert_index)
    {
        uint32_t err_code;

        if (m_tx_buffer[m_tx_index].type == READ_REQ)
        {
            err_code = sd_ble_gattc_read(m_tx_buffer[m_tx_index].conn_handle,
                                         m_tx_buffer[m_tx_index].req.read_handle,
                                         0);
        }
        else
        {
            err_code = sd_ble_gattc_write(m_tx_buffer[m_tx_index].conn_handle,
                                          &m_tx_buffer[m_tx_index].req.write_req.gattc_params);
        }


        if (err_code == NRF_SUCCESS)
        {
            m_tx_index++;
            m_tx_index &= TX_BUFFER_MASK;
        }
        else
        {
            NRF_LOG_DEBUG("SD Read/Write API returns error, will retry later.");
        }
    }
}

// Function for handling Handle Value Notification received from the SoftDevice.
static void on_hvx(nrf_ble_amtc_t * p_ctx, ble_evt_t const * p_ble_evt)
{
    // Check if the event if on the link for this instance
    if (p_ctx->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
    {
        return;
    }


    // Check if this is a AMT notification.
    if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ctx->peer_db.amt_handle)
    {
        nrf_ble_amtc_evt_t amt_c_evt;
        p_ctx->bytes_rcvd_cnt           += p_ble_evt->evt.gattc_evt.params.hvx.len;
        amt_c_evt.evt_type              = NRF_BLE_AMT_C_EVT_NOTIFICATION;
        amt_c_evt.conn_handle           = p_ble_evt->evt.gattc_evt.conn_handle;
        amt_c_evt.params.hvx.notif_len  = p_ble_evt->evt.gattc_evt.params.hvx.len;
        amt_c_evt.params.hvx.bytes_sent = uint32_decode(p_ble_evt->evt.gattc_evt.params.hvx.data);
        amt_c_evt.params.hvx.bytes_rcvd = p_ctx->bytes_rcvd_cnt;
        p_ctx->evt_handler(p_ctx, &amt_c_evt);
    }
}

static void on_read_response(nrf_ble_amtc_t * p_ctx, ble_evt_t const * p_ble_evt)
{
    // Check if the event if on the link for this instance
    if (p_ctx->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
    {
        return;
    }

    // Check if this is a AMT RCB read response.
    if (p_ble_evt->evt.gattc_evt.params.read_rsp.handle == p_ctx->peer_db.amt_rbc_handle)
    {
        nrf_ble_amtc_evt_t amt_c_evt;
        amt_c_evt.evt_type             = NRF_BLE_AMT_C_EVT_RBC_READ_RSP;
        amt_c_evt.conn_handle          = p_ble_evt->evt.gattc_evt.conn_handle;
        amt_c_evt.params.rcv_bytes_cnt = uint32_decode(p_ble_evt->evt.gattc_evt.params.read_rsp.data);
        p_ctx->evt_handler(p_ctx, &amt_c_evt);
    }
}
static void on_write_response(nrf_ble_amtc_t * p_ctx, ble_evt_t const * p_ble_evt)
{
    // Check if the event if on the link for this instance
    if (p_ctx->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
    {
        return;
    }

    // Check if this is a write response on the CCCD.
    if (p_ble_evt->evt.gattc_evt.params.write_rsp.handle == p_ctx->peer_db.amt_cccd_handle)
    {
        NRF_LOG_DEBUG("CCCD configured.");
    }
}

.......
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTC_EVT_HVX:
            on_hvx(p_ctx, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnected(p_ctx, p_ble_evt);
            break;

        case BLE_GATTC_EVT_WRITE_RSP:
            on_write_response(p_ctx, p_ble_evt);
            break;

        case BLE_GATTC_EVT_READ_RSP:
            on_read_response(p_ctx, p_ble_evt);
            break;

......

*/
