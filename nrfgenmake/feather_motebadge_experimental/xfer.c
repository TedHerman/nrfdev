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
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrfx_wdt.h"
#include "nrf_ble_scan.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_fstorage.h"
#include "nrf_802154.h"
#include "nrf_drv_clock.h"
#include "motebadge.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define MANUFACTURER_NAME "Signal Labs"
#define BLE_UUID_OUR_BASE_UUID {0x23, 0xD1, 0x13, 0xEF, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}
#define BLE_UUID_OUR_SERVICE 0x152C
#define CLOCK_READ_UUID 0x3907
#define CLOCK_READ_INDX 0
#define CLOCK_WRITE_UUID 0x3909
#define CLOCK_WRITE_INDX 1
#define NUMBER_OUR_ATTRIBUTES 2

#define APP_BLE_CONN_CFG_TAG    1                                       
#define APP_BLE_OBSERVER_PRIO   3
#define APP_SOC_OBSERVER_PRIO   1
#define SCAN_INTERVAL 180 // scan interval in units of 0.625 millisecond
#define SCAN_WINDOW 90    // scan window in units of 0.625 millisecond
#define SCAN_TIMEOUT 0    // 0 means continuous 

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
  .phys.tx_phys = BLE_GAP_PHY_2MBPS,
  .phys.rx_phys = BLE_GAP_PHY_2MBPS
   };

typedef struct bulkcli_s bulkcli_t;
struct bulkcli_s {
  ble_uuid_t service_uuid;
  uint16_t conn_handle;
  uint16_t service_handle;
  ble_gatts_char_handles_t clock_handle;
  ble_add_char_params_t clock_params;
  ble_gatts_char_handles_t setclock_handle;
  ble_add_char_params_t setclock_params;
  uint8_t uuid_type;
  // uint8_t clock[DATA_LENGTH_MAX];
  uint32_t setclock;
  uint16_t debug1;
  bool notifyset;
  bool fullyconnected;
  bool busy;
  };
static bulkcli_t m_bulkcli;

NRF_BLE_QWR_DEF(m_qwr); 
NRF_BLE_SCAN_DEF(m_scan);
NRF_BLE_GATT_DEF(m_gatt);

APP_TIMER_DEF(ble_scanwatch_id);  // to limit transfer attempt to 20 seconds or so
APP_TIMER_DEF(kickoff_id);        // to start/repeat hvx activity
APP_TIMER_DEF(progress_id);       // mainly for debugging 

static char const m_target_periph_name[] = "Signal Labs";

static bool m_memory_access_in_progress;
static bool isDISset = false;
uint8_t progressClock;

extern bool xfer_active;
extern void neopixel_write (uint8_t *pixels);
extern void neopixel_init();
extern uint8_t* mem_fetch();
extern uint32_t mem_cursor();
extern void mem_rewind();
extern void mem_restore();
extern void mem_undo();
extern void main_xfer_end();
void killscan(void * parameter, uint16_t size);

// ref: http://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.sdk5.v15.2.0%2Fgroup__nrf__ble__scan.html
// found in components/softdevice/s140/headers/ble_gap.h
static ble_gap_scan_params_t m_scan_param = {
  .extended      = 0,
  .report_incomplete_evts = 0,
  .active        = 1,
  .filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL,
  .scan_phys     = BLE_GAP_PHY_2MBPS, 
  .interval      = SCAN_INTERVAL,  // 180
  .window        = SCAN_WINDOW,    // 90
  .timeout       = SCAN_TIMEOUT    // 0
  };

// Connection parameters requested for connection.
static ble_gap_conn_params_t m_conn_param = {
  .min_conn_interval = CONN_INTERVAL_MIN,  
  .max_conn_interval = CONN_INTERVAL_MAX, 
  .slave_latency     = SLAVE_LATENCY,      
  .conn_sup_timeout  = CONN_SUP_TIMEOUT  
  };

static uint32_t adv_report_parse(uint8_t type, ble_data_t * p_advdata, ble_data_t * p_typedata) {
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


/**@brief Function for starting scanning. */
static void scan_start(void) {
  ret_code_t ret;
  if (nrf_fstorage_is_busy(NULL)) {
    m_memory_access_in_progress = true;
    return;
    }
  ret = nrf_ble_scan_start(&m_scan);
  APP_ERROR_CHECK(ret);
  ret = bsp_indication_set(BSP_INDICATE_SCANNING);
  APP_ERROR_CHECK(ret);
  }

static void scan_evt_handler(scan_evt_t const * p_scan_evt) {
  ret_code_t err_code;
  ble_gap_evt_adv_report_t const * p_adv =
      p_scan_evt->params.filter_match.p_adv_report;
  ble_gap_scan_params_t const * p_scan_param =
      p_scan_evt->p_scan_params;
  switch(p_scan_evt->scan_evt_id) {
     case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT: 
       break;
     case NRF_BLE_SCAN_EVT_NOT_FOUND: 
       break;	
     case NRF_BLE_SCAN_EVT_FILTER_MATCH: {
       NRF_LOG_INFO("Device \"%s\" found, sending a connection request.",
            (uint32_t) m_target_periph_name);
       m_conn_param.min_conn_interval = CONN_INTERVAL_DEFAULT;
       m_conn_param.max_conn_interval = CONN_INTERVAL_DEFAULT;
       err_code = sd_ble_gap_connect(&p_adv->peer_addr,
          p_scan_param,
          &m_conn_param,
          APP_BLE_CONN_CFG_TAG);
       if (err_code != NRF_SUCCESS) {
          NRF_LOG_ERROR("sd_ble_gap_connect() failed: 0x%x.", err_code);
          }
       } break;
       default: break;
       }
  }

static void soc_evt_handler(uint32_t evt_id, void * p_context) {
  switch (evt_id) {
    case NRF_EVT_FLASH_OPERATION_SUCCESS:
    case NRF_EVT_FLASH_OPERATION_ERROR:
      if (m_memory_access_in_progress) {
         m_memory_access_in_progress = false;
         scan_start();
         }
      break;

    default:
      break;
    }
  }

static void on_ble_gap_evt_connected(ble_gap_evt_t const * p_gap_evt) {
  ret_code_t err_code;
  if (m_bulkcli.conn_handle == BLE_CONN_HANDLE_INVALID) {
    NRF_LOG_INFO("on ble gap evt at time of connect");
    }
  m_bulkcli.conn_handle = p_gap_evt->conn_handle;
  nrf_ble_scan_stop();
  NRF_LOG_INFO("scan stopped");
  err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr,m_bulkcli.conn_handle);
  APP_ERROR_CHECK(err_code);
  }

static void on_ble_gap_evt_disconnected(ble_gap_evt_t const * p_gap_evt) {
  NRF_LOG_INFO("disconnected event");
  m_bulkcli.conn_handle = BLE_CONN_HANDLE_INVALID;   
  m_bulkcli.notifyset = false;
  m_bulkcli.fullyconnected = false;
  m_bulkcli.busy = false;
  killscan(NULL,0); 
  }

static void scan_init(void) {
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;
    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.connect_if_match = false;
    init_scan.conn_cfg_tag = APP_BLE_CONN_CFG_TAG;
    init_scan.p_scan_param = &m_scan_param; // unclear we need scan_param 
    init_scan.p_scan_param = NULL;  // rewrite to omit scan param
    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    // err_code = nrf_ble_scan_init(&m_scan, NULL, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filter_set(&m_scan,
        SCAN_NAME_FILTER,
        m_target_periph_name);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_ble_scan_filters_enable(&m_scan,
        NRF_BLE_SCAN_NAME_FILTER,
        false);
    APP_ERROR_CHECK(err_code);
    }

static void updateblock(void) {
  ret_code_t err_code;
  uint16_t len = DATA_LENGTH_MAX;
  ble_gatts_hvx_params_t hvx_params;
  if (m_bulkcli.conn_handle == BLE_CONN_HANDLE_INVALID) return;
  if (!m_bulkcli.notifyset) return;
  memset(&hvx_params, 0, sizeof(hvx_params));
  hvx_params.handle = m_bulkcli.clock_handle.value_handle;
  hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
  hvx_params.offset = 0;
  hvx_params.p_len = &len;
  hvx_params.p_data = mem_fetch();
  if (hvx_params.p_data == NULL) {
    on_ble_gap_evt_disconnected(NULL);
    return;
    }
  // else
  if (hvx_params.p_data != NULL) {
    // NRF_LOG_RAW_HEXDUMP_INFO((uint8_t*)hvx_params.p_data,8)
    // NRF_LOG_FLUSH();
    err_code = sd_ble_gatts_hvx(m_bulkcli.conn_handle,&hvx_params);
    if (err_code == NRF_ERROR_RESOURCES) {
      m_bulkcli.busy = true; 
      mem_undo();
      NRF_LOG_INFO("gatts_hvx got resource error on block %d",
		 mem_cursor());
      }
    else if (err_code != NRF_SUCCESS) APP_ERROR_CHECK(err_code);
    }
  }

void tryhvx(void * parameter, uint16_t size) {
  if (m_bulkcli.busy) return;
  updateblock();
  }

// this will repeatedly attempt calling updateblock()
void kickoff_handler(void * p_context) {
  if (m_bulkcli.fullyconnected) 
     app_sched_event_put(NULL,1,&tryhvx);
  }

// this is just a debugging device every second
void progress_handler(void * p_context) {
  NRF_LOG_INFO("progress time %d cursor %d",
     progressClock++,mem_cursor()); 
  }

void debug(void * parameter, uint16_t size) {
  NRF_LOG_INFO("%d versus ( %d %d %d %d %d %d )",m_bulkcli.debug1,
      m_bulkcli.clock_handle.value_handle,
      m_bulkcli.clock_handle.user_desc_handle,
      m_bulkcli.clock_handle.cccd_handle,
      m_bulkcli.clock_handle.sccd_handle);
  NRF_LOG_INFO("extra %d %d",m_bulkcli.conn_handle,m_bulkcli.service_handle);
  NRF_LOG_FLUSH();
  }

static void on_ble_gatt_write(const ble_evt_t * p_ble_evt) {
  uint32_t data_buffer;
  ble_gatts_value_t rx_data;
  ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
  m_bulkcli.debug1 = p_evt_write->handle;
  // app_sched_event_put(NULL,1,&debug);
  if (    p_evt_write->handle == m_bulkcli.clock_handle.cccd_handle
       && p_evt_write->len == 2 ) {
     if (ble_srv_is_notification_enabled(p_evt_write->data)) {
        bsp_board_led_on(3);
        m_bulkcli.notifyset = true;
        // not sure what to do here; maybe set some flag?
        return;
        }
     return;   /// different write op???
     }
  rx_data.len = sizeof(uint32_t);
  rx_data.offset = 0;
  rx_data.p_value = (uint8_t*)&data_buffer;
  if (p_ble_evt->evt.gatts_evt.params.write.handle ==
        m_bulkcli.setclock_handle.value_handle) {
           sd_ble_gatts_value_get(
              m_bulkcli.conn_handle,
              m_bulkcli.setclock_handle.value_handle,
              &rx_data);
        NRF_LOG_INFO("gatt write");
        }
  }

static void on_tx_complete(const ble_evt_t * p_ble_evt) {
  m_bulkcli.busy = false;
  }

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context) {
    ret_code_t err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;


    switch (p_ble_evt->header.evt_id) {

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("BLE_GAP_EVT_CONNECTED"); 
            err_code = sd_ble_gap_phy_update(p_gap_evt->conn_handle,&m_test_params.phys);
	    APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected. conn_handle: 0x%x, reason: 0x%x",
                         p_gap_evt->conn_handle,
                         p_gap_evt->params.disconnected.reason);
            on_ble_gap_evt_disconnected(p_gap_evt);
            break;

        case BLE_GAP_EVT_TIMEOUT:
            NRF_LOG_INFO("BLE_GAP_EVT_TIMEOUT"); 
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN) {
               NRF_LOG_INFO("Connection Request timed out.");
               }
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported.
            NRF_LOG_INFO("BLE_GAP_EVT_SEC_PARAMS_REQUEST"); 
            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

	case BLE_GAP_EVT_CONN_PARAM_UPDATE: {
            // m_conn_interval_configured = true;
            NRF_LOG_INFO("Connection interval updated: 0x%x, 0x%x.",
                p_gap_evt->params.conn_param_update.conn_params.min_conn_interval,
                p_gap_evt->params.conn_param_update.conn_params.max_conn_interval);
            } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST: {
            ble_gap_conn_params_t params;
            params = p_gap_evt->params.conn_param_update_request.conn_params;
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle, &params);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Connection interval updated (upon request): 0x%x, 0x%x.",
                p_gap_evt->params.conn_param_update_request.conn_params.min_conn_interval,
                p_gap_evt->params.conn_param_update_request.conn_params.max_conn_interval);
            } break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING: {
            err_code = sd_ble_gatts_sys_attr_set(p_gap_evt->conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            } break;

        case BLE_GAP_EVT_PHY_UPDATE: {
            NRF_LOG_INFO("PHY update request.");
            ble_gap_evt_phy_update_t const * p_phy_evt = &p_ble_evt->evt.gap_evt.params.phy_update;
            if (p_phy_evt->status == BLE_HCI_STATUS_CODE_LMP_ERROR_TRANSACTION_COLLISION) {
                // Ignore LL collisions.
                NRF_LOG_DEBUG("LL transaction collision during PHY update.");
                } 
            on_ble_gap_evt_connected(p_gap_evt);
	    } break;

	case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
            NRF_LOG_INFO("BLE_GAP_EVT_PHY_UPDATE_REQUEST"); 
            err_code = sd_ble_gap_phy_update(p_gap_evt->conn_handle, &m_test_params.phys);
            APP_ERROR_CHECK(err_code);
            } break;

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
	    ble_gap_evt_adv_report_t const * p_adv = &p_gap_evt->params.adv_report;
	    UNUSED_VARIABLE(p_adv);
            ble_data_t inspectName;
	    ble_data_t inspectServ;
	    uint8_t expectServ[] = {0x0A, 0x18};
            err_code = adv_report_parse(9,&m_scan.scan_buffer,&inspectName);
	    if (err_code == NRF_SUCCESS)  { }; 
	       // NRF_LOG_RAW_HEXDUMP_INFO (inspectName.p_data, inspectName.len)
            err_code = adv_report_parse(3,&m_scan.scan_buffer,&inspectServ);
	    if (err_code == NRF_SUCCESS) { }; 
	       // NRF_LOG_RAW_HEXDUMP_INFO (inspectServ.p_data, inspectServ.len)
	    err_code = adv_report_parse(9,&m_scan.scan_buffer,&inspectName);
	    err_code = err_code + adv_report_parse(3,&m_scan.scan_buffer,&inspectServ);
	    if (  err_code == NRF_SUCCESS 
               && inspectName.len == 11 
	       && (memcmp(inspectName.p_data,"Signal Labs",11) == 0)
	       && inspectServ.len == 4 
	       && (memcmp(inspectServ.p_data,expectServ,2) == 0) 
	       ) {
	       NRF_LOG_INFO("Found Signal Labs Server");
	       }
            } break;

	/* MOVE THIS TO SERVER, WHICH NOW SHOULD GET HVX EVENTS
        case BLE_GATTC_EVT_HVX: 
            on_hvx(p_ble_evt);
            break;
	*/

	// client gets write (to CCCD) from server
	case BLE_GATTS_EVT_WRITE:
            NRF_LOG_DEBUG("BLE_GATTS_EVT_WRITE");
            on_ble_gatt_write(p_ble_evt);
            break;

	case BLE_GATTS_EVT_HVN_TX_COMPLETE:
            NRF_LOG_DEBUG("BLE_GATTS_EVT_HVN_TX_COMPLETE");
            on_tx_complete(p_ble_evt);
            break;
	 
	/* MOVE TO SERVER, SINCE CLIENT NO LONGER WRITES OR READS
        case BLE_GATTC_EVT_WRITE_RSP:
            on_write_response(p_ble_evt);
            break;

        case BLE_GATTC_EVT_READ_RSP:
            on_read_response(p_ble_evt);
            break;
	*/

        default:
            break;
        }
     }

static void nrf_qwr_error_handler(uint32_t nrf_error) {
    APP_ERROR_HANDLER(nrf_error);
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
  attr_char_value.init_len     = DATA_LENGTH_MAX;
  attr_char_value.init_offs    = 0;
  attr_char_value.max_len      = DATA_LENGTH_MAX;
  // attr_char_value.p_value      = p_memchunks[0];

  ble_uuid.type = m_bulkcli.uuid_type;
  ble_uuid.uuid = CLOCK_READ_UUID;

  sd_ble_gatts_characteristic_add(
              m_bulkcli.service_handle, 
	      &char_md,          // which connects to cccd_md
              &attr_char_value,  // which connects to attr_md 
              &m_bulkcli.clock_handle);
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
  attr_char_value.init_len     = sizeof(m_bulkcli.setclock);
  attr_char_value.init_offs    = 0;
  attr_char_value.max_len      = sizeof(m_bulkcli.setclock);
  attr_char_value.p_value      = (uint8_t*)&m_bulkcli.setclock;

  ble_uuid.type = m_bulkcli.uuid_type;
  ble_uuid.uuid = CLOCK_WRITE_UUID;

  sd_ble_gatts_characteristic_add(
              m_bulkcli.service_handle, 
	      &char_md,         
              &attr_char_value,  // which connects to attr_md 
              &m_bulkcli.setclock_handle);
  }

static void local_dis_add() {
  ret_code_t err_code;
  ble_dis_init_t dis_init;
  if (isDISset) return; 
  memset(&dis_init, 0, sizeof(dis_init));
  ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);
  dis_init.dis_char_rd_sec = SEC_OPEN;
  nrf_ble_qwr_init_t qwr_init = {0};
  qwr_init.error_handler = nrf_qwr_error_handler;
  err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
  APP_ERROR_CHECK(err_code);
  err_code = ble_dis_init(&dis_init);
  APP_ERROR_CHECK(err_code);
  isDISset = true;
  }

static void local_service_add() {
  ret_code_t err_code;
  ble_uuid_t ble_uuid;
  ble_uuid128_t base_uuid = {BLE_UUID_OUR_BASE_UUID};

  memset(&m_bulkcli,0,sizeof(m_bulkcli));
  // memchunk_index = 0;
  m_bulkcli.conn_handle = BLE_CONN_HANDLE_INVALID;   

  err_code = sd_ble_uuid_vs_add(&base_uuid, &m_bulkcli.uuid_type);
  APP_ERROR_CHECK(err_code);
  ble_uuid.type = m_bulkcli.uuid_type;
  ble_uuid.uuid = BLE_UUID_OUR_SERVICE;

  // this adds a primary service and sets a service handle 
  err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &m_bulkcli.service_handle);
  APP_ERROR_CHECK(err_code);
  }

static void bulkcli_init(void) {
  local_dis_add();
  local_service_add();
  local_clock_add();
  local_setclock_add();
  }

static void ble_stack_init(void)
{
    ret_code_t err_code;

    // Configure the BLE stack using the default settings.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
    NRF_SDH_SOC_OBSERVER(m_soc_observer, APP_SOC_OBSERVER_PRIO, soc_evt_handler, NULL);
}

void gatt_mtu_set(uint16_t att_mtu) {
  ret_code_t err_code;
  m_test_params.att_mtu = att_mtu;
  err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, att_mtu);
  APP_ERROR_CHECK(err_code);
  err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, att_mtu);
  APP_ERROR_CHECK(err_code);
  }
void connection_interval_set(uint16_t value) {
  m_test_params.conn_interval = value;
  }
void conn_evt_len_ext_set(bool status) {
  ret_code_t err_code;
  ble_opt_t  opt;
  memset(&opt, 0x00, sizeof(opt));
  opt.common_opt.conn_evt_ext.enable = status ? 1 : 0;
  err_code = sd_ble_opt_set(BLE_COMMON_OPT_CONN_EVT_EXT, &opt);
  APP_ERROR_CHECK(err_code);
  m_test_params.conn_evt_len_ext_enabled = status;
  }
void data_len_set(uint8_t value) {
  ret_code_t err_code;
  err_code = nrf_ble_gatt_data_length_set(&m_gatt, BLE_CONN_HANDLE_INVALID, value);
  APP_ERROR_CHECK(err_code);
  m_test_params.data_len = value;
  }

static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt) {
  switch (p_evt->evt_id) {
    case NRF_BLE_GATT_EVT_ATT_MTU_UPDATED: {
      // m_mtu_exchanged = true;
      NRF_LOG_INFO("ATT MTU exchange completed. MTU set to %u bytes.",
                         p_evt->params.att_mtu_effective);
      } break;

    case NRF_BLE_GATT_EVT_DATA_LENGTH_UPDATED: {
      // m_data_length_updated = true;
      NRF_LOG_INFO("Data length updated to %u bytes.", p_evt->params.data_length);
      m_bulkcli.fullyconnected = true;
      } break;

    // case BLE_GATTC_EVT_READ_RSP:                                 
    // case BLE_GATTC_EVT_CHAR_VALS_READ_RSP:
    // case BLE_GATTC_EVT_WRITE_RSP:
    // case BLE_GATTC_EVT_HVX:
    // case BLE_GATTC_EVT_EXCHANGE_MTU_RSP:
    // case BLE_GATTC_EVT_TIMEOUT:
    default:
      break;

    // here is where read/write stuff happens  
    // err_code = sd_ble_gattc_read(m_tx_buffer[m_tx_index].conn_handle,
         // m_tx_buffer[m_tx_index].req.read_handle,
         //  0);
    // err_code = sd_ble_gattc_write(m_tx_buffer[m_tx_index].conn_handle,
    //     &m_tx_buffer[m_tx_index].req.write_req.gattc_params);
    }
  }

void gatt_init(void) {
  ret_code_t err_code;

  err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
  APP_ERROR_CHECK(err_code);

  err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
  APP_ERROR_CHECK(err_code);
  }

void bsp_event_handler(bsp_event_t event) {
    switch (event) {

        case BSP_EVENT_SLEEP:
            nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
            break;

        case BSP_EVENT_DISCONNECT:
	    /*
            err_code = sd_ble_gap_disconnect(m_ble_nus_c.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
	    */
            break;

        default:
            break;
    }
}

// handler called to prematurely terminate BLE Scan and stop xfer 
void killscan(void * parameter, uint16_t size) {
    ret_code_t err_code;
    uint8_t rgb[3] = {0x00,0x00,0x00};
    nrf_ble_scan_stop();
    err_code = nrf_sdh_disable_request();
    APP_ERROR_CHECK(err_code);
    neopixel_write(rgb);
    xfer_active = false;
    mem_restore();
    main_xfer_end();
    }
void ble_scanwatch_handler(void * p_context) {
    app_sched_event_put(NULL,1,&killscan);
    }

void xfer(void) {
    ret_code_t err_code;
    uint8_t rgb[3] = {0x11,0x00,0x11};

    // shut down 802.15.4 and softdevice
    nrf_802154_deinit();
    err_code = nrf_sdh_disable_request();
    APP_ERROR_CHECK(err_code);
    nrf_clock_event_clear(NRF_CLOCK_EVENT_HFCLKSTARTED);
    nrf_drv_clock_init();

    // show user this section of code running
    neopixel_init();
    neopixel_write(rgb);
    mem_rewind();  // prepare to transfer data

    // at this point, only wdt is active and we
    // have several minutes (see RELOAD constant)
    // to scan and transfer, but using a timer 
    // all can be completed here
    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);
    nrfx_wdt_feed();

    // BLE Scan watch
    err_code = app_timer_create(&ble_scanwatch_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                ble_scanwatch_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(ble_scanwatch_id,
               APP_TIMER_TICKS(20000), NULL);
    APP_ERROR_CHECK(err_code);

    // Kickoff timer for HVX transfer
    err_code = app_timer_create(&kickoff_id,
                                APP_TIMER_MODE_REPEATED,
                                kickoff_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(kickoff_id,
               APP_TIMER_TICKS(10), NULL);
    APP_ERROR_CHECK(err_code);

    // Debugging Progress timer
    progressClock = 0;
    err_code = app_timer_create(&progress_id,
                                APP_TIMER_MODE_REPEATED,
                                progress_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(progress_id,
               APP_TIMER_TICKS(1000), NULL);
    APP_ERROR_CHECK(err_code);

    ble_stack_init();
    gatt_init();
    bulkcli_init();
    scan_init();

    gatt_mtu_set(m_test_params.att_mtu);
    conn_evt_len_ext_set(m_test_params.conn_evt_len_ext_enabled);
    data_len_set(DATA_LENGTH_MAX);

    scan_start();
    }
