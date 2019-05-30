#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "app_timer.h"
#include "app_scheduler.h"
#include "app_error.h"
#include "ble_db_discovery.h"
#include "app_util.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_ble_scan.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_fstorage.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define SERVICE_UUID_BASE   {0xBB, 0x4A, 0xFF, 0x4F, 0xAD, 0x03, 0x41, 0x5D, \
                             0xA9, 0x6C, 0x9D, 0x6C, 0xDD, 0xDA, 0x83, 0x04}

#define BLE_UUID_OUR_BASE_UUID {0x23, 0xD1, 0x13, 0xEF, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}
#define BLE_UUID_OUR_SERVICE 0x152C
#define CLOCK_READ_UUID 0x3907
#define CLOCK_READ_INDX 0
#define CLOCK_WRITE_UUID 0x3909
#define CLOCK_WRITE_INDX 1
#define NUMBER_OUR_ATTRIBUTES 2



#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE
#define SCHED_QUEUE_SIZE 10

#define APP_BLE_CONN_CFG_TAG    1                                       
#define APP_BLE_OBSERVER_PRIO   3
#define APP_SOC_OBSERVER_PRIO   1
#define SCAN_INTERVAL 180 // scan interval in units of 0.625 millisecond
#define SCAN_WINDOW 90    // scan window in units of 0.625 millisecond
#define SCAN_TIMEOUT 0    // 0 means continuous 
#define DATA_LENGTH_MAX 251

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
  .phys.tx_phys = BLE_GAP_PHY_2MBPS | BLE_GAP_PHY_1MBPS | BLE_GAP_PHY_CODED,
  .phys.rx_phys = BLE_GAP_PHY_2MBPS | BLE_GAP_PHY_1MBPS | BLE_GAP_PHY_CODED
   };

typedef struct clockread_s clockread_t;
struct clockread_s {
  ble_uuid_t service_uuid;
  uint16_t clockread_handle;  
  uint16_t clockread_cccd_handle; 
  uint16_t conn_handle;
  uint16_t conn_handle_disc;
  uint16_t clockwrite_handle;  
  uint8_t uuid_type;
  uint32_t clock;
  uint32_t setclock;
  bool notificationset;
  };
static clockread_t m_clockread;

NRF_BLE_QWR_DEF(m_qwr); 
NRF_BLE_SCAN_DEF(m_scan);
NRF_BLE_GATT_DEF(m_gatt);
BLE_DB_DISCOVERY_DEF(m_db_disc);
APP_TIMER_DEF(reader_timer_id);

static char const m_target_periph_name[] = "Signal Labs";

static bool  m_memory_access_in_progress;

// ref: http://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.sdk5.v15.2.0%2Fgroup__nrf__ble__scan.html
// found in components/softdevice/s140/headers/ble_gap.h
static ble_gap_scan_params_t m_scan_param = {
  .extended      = 1,
  .report_incomplete_evts = 0,
  .active        = 1,
  .filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL,
  .scan_phys     = BLE_GAP_PHY_1MBPS, 
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

void rcdump(void * parameter, uint16_t size);
void reader_timeout_handler(void * p_context);

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


/**@brief Function for handling asserts in the SoftDevice.
 *
 * @details This function is called in case of an assert in the SoftDevice.
 *
 * @warning This handler is only an example and is not meant for the final product. You need to analyze
 *          how your product is supposed to react in case of assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing assert call.
 * @param[in] p_file_name  File name of the failing assert call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
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
  if (m_clockread.conn_handle == BLE_CONN_HANDLE_INVALID) {
    NRF_LOG_INFO("on ble gap evt at time of connect");
    }

  m_clockread.conn_handle = p_gap_evt->conn_handle;
  nrf_ble_scan_stop();
  err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr,m_clockread.conn_handle);
  APP_ERROR_CHECK(err_code);

  NRF_LOG_INFO("Discovering GATT database... %d",m_clockread.conn_handle);
  err_code  = ble_db_discovery_start(&m_db_disc,m_clockread.conn_handle);
  APP_ERROR_CHECK(err_code);
  }

static void scan_init(void) {
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;
    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.connect_if_match = false;
    init_scan.conn_cfg_tag = APP_BLE_CONN_CFG_TAG;
    init_scan.p_scan_param = &m_scan_param; // unclear we need scan_param 
    // init_scan.p_scan_param = NULL;
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

static void finish_connect(ble_db_discovery_evt_t * p_evt) {
  ret_code_t err_code;
  // ble_gatts_value_t gatts_val;
  uint16_t cccd_valuebase = BLE_GATT_HVX_NOTIFICATION;
  uint8_t cccd_value[BLE_CCCD_VALUE_LEN]; 
  ble_gattc_write_params_t params;
  memset(&params, 0, sizeof(params));
  params.handle = m_clockread.clockread_cccd_handle;
  params.len = BLE_CCCD_VALUE_LEN;
  params.p_value = cccd_value; 
  params.offset   = 0;
  params.write_op = BLE_GATT_OP_WRITE_REQ; // or maybe WRITE_CMD ??
  cccd_value[0] = LSB_16(cccd_valuebase);
  cccd_value[1] = MSB_16(cccd_valuebase);
  err_code = sd_ble_gattc_write(m_clockread.conn_handle,&params);
  APP_ERROR_CHECK(err_code);
  /*
  err_code = sd_ble_gatts_value_get(p_ble_evt->evt.gap_evt.conn_handle,
             m_clockread.clockread_cccd_handle,
             &gatts_val);
  if (err_code == NRF_SUCCESS) bsp_board_led_invert(3);
  */
  err_code = app_timer_create(&reader_timer_id,
              APP_TIMER_MODE_SINGLE_SHOT,
              reader_timeout_handler);
  APP_ERROR_CHECK(err_code);
  err_code = app_timer_start(reader_timer_id,APP_TIMER_TICKS(1000), NULL);
  APP_ERROR_CHECK(err_code);
  }

static void db_disc_handler(ble_db_discovery_evt_t * p_evt) {
  // ref: ble_db_discovery.h for type of p_evt 
  // Check if the clock service was discovered.
  if (p_evt->evt_type != BLE_DB_DISCOVERY_COMPLETE) return;
  if (p_evt->params.discovered_db.srv_uuid.uuid != BLE_UUID_OUR_SERVICE) return;
  m_clockread.conn_handle_disc = p_evt->conn_handle;
  // find characteristic handles 
  for (uint32_t i = 0; i < p_evt->params.discovered_db.char_count; i++) {
    ble_uuid_t uuid = p_evt->params.discovered_db.charateristics[i].characteristic.uuid;
    if (uuid.uuid == CLOCK_READ_UUID) {
       m_clockread.clockread_handle = 
	    p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
       m_clockread.clockread_cccd_handle = 
            p_evt->params.discovered_db.charateristics[i].cccd_handle;
       }
    if (uuid.uuid == CLOCK_WRITE_UUID) {
       m_clockread.clockwrite_handle = 
	    p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
       }
    }
  // after loop -- schedule things to start here
  if (    m_clockread.clockread_handle != BLE_GATT_HANDLE_INVALID 
       && m_clockread.clockwrite_handle != BLE_GATT_HANDLE_INVALID
       && m_clockread.clockread_cccd_handle != BLE_GATT_HANDLE_INVALID ) 
	  finish_connect(p_evt);
  }

/*
static bool shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
            // Prepare wakeup buttons.
            err_code = bsp_btn_ble_sleep_mode_prepare();
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }

    return true;
}
*/

// NRF_PWR_MGMT_HANDLER_REGISTER(shutdown_handler, APP_SHUTDOWN_HANDLER_PRIORITY);
void on_hvx(ble_evt_t const * p_ble_evt) {
  bsp_board_led_on(3);
  if (p_ble_evt->evt.gattc_evt.conn_handle != m_clockread.conn_handle) return;
  if (p_ble_evt->evt.gattc_evt.params.hvx.handle == m_clockread.clockread_handle) { 
    uint32_t len = p_ble_evt->evt.gattc_evt.params.hvx.len;
    UNUSED_VARIABLE(len);
    uint32_t data = uint32_decode(p_ble_evt->evt.gattc_evt.params.hvx.data);
    m_clockread.clock = data;
    }
  }	

static void on_write_response(ble_evt_t const * p_ble_evt) {
  if (m_clockread.conn_handle != p_ble_evt->evt.gattc_evt.conn_handle) return;
  if (p_ble_evt->evt.gattc_evt.params.write_rsp.handle == m_clockread.clockread_cccd_handle) {
     m_clockread.notificationset = true;
     }
  }

static void on_read_response(ble_evt_t const * p_ble_evt) {
  NRF_LOG_DEBUG("read response");
  }

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context) {
    ret_code_t err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id) {

        case BLE_GAP_EVT_CONNECTED:
            on_ble_gap_evt_connected(p_gap_evt);
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
            // Pairing not supported.
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
            ble_gap_evt_phy_update_t const * p_phy_evt = &p_ble_evt->evt.gap_evt.params.phy_update;
            if (p_phy_evt->status == BLE_HCI_STATUS_CODE_LMP_ERROR_TRANSACTION_COLLISION) {
                // Ignore LL collisions.
                NRF_LOG_DEBUG("LL transaction collision during PHY update.");
                } 
	    } break;

	case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
            err_code = sd_ble_gap_phy_update(p_gap_evt->conn_handle, &m_test_params.phys);
            APP_ERROR_CHECK(err_code);
            } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_ADV_REPORT: {
	    ble_gap_evt_adv_report_t const * p_adv = &p_gap_evt->params.adv_report;
	    UNUSED_VARIABLE(p_adv);
            ble_data_t inspectName;
	    ble_data_t inspectServ;
	    uint8_t expectServ[] = {0x0A, 0x18};
            err_code = adv_report_parse(9,&m_scan.scan_buffer,&inspectName);
	    if (err_code == NRF_SUCCESS) 
	       NRF_LOG_RAW_HEXDUMP_INFO (inspectName.p_data, inspectName.len)
            err_code = adv_report_parse(3,&m_scan.scan_buffer,&inspectServ);
	    if (err_code == NRF_SUCCESS) 
	       NRF_LOG_RAW_HEXDUMP_INFO (inspectServ.p_data, inspectServ.len)
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

        case BLE_GATTC_EVT_HVX: 
            on_hvx(p_ble_evt);
            break;
	 
        case BLE_GATTC_EVT_WRITE_RSP:
            on_write_response(p_ble_evt);
            break;

        case BLE_GATTC_EVT_READ_RSP:
            on_read_response(p_ble_evt);
            break;

        default:
            break;
        }
     }

static void softdevice_init() {
  ret_code_t err_code;
  err_code = nrf_sdh_enable_request();
  APP_ERROR_CHECK(err_code);
  }

static void ble_stack_init(void)
{
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

/*
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        NRF_LOG_INFO("ATT MTU exchange completed.");

        //m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        //NRF_LOG_INFO("Ble NUS max data length set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
}
*/


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the BSP module.
 *
 * @param[in] event  Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    switch (event)
    {
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

/**@brief Function for initializing buttons and leds. */
static void buttons_leds_init(void)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the timer. */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the nrf log module. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


static void nrf_qwr_error_handler(uint32_t nrf_error) {
  APP_ERROR_HANDLER(nrf_error);
  }

static void clockread_init(void) {
  ret_code_t err_code;
  ble_uuid_t dis_uuid;
  ble_uuid_t serv_uuid;
  ble_uuid128_t base_uuid = {BLE_UUID_OUR_BASE_UUID};
  memset(&m_clockread, 0, sizeof(m_clockread));
  m_clockread.conn_handle = BLE_CONN_HANDLE_INVALID;
  m_clockread.conn_handle_disc = BLE_CONN_HANDLE_INVALID;
  m_clockread.clockread_handle = BLE_GATT_HANDLE_INVALID; 
  m_clockread.clockwrite_handle = BLE_GATT_HANDLE_INVALID; 
  m_clockread.clockread_cccd_handle = BLE_GATT_HANDLE_INVALID;
  nrf_ble_qwr_init_t qwr_init = {0};
  err_code = sd_ble_uuid_vs_add(&base_uuid, &m_clockread.uuid_type);
  APP_ERROR_CHECK(err_code);
  serv_uuid.uuid = BLE_UUID_OUR_SERVICE;
  serv_uuid.type = m_clockread.uuid_type;
  dis_uuid.uuid = BLE_UUID_DEVICE_INFORMATION_SERVICE;
  dis_uuid.type = BLE_UUID_TYPE_BLE;

  qwr_init.error_handler = nrf_qwr_error_handler;
  err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
  APP_ERROR_CHECK(err_code);
  // set up discovery handler
  err_code = ble_db_discovery_init(db_disc_handler);
  APP_ERROR_CHECK(err_code);
  // say which services we want to discover
  NRF_LOG_INFO("register for uuid %d type %d",serv_uuid.uuid, serv_uuid.type);
  err_code = ble_db_discovery_evt_register(&serv_uuid); 
  APP_ERROR_CHECK(err_code);
  NRF_LOG_INFO("register for uuid %d type %d",dis_uuid.uuid, dis_uuid.type);
  err_code = ble_db_discovery_evt_register(&dis_uuid); 
  APP_ERROR_CHECK(err_code);
  }

void readstart(void * parameter, uint16_t size) {
  ret_code_t err_code;
  if (m_clockread.notificationset) {
      /*
      ble_gatts_value_t rdesc = {
      .len = 4,
      .offset = 0,
      .p_value = (uint8_t *)&m_clockread.clock
      };
      uint32_t xferd = sd_ble_gatts_value_get(
		     m_clockread.conn_handle,
                     m_clockread.clockread_handle,  
		     &rdesc);
      UNUSED_VARIABLE(xferd);
      */
      NRF_LOG_INFO("clock value is %d",m_clockread.clock); 
      }
  err_code = app_timer_start(reader_timer_id,APP_TIMER_TICKS(1000), NULL);
  APP_ERROR_CHECK(err_code);
  }

void reader_timeout_handler(void * p_context) {
  app_sched_event_put(NULL,1,&readstart);
  }

/*
app_sched_event_put(NULL,1,&rcdump);
void rcdump(void * parameter, uint16_t size) {
  NRF_LOG_INFO("disc uuids found = %d %d %d %d",
    m_clockread.uid1,m_clockread.uid2,m_clockread.uid3,m_clockread.uid4);
  }
*/

/**@brief Function for handling the idle state (main loop).
 *
 * @details Handles any pending log operations, then sleeps until the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


int main(void)
{
    // Initialize.
    log_init();
    timer_init();
    buttons_leds_init();
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
    power_management_init();
    softdevice_init();
    ble_stack_init();
    gatt_init();
    scan_init();
    clockread_init();

    gatt_mtu_set(m_test_params.att_mtu);
    conn_evt_len_ext_set(m_test_params.conn_evt_len_ext_enabled);
    data_len_set(DATA_LENGTH_MAX);

    // Start execution.
    NRF_LOG_INFO("central example started");
    scan_start();

    // Enter main loop.
    for (;;)
    {
	bsp_board_led_invert(1);
        idle_state_handle();
        app_sched_execute();
    }
}
