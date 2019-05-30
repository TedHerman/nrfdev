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
#include "nrf_pwr_mgmt.h"
#include "nrf_fstorage.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE
#define SCHED_QUEUE_SIZE 10

#define APP_BLE_CONN_CFG_TAG    1                                       
#define APP_BLE_OBSERVER_PRIO   3
#define APP_SOC_OBSERVER_PRIO   1
#define SCAN_INTERVAL 180 // scan interval in units of 0.625 millisecond
#define SCAN_WINDOW 90    // scan window in units of 0.625 millisecond
#define SCAN_TIMEOUT 0  // 0 means continuous 

NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                        /**< Database discovery module instance. */
NRF_BLE_SCAN_DEF(m_scan);                                               /**< Scanning Module instance. */

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

typedef struct {
  uint8_t uuidseen[6];
  } uuidentry;
uuidentry uuidtable[200];
uint8_t numseen;

bool seenuuid(const uint8_t addr[6]) {
  bool r = false;
  for (int i=0; i<200; i++) {
     if (memcmp(addr,uuidtable[i].uuidseen,6)==0) return true;
     }
  return r;
  }
void recorduuid(const uint8_t addr[6]) {
  if (numseen >= 200) return;
  memcpy(uuidtable[numseen].uuidseen,addr,6);
  numseen++;
  }	

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
static void scan_start(void)
{
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


/*
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;

    switch(p_scan_evt->scan_evt_id)
    {
         case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
         {
              err_code = p_scan_evt->params.connecting_err.err_code;
              APP_ERROR_CHECK(err_code);
         } break;

         case NRF_BLE_SCAN_EVT_CONNECTED:
         {
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

         case NRF_BLE_SCAN_EVT_WHITELIST_ADV_REPORT: {

	      //ble_gap_evt_adv_report_t const * p_adv =
               //    p_scan_evt->params.filter_match.p_adv_report;
	      NRF_LOG_INFO("scan report");
		   // p_adv->peer_addr.addr[0],
		   // p_adv->peer_addr.type,
		   // p_adv->rssi);
              } break;

         case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
         {
             NRF_LOG_INFO("Scan timed out.");
             scan_start();
         } break;

         default:
             break;
    }
}
*/

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
static void scan_evt_handler(scan_evt_t const * p_scan_evt) {
    switch(p_scan_evt->scan_evt_id) {
        case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
        {
            NRF_LOG_INFO("Scan timed out.");
            // scan_start();
        } break;

        default:
          break;
    }
}

static void scan_init(void) {
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;
    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.connect_if_match = false;
    init_scan.conn_cfg_tag = APP_BLE_CONN_CFG_TAG;
    init_scan.p_scan_param = &m_scan_param; 
    // init_scan.p_scan_param = NULL;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);
    }


/**@brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function forwards the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    // ble_nus_c_on_db_disc_evt(&m_ble_nus_c, p_evt);
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

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {

        case BLE_GAP_EVT_CONNECTED:
            /*
            err_code = ble_nus_c_handles_assign(&m_ble_nus_c, p_ble_evt->evt.gap_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);

            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);

            err_code = ble_db_discovery_start(&m_db_disc, p_ble_evt->evt.gap_evt.conn_handle);
            APP_ERROR_CHECK(err_code);
	    */
            break;

        case BLE_GAP_EVT_DISCONNECTED:

            NRF_LOG_INFO("Disconnected. conn_handle: 0x%x, reason: 0x%x",
                         p_gap_evt->conn_handle,
                         p_gap_evt->params.disconnected.reason);
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_INFO("Connection Request timed out.");
            }
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported.
            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
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

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_ADV_REPORT: {
	    ble_gap_evt_adv_report_t const * p_adv = &p_gap_evt->params.adv_report;
	    ble_gap_scan_params_t    const * p_scan_param =
                   p_scan_evt->p_scan_params;
            ble_data_t inspectName;
	    ble_data_t inspectServ;
	    uint8_t expectServ[] = {0x0A, 0x18, 0x2C, 0x15};
	    if (seenuuid(p_adv->peer_addr.addr)) break;
	    recorduuid(p_adv->peer_addr.addr);
	    /*
	    NRF_LOG_INFO("scan %d",p_gap_evt->conn_handle);
            NRF_LOG_INFO("scanned %02x%02x%02x%02x%02x%02x",
		   p_adv->peer_addr.addr[0],
		   p_adv->peer_addr.addr[1],
		   p_adv->peer_addr.addr[2],
		   p_adv->peer_addr.addr[3],
		   p_adv->peer_addr.addr[4],
		   p_adv->peer_addr.addr[5])
            NRF_LOG_INFO("rssi %d",p_adv->rssi);
	    // NRF_LOG_RAW_HEXDUMP_INFO (m_scan.scan_buffer.p_data, m_scan.scan_buffer.len)
	    // see  https://www.bluetooth.com/specifications/assigned-numbers/generic-access-profile
	    // for a list of types, the first parameter to adv_report_parse, but then look at the
	    // hexdump from the buffer to be sure about what you are getting
            err_code = adv_report_parse(9,&m_scan.scan_buffer,&inspect);
	    if (err_code == NRF_SUCCESS) 
	       NRF_LOG_RAW_HEXDUMP_INFO (inspect.p_data, inspect.len)
            err_code = adv_report_parse(3,&m_scan.scan_buffer,&inspect);
	    if (err_code == NRF_SUCCESS) 
	       NRF_LOG_RAW_HEXDUMP_INFO (inspect.p_data, inspect.len)
            // inspect should be 4 bytes 0A182C15 which is backwards
	    // for 180A (device information service) + 152C (clock service)
	    */
	    err_code = adv_report_parse(9,&m_scan.scan_buffer,&inspectName);
	    err_code = err_code + adv_report_parse(3,&m_scan.scan_buffer,&inspectServ);
	    if (  err_code == NRF_SUCCESS 
               && inspectName.len == 11 
	       && (memcmp(inspectName.p_data,"Signal Labs",11) == 0)
	       && inspectServ.len == 4 
	       && (memcmp(inspectServ.p_data,expectServ,4) == 0) 
	       ) {
	       NRF_LOG_INFO("Found Signal Labs Server");
	       }
            } break;

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


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        NRF_LOG_INFO("ATT MTU exchange completed.");

        //m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        //NRF_LOG_INFO("Ble NUS max data length set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
}


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


/** @brief Function for initializing the database discovery module. */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}


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
    db_discovery_init();
    power_management_init();
    softdevice_init();
    ble_stack_init();
    gatt_init();
    scan_init();
    numseen = 0;

    // Start execution.
    NRF_LOG_INFO("central example started");
    scan_start();

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}
