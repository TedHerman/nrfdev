#define CHANNEL 26 // for 802.15.4 radio
#define MAX_MESSAGE_SIZE 37 
#define PACKET_MAX_PAYLOAD 26 
#define PACKET_POOL_SIZE 256  // power of 2 helpful for heapsort

/*************************************************************************
 * TinyOS Header and Message Format (compatible with IEEE 802.15.4)      *
 ************************************************************************/
typedef uint16_t mac_addr_t;
struct motepacket {
  uint8_t length;
  uint16_t fcf;
  uint8_t data_seq_no;
  mac_addr_t pan;
  mac_addr_t dest;
  mac_addr_t src;
  uint8_t iframe;    // field for 6lowpan, is 0x3f for TinyOS
  uint8_t type;
  uint8_t data[PACKET_MAX_PAYLOAD];
  uint8_t fcs[2];    // will become CRC, don't count this in length.
  };
typedef struct motepacket motepacket_t;

/*************************************************************************
 * The format of the payload for a badge probe (also telling any room    *
 * device what other badges were recently seen and with what rssi level) *
 ************************************************************************/
struct mote_received {  // there will be N of these structs in payload
  uint8_t mote_id;   // id of other mote
  int8_t rssi;       // rssi value observed 
  };
typedef struct mote_received mote_received_t;

struct mote_report {
  uint32_t time;     // timestamp of this report
  uint8_t mote_id;   // id of reporting mote
  uint8_t num_reports;  // how many reports we have
  mote_received_t reports[6];  // six highest rssi observations 
  };
typedef struct mote_report mote_report_t;
