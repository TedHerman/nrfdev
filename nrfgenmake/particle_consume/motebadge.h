#define MAX_MESSAGE_SIZE 37 
#define PACKET_MAX_PAYLOAD 26 
#define DATA_LENGTH_MAX 244   // empirically found max size on BLE send
#define MEMCHUNKS_POOL 720    // total memory just under 170kB
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


/*************************************************************************
 * the MemChunks Pool is in the data section, see s140.ld -- it should   *
 * have the size for the memchunk_t structure defined here               *
 ************************************************************************/
typedef struct {
    uint8_t prefence[4];  // will be set to "fnce"
    uint8_t batch_offset; // index into current batch of memchunk_index
    bool empty;           // true iff there is no data
    uint16_t cursor;      // range is 0 to MEMCHUNKS_POOL
    uint16_t high_water;  // mirror of memchunk_index when rewind/iterate
    uint8_t area[DATA_LENGTH_MAX*MEMCHUNKS_POOL];
    uint8_t postfence[4];  // will be set to "fnce"
    } memchunk_t;
struct batch {
  uint8_t time[4]; // timestamp of this batch
  uint8_t num_reports; // number of reports that follow
  uint8_t reports[2];  // each report is 2 bytes, number of reports varies
  };
typedef struct batch batch_t;
struct batch_digest {
  uint16_t record_number; // number the records for debugging / correcting 
  uint8_t id;  // id of this badge who is sending a digest
  uint8_t num_reports;  // how many batches we have
  batch_t batches[1];   // variable number of batches	 
  };
typedef struct batch_digest batch_digest_t;
// static uint16_t memchunk_index;  // current batch in memchunk_area
// static uint8_t batch_index;      // current place in batch 
