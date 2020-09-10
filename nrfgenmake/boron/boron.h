// Boron state structure
typedef struct {
  uint8_t fence[4];  // should be aabbccdd
  uint32_t clock;
  bool isdst;     // true if current clock is on daylight savings
  bool connected; // true if Particle.connected() 
  bool capped;    // true if too many BPS readings in current day 
  bool clinic;    // true if clinic should be called ASAP
  } boronstate_t;
// The structure that Boron expects for I2C write
typedef struct {
  uint8_t fence[4];  // should be 55aaff77
  uint8_t status[4]; // LED code, battery, waiting readings, etc
  uint32_t clock;
  uint8_t systolic;
  uint8_t diastolic;
  uint8_t pulse;
  } bps_reading_t;
