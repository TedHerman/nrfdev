// Boron state structure
typedef struct {
  uint8_t fence[4];  // should be aabbccdd
  uint32_t clock;
  uint32_t lastpost; // timestamp of last reading sent to cloud
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
