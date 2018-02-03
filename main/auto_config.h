
#define BASE_TRIM 0
#define BASE_L1_TIME 4500
#define BASE_L1L2_TURN_TIME 120
#define BASE_L2_TIME 3000
#define BASE_L2L3_TURN_TIME 250
#define BASE_L3_TIME 2500

#define STRAIGHT_SPEED 200
#define LEFT_TURN_VAL 234
#define RIGHT_TURN_VAL (20)
enum AutoState {
  START,
  L1,
  L1L2,
  L2,
  L2L3,
  L3
};

const long base_auto_values[6] = {
  BASE_TRIM,
  BASE_L1_TIME,
  BASE_L1L2_TURN_TIME,
  BASE_L2_TIME,
  BASE_L2L3_TURN_TIME,
  BASE_L3_TIME
};

long auto_values[6] = {
  BASE_TRIM,
  BASE_L1_TIME,
  BASE_L1L2_TURN_TIME,
  BASE_L2_TIME,
  BASE_L2L3_TURN_TIME,
  BASE_L3_TIME
};

AutoState auto_state = START;
