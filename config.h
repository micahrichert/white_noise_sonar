constexpr unsigned int NR_OUTPUTS = 2;
constexpr unsigned int NR_INPUTS = 2;
constexpr unsigned int SAMPLE_RATE = 160000; // in Hz
constexpr unsigned int MERGE_BIN_CNT = 16;
constexpr bool USE_VARIANCE = true;

constexpr double MAX_DISTANCE = 10; // m
constexpr int SAMPLE_LAG = 0;  // must be a multiple of HIST_TYPE_BITS
constexpr double MAX_SPEED = 0.0;//1.2;  // m/s
constexpr double MIN_SPEED = 0.0;  // m/s
constexpr double SPEED_BANDWIDTH = 0.2;  // m/s, when a large target is stationary what is the fastest single motion channel that still detects it reasonably
constexpr double XCOR_DISPLAY_TIME = 0.1;  // s
constexpr double SPEED_OF_SOUND = 340.3/2.0;  // m/s, divide by 2 because sound travels out and back

constexpr bool USE_AVE_OUTPUT = true;  // minimizes DC of the output
