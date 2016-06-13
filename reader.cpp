#include <stdint.h>
#include <termio.h>
#include <fcntl.h>
#include <unistd.h>
#include <err.h>
#include <stdio.h>
#include <linux/serial.h>
#include <sys/time.h>
#include <math.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "config.h"

using namespace cv;

#define USE_BUILTIN 1  // slow on stm32 but may be fast on other chips
#define USE_PARALLEL_BITS 0  // slower than lookup but doesn't use any memory
#define USE_LOOKUP 0  // fastest solution on stm32 but uses 64K of memory
#include "popcount.cpp"


const float MAX_DISTANCE = 1.5;//1.5; // m
const int SAMPLE_LAG = 0;  // must be a multiple of HIST_TYPE_BITS
const unsigned int MERGE_BIN_CNT = 8;
const float MAX_SPEED = 1.2;  // m/s
const float MIN_SPEED = 0.0;  // m/s
const float SPEED_BANDWIDTH = 0.2;  // m/s, when a large target is stationary what is the fastest single motion channel that still detects it reasonably
const unsigned int NR_SPEEDS = ceil((MAX_SPEED - MIN_SPEED) / (SPEED_BANDWIDTH * 2));
const float SPEED_STEP = (MAX_SPEED - MIN_SPEED) / (NR_SPEEDS * 2);
const float XCOR_DECAY_TIME = 0.1;//0.10;//0.05;//1.0; // s
const float SPEED_OF_SOUND = 340.3/2.0; // m/s, divide by 2 because sound travels out and back

// dependant variables
const float DISTANCE_PER_SAMPLE = SPEED_OF_SOUND/SAMPLE_RATE;
const float ACTUAL_OUTPUT_BIN_DISTANCE = MERGE_BIN_CNT*DISTANCE_PER_SAMPLE; // m, see DESIRED_OUTPUT_BIN_DISTANCE
const unsigned int NR_BINS = (int(MAX_DISTANCE/DISTANCE_PER_SAMPLE)+31)/32*32; // should be multiple of 32, round up //int(MAX_DISTANCE/DISTANCE_PER_SAMPLE)/MERGE_BIN_CNT*MERGE_BIN_CNT; // make sure NR_BINS is a multple of merge
const int XCOR_DECAY_SAMPLES = SAMPLE_RATE * XCOR_DECAY_TIME;
const float XCOR_DECAY_RATE = 1.0 - 1.0/XCOR_DECAY_SAMPLES;
const float MAX_SPEED_DILATION = (MAX_SPEED-SPEED_STEP)/DISTANCE_PER_SAMPLE/SAMPLE_RATE;
const float SPEED_DILATION_STEP = SPEED_STEP/DISTANCE_PER_SAMPLE/SAMPLE_RATE;
const unsigned int EXTRA_DILATION_BINS = int(MAX_SPEED_DILATION*XCOR_DECAY_SAMPLES);
const unsigned int HISTORY_LEN = NR_BINS/32 + (SAMPLE_LAG+EXTRA_DILATION_BINS+31)/32 + 1;

#define HIST_TYPE uint32_t
#define HIST_TYPE_BITS (sizeof(HIST_TYPE)*8)

const bool USE_VARIANCE = true;


static int rate_to_constant(int baudrate) {
#define B(x) case x: return B##x
	switch(baudrate) {
		B(50);     B(75);     B(110);    B(134);    B(150);
		B(200);    B(300);    B(600);    B(1200);   B(1800);
		B(2400);   B(4800);   B(9600);   B(19200);  B(38400);
		B(57600);  B(115200); B(230400); B(460800); B(500000); 
		B(576000); B(921600); B(1000000);B(1152000);B(1500000);
	default: return 0;
	}
#undef B
}    

/* Open serial port in raw mode, with custom baudrate if necessary */
int serial_open(const char *device, int rate)
{
	struct termios options;
	struct serial_struct serinfo;
	int fd;
	int speed = 0;

	/* Open and configure serial port */
	if ((fd = open(device,O_RDWR|O_NOCTTY)) == -1)
		return -1;

	speed = rate_to_constant(rate);

	if (speed == 0) {
		/* Custom divisor */
		serinfo.reserved_char[0] = 0;
		if (ioctl(fd, TIOCGSERIAL, &serinfo) < 0)
			return -1;
		serinfo.flags &= ~ASYNC_SPD_MASK;
		serinfo.flags |= ASYNC_SPD_CUST;
		serinfo.custom_divisor = (serinfo.baud_base + (rate / 2)) / rate;
printf("%d %d %d\n",serinfo.baud_base, rate, serinfo.custom_divisor);
		if (serinfo.custom_divisor < 1) 
			serinfo.custom_divisor = 1;
		ioctl(fd, TIOCSSERIAL, &serinfo);
		if (ioctl(fd, TIOCGSERIAL, &serinfo) < 0)
			return -1;
/*		if (serinfo.custom_divisor * rate != serinfo.baud_base) {
			printf("actual baudrate is %d / %d = %f",
			      serinfo.baud_base, serinfo.custom_divisor,
			      (float)serinfo.baud_base / serinfo.custom_divisor);
		}
*/	}

	fcntl(fd, F_SETFL, 0);
	tcgetattr(fd, &options);
	cfsetispeed(&options, speed ? speed: B38400);
	cfsetospeed(&options, speed ? speed: B38400);
	cfmakeraw(&options);
	options.c_cflag |= (CLOCAL | CREAD);
	options.c_oflag &= ~OPOST;
	options.c_cflag &= ~CRTSCTS;
    options.c_cflag &= ~CSIZE; /* Mask the character size bits */
    options.c_cflag |= CS8;    /* Select 8 data bits */
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	if (tcsetattr(fd, TCSANOW, &options) != 0)
		return -1;

	return fd;
}

int read_byte(int fd)
{
    uint8_t c;
    if (read(fd, &c, 1)) return c;
    else return -1;
}

void read_N_bytes(int fd, void* dst, int cnt)
{
    int bytes_read = 0;
    
    while (bytes_read != cnt)
    {
        bytes_read += read(fd, ((uint8_t*)dst)+bytes_read, cnt-bytes_read);
    }
}

void find_magic(int fd)
{
    uint8_t c;
    while ((c = read_byte(fd)) != 0xA1) ;// printf("read 0x%x\n",c);
}


typedef struct
{
    uint8_t magic;
    uint8_t count;
    uint8_t nr_inputs;
    uint8_t nr_outputs;
    uint32_t inputs[NR_INPUTS];
    uint32_t outputs[NR_OUTPUTS];
} __attribute__((packed)) packet_t;

int read_packet(int fd, packet_t* p)
{
    find_magic(fd);
    
    p->count = read_byte(fd);
    p->nr_inputs = read_byte(fd);
    p->nr_outputs = read_byte(fd);
    
    if (p->nr_inputs*p->nr_outputs == 0 || p->nr_inputs > NR_INPUTS || p->nr_outputs > NR_OUTPUTS)
    {
        printf("# inputs or outputs out of range\n");
        return -1;
    }
    
    read_N_bytes(fd, p->inputs, p->nr_inputs*sizeof(uint32_t));
    read_N_bytes(fd, p->outputs, p->nr_outputs*sizeof(uint32_t));
    
    return 0;
}


inline void insert_bit(HIST_TYPE* buf, HIST_TYPE bit, int16_t loc)
{
    int i = loc / HIST_TYPE_BITS;
    int offset = loc % HIST_TYPE_BITS;
    buf[i] = (buf[i]&(~(HIST_TYPE(1)<<offset))) | (bit << offset);
}
    
int main()
{
    int fd = serial_open("/dev/ttyUSB0", 1500000);
    int cnt = 0, t = 0;
    HIST_TYPE rand_hist[NR_OUTPUTS][HISTORY_LEN] = {0};
    HIST_TYPE input[NR_INPUTS] = {0};
    int previous_input[NR_INPUTS] = {0};
    float dilations[NR_SPEEDS] = {0};
    int bit_pos = 0;
    int hist_i;
    int xcor_i[NR_SPEEDS];
    int decay_i = 0;
    int r;
    bool do_reset = true;
    int reset_cnt = 0;
    struct timeval start_time, curr_time;
    float merged_xcors[NR_INPUTS][NR_OUTPUTS][NR_BINS/MERGE_BIN_CNT] = {0};
    float merged_xcors_hist[100][NR_BINS/MERGE_BIN_CNT] = {0};
    float scaled_merged_xcors_hist[100][NR_BINS/MERGE_BIN_CNT] = {0};
    int32_t xcors[NR_SPEEDS][NR_BINS+EXTRA_DILATION_BINS][NR_INPUTS][NR_OUTPUTS] = {0};
    packet_t p;
    uint8_t prev_packet_cnt = 0;
    
    popcount_setup();

    gettimeofday(&start_time, NULL);

    if (fd == -1)
    {
        printf("Unable to open serial port\n");
        return -1;
    }
    
    read_packet(fd, &p);
    prev_packet_cnt = p.count;

    while (1)
    {
        read_packet(fd, &p);
        if (uint8_t(prev_packet_cnt+1) != p.count) printf("dropped packet %d %d\n",prev_packet_cnt, p.count);
        prev_packet_cnt = p.count;
        
        cnt += 1;
        for (int i = -1; i<31; i++)
        {
            unsigned int calc_phase = i & (HIST_TYPE_BITS-1); // & with 31 to ensure it is always between 0 and 31, even when negative; % keeps sign.

            for (uint8_t dpin=0; dpin<p.nr_outputs; dpin++)
            {
                bool r = p.outputs[dpin] & (1UL<<31);
                p.outputs[dpin] <<= 1;
                insert_bit(rand_hist[dpin], r, bit_pos);
            }

            hist_i = (bit_pos+(HIST_TYPE_BITS-1)+1+SAMPLE_LAG)/HIST_TYPE_BITS; // the 32bits we want start 31 bits back (positive values are back in time)
            for (int s=0; s<NR_SPEEDS;s++) xcor_i[s] = calc_phase + dilations[s];
            do
            {
                if (hist_i>=HISTORY_LEN) hist_i-=HISTORY_LEN;
                for (uint8_t apin=0; apin<p.nr_inputs; apin++)
                {
                    HIST_TYPE input_apin = input[apin];
                    for (uint8_t dpin=0; dpin<p.nr_outputs; dpin++)
                    {
                        HIST_TYPE shift_hist = rand_hist[dpin][hist_i];
                        int xcor = popcount(shift_hist ^ input_apin)*2 - HIST_TYPE_BITS; // count the number of 1s in the 32 bits *2 (for unit variance) and subtract the mean
                        
                        //compute actual xcross-correlation
                        for (int s=0; s<NR_SPEEDS;s++) xcors[s][xcor_i[s]][apin][dpin] += xcor;
                    }
                }
                
                for (int s=0; s<NR_SPEEDS;s++) xcor_i[s] += HIST_TYPE_BITS;
                hist_i++;
            } while (xcor_i[0]<NR_BINS);

            for (int s=0; s<NR_SPEEDS;s++)
            {
                dilations[s] += SPEED_DILATION_STEP*(s*2+1) + MIN_SPEED;
            }

            t += 1;
            if (decay_i == 0)
            {
                if (t >= XCOR_DECAY_SAMPLES)
                {
                    t = 0;
                    for (int s=0; s<NR_SPEEDS;s++) dilations[s] = 0;
                    do_reset = true;
                    reset_cnt += 1;

                    gettimeofday(&curr_time, NULL);

                    float timediff = curr_time.tv_sec - start_time.tv_sec + (curr_time.tv_usec - start_time.tv_usec)/1000000.0;
                    printf("read %d packets in %f at %f\n", cnt, timediff, 32*cnt/timediff);

                    memcpy(merged_xcors_hist, &merged_xcors_hist[1][0], sizeof(merged_xcors_hist)*99/100);
                    memcpy(&merged_xcors_hist[99][0], &merged_xcors[0][0][0], sizeof(merged_xcors[0][0]));

                    float max = 0, min=0;
                    for (int k=0;k<100;k++)
                    {
                        for (int j=0;j<NR_BINS/MERGE_BIN_CNT;j++)
                        {
                            if (merged_xcors_hist[k][j] > max) max = merged_xcors_hist[k][j];
                            if (merged_xcors_hist[k][j] < min) min = merged_xcors_hist[k][j];
                        }
                    }
                    
                    for (int k=0;k<100;k++)
                    {
                        for (int j=0;j<NR_BINS/MERGE_BIN_CNT;j++)
                        {
                            scaled_merged_xcors_hist[k][j] = (merged_xcors_hist[k][j] - min)/(max-min);
                        }
                    }
                    
                    // display data merged_xcors data
//                    for (uint8_t dpin=0; dpin<p.nr_outputs; dpin++)
                    {
//                        for (uint8_t apin=0; apin<p.nr_inputs; apin++)
                        {
                            char title[100];
                            sprintf(title, "merged xcors %d %d", 0, 0);//apin, dpin);
                            imshow(title , Mat(100, NR_BINS/MERGE_BIN_CNT, CV_32FC1, &scaled_merged_xcors_hist));
                        }
                    }
                    
                    waitKey(1);
                } else {
                    do_reset = false;                
                }
            }

            // gets run once every NR_BINS
            for (uint8_t dpin=0; dpin<p.nr_outputs; dpin++)
            {
                for (uint8_t apin=0; apin<p.nr_inputs; apin++)
                {
                    if (USE_VARIANCE)
                    {
                        bool reset = do_reset && ((decay_i % MERGE_BIN_CNT) == 0);
                        
                        if (reset) merged_xcors[apin][dpin][decay_i/MERGE_BIN_CNT] = xcors[0][decay_i][apin][dpin]*xcors[0][decay_i][apin][dpin];// - (int)NR_BINS;

                        for (int s=reset; s<NR_SPEEDS;s++)
                            merged_xcors[apin][dpin][decay_i/MERGE_BIN_CNT] += xcors[s][decay_i][apin][dpin]*xcors[s][decay_i][apin][dpin];// - (int)NR_BINS;
      
                        for (int s=0; s<NR_SPEEDS;s++) xcors[s][decay_i][apin][dpin] = 0;
                    } else {
                        bool reset = do_reset && ((decay_i % MERGE_BIN_CNT) == 0);
                        
                        if (reset) merged_xcors[apin][dpin][decay_i/MERGE_BIN_CNT] = xcors[0][decay_i][apin][dpin];

                        for (int s=reset; s<NR_SPEEDS;s++)
                            merged_xcors[apin][dpin][decay_i/MERGE_BIN_CNT] += xcors[s][decay_i][apin][dpin];

                        for (int s=0; s<NR_SPEEDS;s++) xcors[s][decay_i][apin][dpin] = 0;
                    }
                }
            }
            decay_i++;
            if (decay_i >= NR_BINS) decay_i = 0;

            bit_pos--;
            if (bit_pos<0) bit_pos += HISTORY_LEN*HIST_TYPE_BITS;
            
            // read input
            for (uint8_t apin=0; apin<p.nr_inputs; apin++)
            {
                bool r = p.inputs[apin] & (1UL<<31);
                p.inputs[apin] <<= 1;

                input[apin] <<= 1;
                input[apin] |= r;
            }
        }
    }
}
