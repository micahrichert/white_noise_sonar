#include <stdint.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/f4/rng.h>
#include <libopencm3/stm32/adc.h>

#include "serial_printf.cpp"
#include "clock.cpp"

/*
Chi2 significance threshold table generated with the following python code:

import scipy.stats
import numpy as np
cum_p_thresh = scipy.stats.norm.cdf(3.0)  # 3.0 sigmas (1 in 370 false positive) works pretty well
thresh = scipy.stats.chi2.ppf(cum_p_thresh, np.arange(1, 100))
np.set_printoptions(precision=2)
print repr(thresh)

*/
const float chi2_threshs[100] = {  10.27,   13.22,   15.63,   17.8 ,   19.82,   21.74,   23.58,
         25.36,   27.09,   28.78,   30.44,   32.07,   33.67,   35.25,
         36.81,   38.35,   39.87,   41.38,   42.87,   44.35,   45.82,
         47.28,   48.73,   50.16,   51.59,   53.01,   54.42,   55.83,
         57.23,   58.62,   60.  ,   61.38,   62.75,   64.12,   65.48,
         66.83,   68.18,   69.53,   70.87,   72.21,   73.54,   74.87,
         76.2 ,   77.52,   78.84,   80.15,   81.46,   82.77,   84.07,
         85.37,   86.67,   87.97,   89.26,   90.55,   91.84,   93.12,
         94.4 ,   95.68,   96.96,   98.24,   99.51,  100.78,  102.05,
        103.31,  104.58,  105.84,  107.1 ,  108.36,  109.61,  110.87,
        112.12,  113.37,  114.62,  115.87,  117.11,  118.36,  119.6 ,
        120.84,  122.08,  123.32,  124.55,  125.79,  127.02,  128.25,
        129.48,  130.71,  131.94,  133.17,  134.39,  135.62,  136.84,
        138.06,  139.28,  140.5 ,  141.72,  142.94,  144.15,  145.37,
        146.58};

const int NOISE_LEVEL = 0; // add hysteresis to input when using analog/ADC
const int USE_ANALOG_INPUT = 1;
const int NR_OUTPUTS = 2;
const int NR_INPUTS = 3;
const float MAX_DISTANCE = 1.5; // m
const float TIME_LAG = 0.0000; // s, time to shift xcor analysis by to compensate for speaker lag
const float XCOR_DECAY_TIME = 1.000; // s
const float DESIRED_OUTPUT_BIN_DISTANCE = 0.025; // m, set desired value but ACTUAL_OUTPUT_BIN_DISTANCE is what is used
const float SAMPLE_RATE = 75000; // Hz //3in, 2out at 1.5m
//const float SAMPLE_RATE = 75000; // Hz //3in, 2out at 1.5m
//const float SAMPLE_RATE = 88000; // Hz //2in, 2out at 1.5m
//const float SAMPLE_RATE = 111500; // Hz //1in, 2out at 1.5m
//const float SAMPLE_RATE = 89500; // Hz //4in, 1out at 1.5m
//const float SAMPLE_RATE = 96500; // Hz //3in, 1out at 1.5m
//const float SAMPLE_RATE = 121000; // Hz //2in, 1out at 1.5m
//const float SAMPLE_RATE = 152000; // Hz //1in, 1out at 1.5m
const float SPEED_OF_SOUND = 340.3/2.0; // m/s, divide by 2 because sound travels out and back

// dependant variables
const float DISTANCE_PER_SAMPLE = SPEED_OF_SOUND/SAMPLE_RATE;
const int MERGE_BIN_CNT = DESIRED_OUTPUT_BIN_DISTANCE/DISTANCE_PER_SAMPLE;
const float ACTUAL_OUTPUT_BIN_DISTANCE = MERGE_BIN_CNT*DISTANCE_PER_SAMPLE; // m, see DESIRED_OUTPUT_BIN_DISTANCE
const int NR_SAMPLES = int(MAX_DISTANCE/DISTANCE_PER_SAMPLE)/MERGE_BIN_CNT*MERGE_BIN_CNT; // make sure NR_SAMPLES is a multple of merge
const int XCOR_DECAY_SAMPLES = SAMPLE_RATE * XCOR_DECAY_TIME;
const float XCOR_DECAY_RATE = 1.0 - 1.0/XCOR_DECAY_SAMPLES;
const int SAMPLE_LAG = TIME_LAG*SAMPLE_RATE + 1; // add 1 because inputs are always sampled time step behind
const int HISTORY_LEN = NR_SAMPLES/32 + 1;


inline uint32_t pin_to_ADC(uint8_t apin)
{
    if (apin == 0) return ADC1;
    else if (apin == 1) return ADC2;
    return ADC3;
}

void setup()
{
    clock_setup();
    serial_init(115200L*4);
    serial_printf(true, "Starting up\r\n");

    rcc_periph_clock_enable(RCC_GPIOB);

    for (uint8_t dpin=0; dpin<NR_OUTPUTS; dpin++)
    {
        gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, 1<<dpin); // Use PB0...
    }
    
	rcc_periph_clock_enable(RCC_GPIOA);
    if (USE_ANALOG_INPUT)
    {
	    rcc_periph_clock_enable(RCC_ADC1);
	    rcc_periph_clock_enable(RCC_ADC2);
	    rcc_periph_clock_enable(RCC_ADC3);
    }

	for (uint8_t apin=0; apin<NR_INPUTS; apin++)
	{
	    if (USE_ANALOG_INPUT)
	    {
	        uint32_t adc = pin_to_ADC(apin);

            gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, 1<<apin); // Use PA0...
	
	        adc_off(adc);

	        adc_disable_external_trigger_regular(adc);
	        adc_disable_scan_mode(adc);
	        adc_disable_eoc_interrupt(adc);
    	    adc_set_single_conversion_mode(adc);
            adc_set_continuous_conversion_mode(adc);
	        adc_set_sample_time_on_all_channels(adc, ADC_SMPR_SMP_3CYC); // need to play with this value and the prescaler
            adc_set_clk_prescale(ADC_CCR_ADCPRE_BY8);  // prescaler defaults to 2 at startup
            adc_power_on(adc);
        } else {
            gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, 1<<apin); // Use PA0...
        }
    }
    
	rcc_periph_clock_enable(RCC_RNG);
	RNG_CR |= RNG_CR_RNGEN;
}

uint32_t real_rand()
{
    while (!(RNG_SR & RNG_SR_DRDY));
    return RNG_DR;
}

#define HIST_TYPE uint32_t
#define HIST_TYPE_BITS (sizeof(HIST_TYPE)*8)

inline void insert_bit(HIST_TYPE* buf, HIST_TYPE bit, int16_t loc)
{
    int i = loc / HIST_TYPE_BITS;
    int offset = loc % HIST_TYPE_BITS;
    buf[i] = (buf[i]&(~(HIST_TYPE(1)<<offset))) | (bit << offset);
}  

inline unsigned int bitcount32(uint32_t i)
{
  //Parallel binary bit add
  i = i - ((i >> 1) & 0x55555555);
  i = (i & 0x33333333) + ((i >> 2) & 0x33333333);
  return (((i + (i >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
}

uint8_t wordbits[65536]; // use 64K of memory but fastest implementation
inline int popcount32_tbl(uint32_t i)
{
    return (wordbits[i&0xFFFF] + wordbits[i>>16]);
}

#define POPCNT(i) popcount32_tbl(i) // bitcount32(i) //__builtin_popcount(i)

int main(void)
{
// p = desired significance threshold for statistical significance, should be determined from desired false positive rate corrected for number of comparisons (NR_INPUTS*NR_SAMPLES*NR_OUTPUTS)
// sigma = norm_inverse_cdf(p/2), divide by 2 because we are using a single-tailed test.
// significant if xcors**2 > sum((input-mean_input)**2)*sigma**2

    int32_t xcors[NR_SAMPLES][NR_INPUTS][NR_OUTPUTS] = {0};
    int32_t prev_xcors[NR_SAMPLES][NR_INPUTS][NR_OUTPUTS] = {0};
    HIST_TYPE rand_hist[NR_OUTPUTS][HISTORY_LEN] = {0}; // need one extra slot due to the analog value being delayed by 1 cycle
    HIST_TYPE input[NR_INPUTS] = {0};
    int previous_input[NR_INPUTS] = {0};
    uint32_t t = 0;
    uint32_t start_time = 0;
    uint32_t rnd = 0;
    uint8_t calc_phase = ((-SAMPLE_LAG) & (HIST_TYPE_BITS-1)); // & with 31 to ensure it is always between 0 and 31, even when negative; % keeps sign.
    int16_t bit_pos = 0;
    int hist_i, xcor_i;
    int decay_i = 0;
    int r;
    bool adc_too_slow = false;
    int tmp_in;
    int dtime1=0, dtime2=0;
    bool first_time = true;
    
    for (uint32_t i=0; i<65536; i++) wordbits[i] = __builtin_popcount(i);
    
    setup();

    // select a2d input pin and start the conversion
    for (uint8_t apin=0; apin<NR_INPUTS; apin++)
    {
        if (USE_ANALOG_INPUT)
        {
            uint32_t adc = pin_to_ADC(apin);
            adc_set_regular_sequence(adc, 1, &apin);
        	adc_start_conversion_regular(adc);
        }
    }

    while (true)
    {
        int start=get_time_ms();
        rnd = real_rand();

        for (uint8_t dpin=0; dpin<NR_OUTPUTS; dpin++)
        {
            r = (rnd&(1<<dpin))>>dpin;
            insert_bit(rand_hist[dpin], r, bit_pos);
            if (r) gpio_set(GPIOB, 1<<dpin);
            else gpio_clear(GPIOB, 1<<dpin);
        }

        hist_i = (bit_pos+(HIST_TYPE_BITS-1)+1)/HIST_TYPE_BITS; // the 32bits we want start 31 bits back (positive values are back in time) and 1 more bit because input is delayed by 1 cycle
        xcor_i = calc_phase;
        do
        {
            if (hist_i>=HISTORY_LEN) hist_i-=HISTORY_LEN;
            for (uint8_t apin=0; apin<NR_INPUTS; apin++)
            {
                HIST_TYPE input_apin = input[apin];
                for (uint8_t dpin=0; dpin<NR_OUTPUTS; dpin++)
                {
                    HIST_TYPE shift_hist = rand_hist[dpin][hist_i];
                    
                    //compute actual xcross-correlation
                    xcors[xcor_i][apin][dpin] -= POPCNT(shift_hist ^ input_apin)*2 - HIST_TYPE_BITS; // count the number of 1s in the 32 bits *2 (for unit variance) and subtract the mean
                }
            }
            
            xcor_i += HIST_TYPE_BITS;
            hist_i++;
        } while (xcor_i<NR_SAMPLES);
        calc_phase++;
        calc_phase %= HIST_TYPE_BITS;
        bit_pos--;
        if (bit_pos<0) bit_pos += HISTORY_LEN*HIST_TYPE_BITS;

        // decay and detect significant values        
        for (uint8_t dpin=0; dpin<NR_OUTPUTS; dpin++)
        {
            for (uint8_t apin=0; apin<NR_INPUTS; apin++)
            {
                xcors[decay_i][apin][dpin] *= 1.0 - 1.0/(XCOR_DECAY_SAMPLES/float(NR_SAMPLES)); // we only decay once every NR_SAMPLES iterations
            }
        }
        decay_i++;
        if (decay_i >= NR_SAMPLES) decay_i = 0;

        // read input
        for (uint8_t apin=0; apin<NR_INPUTS; apin++)
        {
            input[apin] <<= 1;
            if (USE_ANALOG_INPUT)
            {
        	    uint32_t adc = pin_to_ADC(apin);
                while (!adc_eoc(adc)) adc_too_slow = true;
                
                tmp_in = adc_read_regular(adc);
                if ((tmp_in - previous_input[apin] < NOISE_LEVEL) & (tmp_in - previous_input[apin] > -NOISE_LEVEL))
                    input[apin] |= (input[apin]&0x2)>>1;
                else
                    input[apin] |= tmp_in > previous_input[apin];
                previous_input[apin] = tmp_in;
            } else {
                input[apin] |= gpio_get(GPIOA, 1<<apin)>>apin;
            }
        }

        t += 1;
        if (t >= XCOR_DECAY_SAMPLES)
        {
            int time = get_time_ms();
            t = 0;
            serial_printf(false, "\033[2J\033[1;1H"); // clear terminal screen
            serial_printf(false, "cycle time: %d %d %s\r\n", dtime1, dtime2, adc_too_slow?"adc too slow":"");
            for (uint8_t apin=0; apin<NR_INPUTS; apin++)
            {
                for (uint8_t dpin=0; dpin<NR_OUTPUTS; dpin++)
                {
                    serial_printf(false, "%d %d: ", apin, dpin);
                    for (int i=0;i<NR_SAMPLES;i+=MERGE_BIN_CNT)
                    {
                        float sum2 = 0;
                        for (int j=i; (j<i+MERGE_BIN_CNT)&(j<NR_SAMPLES); j++)
                        {
                            float tmp = xcors[j][apin][dpin] - prev_xcors[j][apin][dpin];
                            sum2 += tmp*tmp;
                        }
                        if (sum2 >= chi2_threshs[MERGE_BIN_CNT]*XCOR_DECAY_SAMPLES) serial_printf(false, "%d:%d ", int(i*DISTANCE_PER_SAMPLE*100), int(sum2/XCOR_DECAY_SAMPLES/chi2_threshs[MERGE_BIN_CNT]));
                    }
                    serial_printf(false, "\r\n");
                }
            }
            if (first_time)
            {
                memcpy(prev_xcors, xcors, sizeof(xcors));
                first_time = false;
            } else {
                for (int i=0;i<NR_SAMPLES;i++)
                {
                    for (uint8_t dpin=0; dpin<NR_OUTPUTS; dpin++)
                    {
                        for (uint8_t apin=0; apin<NR_INPUTS; apin++)
                        {
                            prev_xcors[i][apin][dpin] = prev_xcors[i][apin][dpin]*0.9 + 0.1*xcors[i][apin][dpin];
                        }
                    }
                }
            }
            serial_printf_flush();
            dtime1 = time - start_time;
            dtime2 = get_time_ms() - time;
            start_time = get_time_ms();
//            serial_printf_flush();
        }
    }
}

