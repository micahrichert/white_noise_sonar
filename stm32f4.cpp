#include <stdint.h>
#include <math.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/f4/rng.h>
#include <libopencm3/stm32/adc.h>

#include "serial_printf.cpp"
#include "clock.cpp"

#define USE_BUILTIN 0  // slow on stm32 but may be fast on other chips
#define USE_PARALLEL_BITS 0  // slower than lookup but doesn't use any memory
#define USE_LOOKUP 1  // fastest solution on stm32 but uses 64K of memory
#include "popcount.cpp"

/*
Chi2 significance threshold table generated with the following python code:

import scipy.stats
import numpy as np
cum_p_thresh = 1.0-(1.0-scipy.stats.norm.cdf(3.0))*2  # 3.0 sigmas (1 in 370 false positive) works pretty well
thresh = scipy.stats.chi2.ppf(cum_p_thresh, np.arange(1, 100))
np.set_printoptions(precision=2)
print repr(thresh)

*/

#define HIST_TYPE uint32_t
#define HIST_TYPE_BITS (sizeof(HIST_TYPE)*8)

#define RECORD_INPUT 0

const bool USE_VARIANCE = true;
const bool USE_ANALOG_INPUT = true;

const int NOISE_LEVEL = 0; // add hysteresis to input when using analog/ADC
const unsigned int NR_OUTPUTS = 1;
const unsigned int NR_INPUTS = 1;
const float MAX_DISTANCE = 1.5;//1.5; // m
const int SAMPLE_LAG = 0;  // must be a multiple of HIST_TYPE_BITS
const unsigned int MERGE_BIN_CNT = 8;
const float MAX_SPEED = 1.2;  // m/s
const float MIN_SPEED = 0.0;  // m/s
const float SPEED_BANDWIDTH = 0.2;  // m/s, when a large target is stationary what is the fastest single motion channel that still detects it reasonably
const unsigned int NR_SPEEDS = ceil((MAX_SPEED - MIN_SPEED) / (SPEED_BANDWIDTH * 2));
const float SPEED_STEP = (MAX_SPEED - MIN_SPEED) / (NR_SPEEDS * 2);
#if (RECORD_INPUT)
    const float XCOR_DECAY_TIME = 1.0;
    const unsigned int NOP_LOOP = 270;
#else
    const float XCOR_DECAY_TIME = 0.1;//0.10;//0.05;//1.0; // s
    const unsigned int NOP_LOOP = 0;
#endif
//const float SAMPLE_RATE = 75000; // Hz //3in, 2out at 1.5m
//const float SAMPLE_RATE = 75000; // Hz //3in, 2out at 1.5m
//const float SAMPLE_RATE = 88000; // Hz //2in, 2out at 1.5m
//const float SAMPLE_RATE = 111500; // Hz //1in, 2out at 1.5m
//const float SAMPLE_RATE = 80000; // Hz //1in, 2out at 1.5m
//const float SAMPLE_RATE = 47000; // Hz //1in, 2out at 1.5m, 10speeds
//const float SAMPLE_RATE = 89500; // Hz //4in, 1out at 1.5m
//const float SAMPLE_RATE = 96500; // Hz //3in, 1out at 1.5m
//const float SAMPLE_RATE = 121000; // Hz //2in, 1out at 1.5m
const float SAMPLE_RATE = 105000; // Hz //1in, 1out at 1.5m
//const float SAMPLE_RATE = 400000; // Hz //1in, 1out at 1.5m
//const float SAMPLE_RATE = 50000; // Hz //1in, 1out at 1.5m, 10speeds
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

inline uint32_t pin_to_ADC(uint8_t apin)
{
    if (apin == 0) return ADC1;
    else if (apin == 1) return ADC2;
    return ADC3;
}

void setup()
{
    clock_setup();
    serial_init(115200*16);

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
    
    // enable the RNG
	rcc_periph_clock_enable(RCC_RNG);
	RNG_CR |= RNG_CR_RNGEN;
	
	popcount_setup();
}

uint32_t real_rand()
{
    while (!(RNG_SR & RNG_SR_DRDY));
    return RNG_DR;
}

inline void insert_bit(HIST_TYPE* buf, HIST_TYPE bit, int16_t loc)
{
    int i = loc / HIST_TYPE_BITS;
    int offset = loc % HIST_TYPE_BITS;
    buf[i] = (buf[i]&(~(HIST_TYPE(1)<<offset))) | (bit << offset);
}

typedef struct
{
    uint32_t magic;
    uint32_t magic2;
    uint32_t nr_samples;
    uint32_t nr_bins;
    uint32_t merge_bin_cnt;
    uint32_t nr_inputs;
    uint32_t nr_outputs;
    uint32_t cycle_time;
    uint32_t io_time;
#if (RECORD_INPUT)
    uint32_t merged_xcors[NR_BINS/MERGE_BIN_CNT][NR_INPUTS][NR_OUTPUTS];
#else
    float merged_xcors[NR_BINS/MERGE_BIN_CNT][NR_INPUTS][NR_OUTPUTS];
    float input_var;
#endif
} __attribute__((packed)) merged_xcors_struct_t;

int main(void)
{
// p = desired significance threshold for statistical significance, should be determined from desired false positive rate corrected for number of comparisons (NR_INPUTS*NR_BINS*NR_OUTPUTS)
// sigma = norm_inverse_cdf(p/2), divide by 2 because we are using a single-tailed test.
// significant if xcors**2 > sum((input-mean_input)**2)*sigma**2

    int32_t xcors[NR_SPEEDS][NR_BINS+EXTRA_DILATION_BINS][NR_INPUTS][NR_OUTPUTS] = {0};
    // use triple buffering because the dma transfer takes a little time to initiate
    merged_xcors_struct_t merged_xcors_structs[3] = {
        {.magic=0x12345678, .magic2=0x9ABCDEF0, .nr_samples=XCOR_DECAY_SAMPLES, .nr_bins=NR_BINS, .merge_bin_cnt=MERGE_BIN_CNT, .nr_inputs=NR_INPUTS, .nr_outputs=NR_OUTPUTS},
        {.magic=0x12345678, .magic2=0x9ABCDEF0, .nr_samples=XCOR_DECAY_SAMPLES, .nr_bins=NR_BINS, .merge_bin_cnt=MERGE_BIN_CNT, .nr_inputs=NR_INPUTS, .nr_outputs=NR_OUTPUTS},
        {.magic=0x12345678, .magic2=0x9ABCDEF0, .nr_samples=XCOR_DECAY_SAMPLES, .nr_bins=NR_BINS, .merge_bin_cnt=MERGE_BIN_CNT, .nr_inputs=NR_INPUTS, .nr_outputs=NR_OUTPUTS}
    };
    int merged_xcors_ind = 0;
    HIST_TYPE rand_hist[NR_OUTPUTS][HISTORY_LEN] = {0};
    HIST_TYPE input[NR_INPUTS] = {0};
    int previous_input[NR_INPUTS] = {0};
    float slow_input[NR_INPUTS] = {0};
    float slow_input2[NR_INPUTS] = {0};
    uint32_t t = 0;
    uint32_t start_time = 0;
    uint32_t rnd = 0;
    int calc_phase;
    float dilations[NR_SPEEDS] = {0};
    int16_t bit_pos = 0;
    int hist_i;
    int xcor_i[NR_SPEEDS];
    int decay_i = 0;
    int r;
    bool adc_too_slow = false;
    int cycle_time=0, io_time=0;
    bool first_time = true;
    bool do_reset = true;
    int reset_cnt = 0;
    
    setup();
    
    calc_phase = (-1 & (HIST_TYPE_BITS-1)); // & with 31 to ensure it is always between 0 and 31, even when negative; % keeps sign.

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
        int start=get_time_us();
        rnd = real_rand();

        for (uint8_t dpin=0; dpin<NR_OUTPUTS; dpin++)
        {
            r = (rnd&(1<<(dpin)))>>dpin;
            insert_bit(rand_hist[dpin], r, bit_pos);
            if (r) gpio_set(GPIOB, 1<<dpin);
            else gpio_clear(GPIOB, 1<<dpin);
        }

/*        // generate noise output but don't analyze it
        for (uint8_t dpin=NR_OUTPUTS; dpin<4; dpin++)
        {
            r = (rnd&(1<<dpin))>>dpin;
            if (r) gpio_set(GPIOB, 1<<dpin);
            else gpio_clear(GPIOB, 1<<dpin);
        }
*/
        hist_i = (bit_pos+(HIST_TYPE_BITS-1)+1+SAMPLE_LAG)/HIST_TYPE_BITS; // the 32bits we want start 31 bits back (positive values are back in time)
        for (int s=0; s<NR_SPEEDS;s++) xcor_i[s] = calc_phase + dilations[s];
#if (!RECORD_INPUT)
        do
        {
            if (hist_i>=HISTORY_LEN) hist_i-=HISTORY_LEN;
            for (uint8_t apin=0; apin<NR_INPUTS; apin++)
            {
                HIST_TYPE input_apin = input[apin];
                for (uint8_t dpin=0; dpin<NR_OUTPUTS; dpin++)
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

        calc_phase += 1;
        calc_phase %= 32;
        for (int s=0; s<NR_SPEEDS;s++)
        {
//            dilations[s] += SPEED_DILATION_STEP*s;
            dilations[s] += SPEED_DILATION_STEP*(s*2+1) + MIN_SPEED;
        }
#endif
        t += 1;
        if (decay_i == 0)
        {
            if (t >= XCOR_DECAY_SAMPLES)
            {
                merged_xcors_structs[merged_xcors_ind].nr_samples = t;
                t = 0;
                for (int s=0; s<NR_SPEEDS;s++) dilations[s] = 0;
                do_reset = true;

                int start_io_time = get_time_us();
                reset_cnt += 1;
                
                //transfer merged_xcors over serial using dma
                merged_xcors_structs[merged_xcors_ind].cycle_time = cycle_time;
                merged_xcors_structs[merged_xcors_ind].io_time = io_time;
                
                dma_set_memory_address(DMA1, DMA_STREAM3, (uint32_t)&merged_xcors_structs[merged_xcors_ind]);
                dma_set_number_of_data(DMA1, DMA_STREAM3, sizeof(merged_xcors_struct_t));

                usart_enable_tx_dma(USART3);
                dma_enable_stream(DMA1, DMA_STREAM3);

                merged_xcors_ind += 1; // switch buffers
                merged_xcors_ind %= sizeof(merged_xcors_structs)/sizeof(*merged_xcors_structs);

                cycle_time = start_io_time - start_time;
                io_time = get_time_us() - start_io_time;
                start_time = get_time_us();
            } else {
                do_reset = false;                
            }
        }

#if (!RECORD_INPUT)
        // gets run once every NR_BINS
        for (uint8_t dpin=0; dpin<NR_OUTPUTS; dpin++)
        {
            for (uint8_t apin=0; apin<NR_INPUTS; apin++)
            {
                if (USE_VARIANCE)
                {
                    bool reset = do_reset && ((decay_i % MERGE_BIN_CNT) == 0);
                    
                    if (reset) merged_xcors_structs[merged_xcors_ind].merged_xcors[decay_i/MERGE_BIN_CNT][apin][dpin] = xcors[0][decay_i][apin][dpin]*xcors[0][decay_i][apin][dpin];// - (int)NR_BINS;

                    for (int s=reset; s<NR_SPEEDS;s++)
                        merged_xcors_structs[merged_xcors_ind].merged_xcors[decay_i/MERGE_BIN_CNT][apin][dpin] += xcors[s][decay_i][apin][dpin]*xcors[s][decay_i][apin][dpin];// - (int)NR_BINS;
  
                    for (int s=0; s<NR_SPEEDS;s++) xcors[s][decay_i][apin][dpin] = 0;
                } else {
                    bool reset = do_reset && ((decay_i % MERGE_BIN_CNT) == 0);
                    
                    if (reset) merged_xcors_structs[merged_xcors_ind].merged_xcors[decay_i/MERGE_BIN_CNT][apin][dpin] = xcors[0][decay_i][apin][dpin];

                    for (int s=reset; s<NR_SPEEDS;s++)
                        merged_xcors_structs[merged_xcors_ind].merged_xcors[decay_i/MERGE_BIN_CNT][apin][dpin] += xcors[s][decay_i][apin][dpin];

                    for (int s=0; s<NR_SPEEDS;s++) xcors[s][decay_i][apin][dpin] = 0;
                }
            }
        }
        decay_i++;
        if (decay_i >= NR_BINS) decay_i = 0;
#endif
        bit_pos--;
        if (bit_pos<0) bit_pos += HISTORY_LEN*HIST_TYPE_BITS;
        
#if (RECORD_INPUT)
        if (t%32 == 31 && t/32<NR_BINS/MERGE_BIN_CNT)
        {
            for (uint8_t apin=0; apin<NR_INPUTS; apin++)
            {
                merged_xcors_structs[merged_xcors_ind].merged_xcors[t/32][apin][0] = input[apin];
            }
        }
#endif
        
        for (int tmp=0;tmp<NOP_LOOP;tmp++) asm("nop");

        // read input
        for (uint8_t apin=0; apin<NR_INPUTS; apin++)
        {
            input[apin] <<= 1;
            if (USE_ANALOG_INPUT)
            {
        	    uint32_t adc = pin_to_ADC(apin);
                while (!adc_eoc(adc)) adc_too_slow = true;
                
                int tmp_in = adc_read_regular(adc);

                if ((tmp_in - previous_input[apin] < NOISE_LEVEL) & (tmp_in - previous_input[apin] > -NOISE_LEVEL))
                    // if we don't exceed the required change threshold then just repeat the previous input value
                    input[apin] |= ((input[apin]&0x2)>>1)*0x1;
                else
                    input[apin] |= (tmp_in > previous_input[apin])*0x1;
/*
                input[apin] |= (tmp_in > slow_input[apin])*0x1;
                auto tmp = slow_input[apin];
                slow_input[apin] = (slow_input2[apin] + slow_input[apin]) * 0.9 + 0.1 * tmp_in;
                slow_input2[apin] = slow_input2[apin] * 0.9 + 0.1 * (slow_input[apin] - tmp);
*/

#if (!RECORD_INPUT)
                merged_xcors_structs[merged_xcors_ind].input_var *= 0.99;
                merged_xcors_structs[merged_xcors_ind].input_var += 0.01 * (tmp_in - previous_input[apin]) * (tmp_in - previous_input[apin]);
#endif
                previous_input[apin] = tmp_in;
            } else {
                input[apin] |= (gpio_get(GPIOA, 1<<apin)>>apin)*0x1;
            }
        }
    }
}

