import serial
import time
import numpy as np
import nonblocking_plot as plt
import scipy.stats

READ_INPUT=0

def read_N_bytes(ser, N, first_read_timeout=1.5, subsequent_read_timeout=0.5):
    bytes_read = ''
    start_time = time.time()
    while len(bytes_read) < N:
        try:
            bytes_read += ser.read(N - len(bytes_read))
            start_time = time.time()
        except serial.serialutil.SerialException:
            pass
        if len(bytes_read) == 0 and time.time()-start_time > first_read_timeout:
            raise serial.serialutil.SerialException("No data read within first_read_timeout period")
        if time.time()-start_time > subsequent_read_timeout:
            return bytes_read
    return bytes_read


def find_magic(ser):
    data_str = read_N_bytes(ser, 1*4)
    while True:
        if np.fromstring(data_str, dtype=np.uint32) == 0x12345678:
            return
        data_str = data_str[1:] + read_N_bytes(ser, 1)

# typedef struct
# {
#     float magic;
#     float nr_samples;
#     float merge_bin_cnt;
#     float nr_inputs;
#     float nr_outputs;
#     float cycle_time;
#     float io_time;
#     float merged_xcors[NR_SAMPLES/MERGE_BIN_CNT][NR_INPUTS][NR_OUTPUTS];
# } merged_xcors_struct_t;
def read_merged_xcor_struct(ser):
    while True:
        find_magic(ser)
        data = np.fromstring(read_N_bytes(ser, 8*4), dtype=np.uint32)
        magic2, nr_samples, nr_bins, merge_bin_cnt, nr_inputs, nr_outputs, cycle_time, io_time = data
        
        if magic2 != 0x9ABCDEF0:
            print "bad magic2"
            continue
        
        if nr_samples <= 0 or nr_bins <= 0 or merge_bin_cnt <= 0 or nr_inputs <= 0 or nr_outputs <= 0:
            print "rejecting packet based upon receiving 0"
            continue
        
        if nr_inputs > 5 or nr_outputs > 5:
            print "rejecting packet based upon to large inputs or outputs", nr_inputs, nr_outputs
            continue
            
        if merge_bin_cnt > 32:
            print "rejecting packet based upon merge_bin_cnt too high", merge_bin_cnt
            continue
            
        merged_xcors_size = int(nr_bins/merge_bin_cnt*nr_inputs*nr_outputs)

        if READ_INPUT:
            data = np.fromstring(read_N_bytes(ser, merged_xcors_size*4), dtype=np.uint32)
            input_var = 0
        else:
            data = np.fromstring(read_N_bytes(ser, (merged_xcors_size+1)*4), dtype=np.float32)
            input_var = data[-1]
            data = data[0:merged_xcors_size]
        if data.size != merged_xcors_size:
            print "rejecting packet based upon bad size", merged_xcors_size, data.size
            continue
            
        merged_xcors = data.reshape((nr_bins/merge_bin_cnt, nr_inputs, nr_outputs))
        return nr_samples, nr_bins, merge_bin_cnt, nr_inputs, nr_outputs, cycle_time, io_time, merged_xcors, input_var


if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0', 115200*16, parity=serial.PARITY_NONE, timeout=1)

    cum_p_thresh = 1.0-(1.0-scipy.stats.norm.cdf(3.0))*2  # number of sigmas/stds to normalize by
    print cum_p_thresh, scipy.stats.chi2.ppf(cum_p_thresh, 1)
    xcors_time = [np.zeros(0)]

    cnt = 0
    cum1 = 0
    cum2 = 0
    cum_fft = 0
    cum_max = 0
    prev_time = 0
    m = 0
    while (True):
        cnt += 1
        if not READ_INPUT:
            for i in range(10):
                nr_samples, nr_bins, merge_bin_cnt, nr_inputs, nr_outputs, cycle_time, io_time, merged_xcors, input_var = read_merged_xcor_struct(ser)

                merged_xcors[np.isnan(merged_xcors)] = 0;
                if False:#merge_bin_cnt > 1:
                    merged_xcors[merged_xcors<0] = 0;
    #            if np.any(np.isnan(merged_xcors)) or np.any(np.abs(merged_xcors) > (nr_samples*merge_bin_cnt)**2/100.0) or (merge_bin_cnt > 1 and np.any(merged_xcors<0)):
    #                print "bad xcors"
    #                continue

                merge_t = 100
                merge_s = 1
                
                merged_xcors=merged_xcors.reshape((-1, nr_inputs*nr_outputs))
                
                if merge_s>1:
                    for j in range(merged_xcors.shape[1]):
                        merged_xcors[:,j] = np.convolve(merged_xcors[:,j], np.ones(merge_s), 'same')
                
                if merged_xcors.shape[0] != xcors_time[0].shape[0]:
                    xcors_time = [] 
                
                xcors_time.append(merged_xcors)
                if len(xcors_time) > merge_t:
                    xcors_time = xcors_time[1:(merge_t+1)]

            thresh = scipy.stats.chi2.ppf(cum_p_thresh, merge_bin_cnt)*nr_samples
            thresh8 = scipy.stats.chi2.ppf(cum_p_thresh, merge_t*merge_s*merge_bin_cnt)*nr_samples

            plt.cla()
            if True:#merge_bin_cnt == 1:
#                plt.plot(merged_xcors**2/thresh, '--')
    #            plt.plot(white_xcor**2/thresh,'--')
#                plt.plot(np.sum(np.array(xcors_time), axis=0)/thresh8)
#                for i in range(merged_xcors.shape[1]):
#                    plt.subplot(merged_xcors.shape[1], 1, i+1)
                try:
                    plt.imshow(np.maximum(-1, np.minimum(1, np.array(xcors_time)[::merge_s,2:,0]**1/merge_s/merge_bin_cnt/nr_samples/10)), interpolation='none')#/20000.0/3)#np.maximum(-1, np.minimum(1, (np.array(xcors_time)[:,:,0])/thresh/10)))
#                    plt.plot(np.mean(np.array(xcors_time)[:,:,0],axis=1))
#                    plt.plot(merged_xcors)
#                    plt.ylim(0,10000)
                    pass
                except TypeError:
                    pass
    #            plt.plot(np.sum(np.array(white_xcors_time), axis=0)/thresh8, '.')
    #            plt.ylim(0,10)
    #            for i in range(merged_xcors.shape[1]):
    #                plt.plot(np.sqrt(np.convolve(merged_xcors[:,i]**2, np.ones(merge), 'same')/thresh8),'--')
    #                plt.plot(np.sqrt(merged_xcors[:,i]**2/thresh), np.sqrt(np.convolve(merged_xcors[:,i]**2, np.ones(merge), 'same')/thresh8), '.b')
    #            plt.plot([0, 50], [0, 50])
                fft = np.fft.fft(merged_xcors[:,:]-np.mean(merged_xcors[:,:]), axis=0)
#                cum_fft += np.abs(fft)
    #            cum_fft[0] = 1
    #            abs_fft = np.abs(fft)#np.abs(cum_fft/cnt)#
    #            abs_fft[0] = 1
    #            abs_fft[abs_fft<0.5] = 0.5
    #            white_xcor = np.real(np.fft.ifft(fft/abs_fft, axis=0))
    #            white_xcor *= np.sqrt(merged_xcors.shape[0])
#                plt.plot(cum_fft/cnt)
    #            plt.plot(white_xcor)
    #            plt.plot(white_xcor*20,(merged_xcors/np.sqrt(nr_samples)),'b.')
    #            plt.plot(merged_xcors/np.sqrt(nr_samples))
    #            cum1 += np.mean(merged_xcors[10:,:]**2/thresh>1)
    #            cum2 += np.mean((white_xcor[10:,:]*np.sqrt(nr_samples))**2/thresh>1)
                cum1 += np.mean(np.sum(np.array(xcors_time)[:,1:], axis=0)/thresh8>1)
                m += np.mean(merged_xcors)
                print "%0.6f, %0.6f" % (cum1/cnt, cum2/cnt), nr_bins, m/cnt, (np.min(xcors_time), np.max(xcors_time)), nr_samples, input_var,
            else:
                tmp = merged_xcors/thresh
                plt.plot(tmp*(tmp>1))
                tmp = np.sum(np.array(xcors_time), axis=0)/thresh8
                plt.plot(tmp*(tmp>1), '--')
                print "%0.6f, %0.6f" % (np.mean(merged_xcors/thresh>1), np.mean(np.sum(np.array(xcors_time), axis=0)/thresh8>1)),
            plt.show()
        else:
            nr_samples, nr_bins, merge_bin_cnt, nr_inputs, nr_outputs, cycle_time, io_time, merged_xcors, input_var = read_merged_xcor_struct(ser)

            input_str = ''
            for x in merged_xcors[:,0,0]:
                input_str += np.binary_repr(x, width=32)
            input_data = (np.fromstring(input_str, dtype=np.uint8) - ord('0')).astype(np.float32)
            plt.cla()
            fft_abs = np.abs(np.fft.fft(input_data-np.mean(input_data)))
            fft_abs = fft_abs[0:fft_abs.shape[0]/2]
            cum_fft += fft_abs
#            print input_data.shape, nr_samples, cycle_time
            plt.plot(np.arange(fft_abs.shape[0])*nr_samples/input_data.shape[0]/(cycle_time/1000000.0), cum_fft/cnt)#fft_abs)
            plt.show()
        print cycle_time, io_time, cnt, time.time()-prev_time
        prev_time = time.time()
