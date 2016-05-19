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
        if np.fromstring(data_str, dtype=np.uint32) == 123:
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
        data = np.fromstring(read_N_bytes(ser, 7*4), dtype=np.uint32)
        nr_samples, nr_bins, merge_bin_cnt, nr_inputs, nr_outputs, cycle_time, io_time = data
        
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
        else:
            data = np.fromstring(read_N_bytes(ser, merged_xcors_size*4), dtype=np.float32)
        if data.size != merged_xcors_size:
            print "rejecting packet based upon bad size", merged_xcors_size, data.size
            continue
            
        merged_xcors = data.reshape((nr_bins/merge_bin_cnt, nr_inputs, nr_outputs))
        return nr_samples, nr_bins, merge_bin_cnt, nr_inputs, nr_outputs, cycle_time, io_time, merged_xcors


if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0', 115200*4, timeout=1)

    cum_p_thresh = 1.0-(1.0-scipy.stats.norm.cdf(3.0))*2  # number of sigmas/stds to normalize by
    print cum_p_thresh, scipy.stats.chi2.ppf(cum_p_thresh, 1)
    xcors_time = []
    white_xcors_time = []

    cnt = 0
    cum1 = 0
    cum2 = 0
    cum_fft = 0
    cum_max = 0
    while (True):
        cnt += 1
        nr_samples, nr_bins, merge_bin_cnt, nr_inputs, nr_outputs, cycle_time, io_time, merged_xcors = read_merged_xcor_struct(ser)

        if not READ_INPUT:
            if np.any(np.isnan(merged_xcors)) or np.any(np.abs(merged_xcors) > (nr_samples*merge_bin_cnt)**2/100.0) or (merge_bin_cnt > 1 and np.any(merged_xcors<0)):
                print "bad xcors"
                continue

            merge_t = 100
            merge_s = 1
            
            thresh = scipy.stats.chi2.ppf(cum_p_thresh, merge_bin_cnt)*nr_samples
            thresh8 = scipy.stats.chi2.ppf(cum_p_thresh, merge_t*merge_s*merge_bin_cnt)*nr_samples

            merged_xcors=merged_xcors.reshape((-1, nr_inputs*nr_outputs))
            
            xcors_conv = merged_xcors.copy()
            white_xcors_conv = merged_xcors.copy()
            for i in range(merged_xcors.shape[1]):
                if merge_bin_cnt == 1:
                    xcors_conv[:,i] = merged_xcors[:,i]**2
                    xcors_conv[:,i] = np.abs(np.convolve(xcors_conv[:,i], np.ones(merge_s), 'same'))

                    fft = np.fft.fft(merged_xcors[:,i]/np.sqrt(nr_samples), axis=0)
                    white_xcor = np.real(np.fft.ifft(fft/np.abs(fft), axis=0))*np.sqrt(nr_samples)*np.sqrt(merged_xcors.shape[0])
                    white_xcors_conv[:,i] = white_xcor**2
                    white_xcors_conv[:,i] = np.abs(np.convolve(white_xcors_conv[:,i], np.ones(merge_s), 'same'))
                    cum_max += np.max(white_xcors_conv[:,i]/thresh8)
                else:
                    xcors_conv[:,i] = np.convolve(merged_xcors[:,i], np.ones(merge_s), 'same') - np.convolve(merged_xcors[:,i], np.ones(merge_s)/merge_s, 'same') + merge_s*nr_samples*0
            
            xcors_time.append(xcors_conv)
            if len(xcors_time) > merge_t:
                xcors_time = xcors_time[1:(merge_t+1)]

            white_xcors_time.append(white_xcors_conv)
            if len(white_xcors_time) > merge_t:
                white_xcors_time = white_xcors_time[1:(merge_t+1)]

            plt.cla()
            if merge_bin_cnt == 1:
#                plt.plot(merged_xcors**2/thresh, '--')
    #            plt.plot(white_xcor**2/thresh,'--')
#                plt.plot(np.sum(np.array(xcors_time), axis=0)/thresh8)
                try:
                    plt.imshow(np.maximum(-1, np.minimum(1, np.array(xcors_time).squeeze()/np.max(np.abs(xcors_time)))))
                except TypeError:
                    pass
    #            plt.plot(np.sum(np.array(white_xcors_time), axis=0)/thresh8, '.')
    #            plt.ylim(0,10)
    #            for i in range(merged_xcors.shape[1]):
    #                plt.plot(np.sqrt(np.convolve(merged_xcors[:,i]**2, np.ones(merge), 'same')/thresh8),'--')
    #                plt.plot(np.sqrt(merged_xcors[:,i]**2/thresh), np.sqrt(np.convolve(merged_xcors[:,i]**2, np.ones(merge), 'same')/thresh8), '.b')
    #            plt.plot([0, 50], [0, 50])
    #            fft = np.fft.fft(merged_xcors/np.sqrt(nr_samples), axis=0)
    #            cum_fft += np.abs(fft)
    #            cum_fft[0] = 1
    #            abs_fft = np.abs(fft)#np.abs(cum_fft/cnt)#
    #            abs_fft[0] = 1
    #            abs_fft[abs_fft<0.5] = 0.5
    #            white_xcor = np.real(np.fft.ifft(fft/abs_fft, axis=0))
    #            white_xcor *= np.sqrt(merged_xcors.shape[0])
    #            plt.plot(cum_fft/cnt)
    #            plt.plot(white_xcor)
    #            plt.plot(white_xcor*20,(merged_xcors/np.sqrt(nr_samples)),'b.')
    #            plt.plot(merged_xcors/np.sqrt(nr_samples))
    #            cum1 += np.mean(merged_xcors[10:,:]**2/thresh>1)
    #            cum2 += np.mean((white_xcor[10:,:]*np.sqrt(nr_samples))**2/thresh>1)
                cum1 += np.mean(np.sum(np.array(xcors_time)[:,1:], axis=0)/thresh8>1)
                cum2 += np.mean(np.sum(np.array(white_xcors_time)[:,1:], axis=0)/thresh8>1)
                print "%0.6f, %0.6f" % (cum1/cnt, cum2/cnt),
            else:
                tmp = merged_xcors/thresh
                plt.plot(tmp*(tmp>1))
                tmp = np.sum(np.array(xcors_time), axis=0)/thresh8
                plt.plot(tmp*(tmp>1), '--')
                print "%0.6f, %0.6f" % (np.mean(merged_xcors/thresh>1), np.mean(np.sum(np.array(xcors_time), axis=0)/thresh8>1)),
            plt.show()
        else:
            input_str = ''
            for x in merged_xcors[:,0,0]:
                input_str += np.binary_repr(x, width=32)
            input_data = (np.fromstring(input_str, dtype=np.uint8) - ord('0')).astype(np.float32)
            plt.cla()
            plt.plot(np.abs(np.fft.fft(input_data-np.mean(input_data))))
            plt.show()
        print cycle_time, io_time, cum_max/cnt, np.max(np.abs(xcors_time))
