import serial
import time
import numpy as np
import nonblocking_plot as plt
#from matplotlib import pylab as plt
import scipy.stats

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
    while ord(read_N_bytes(ser, 1)) != 0xA1:
        pass

# typedef struct
# {
#     uint8_t magic;
#     uint8_t count;
#     uint8_t nr_inputs;
#     uint8_t nr_outputs;
#     uint32_t inputs[NR_INPUTS];
#     uint32_t outputs[NR_OUTPUTS];
# } __attribute__((packed)) packet_t;
def read_packet(ser):
    while True:
        find_magic(ser)
        data = np.fromstring(read_N_bytes(ser, 3), dtype=np.uint8)
        count, nr_inputs, nr_outputs = data
        
        if nr_inputs > 10 or nr_outputs > 10 or nr_inputs*nr_outputs == 0:
            print "too many inputs or outputs likely corrupt stream or still synching"
            continue
            
        inputs = np.fromstring(read_N_bytes(ser, 4*nr_inputs), dtype=np.uint32)
        outputs = np.fromstring(read_N_bytes(ser, 4*nr_outputs), dtype=np.uint32)
        
        return count, inputs, outputs


if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0', 2500000, parity=serial.PARITY_NONE, timeout=1)

    cnt = 0
    prev_count, inputs, outputs = read_packet(ser)
    nr_inputs = inputs.size

    start_time = time.time()
    abs_fft = [0]*nr_inputs
    fft_cnt = 0
    while True:
        ins = []
        for i in range(160000/32):
            cnt += 1
            count, inputs, outputs = read_packet(ser)
            if np.uint8(prev_count+1) != count:
                print "dropped packets?", prev_count, count
            prev_count = count
            if inputs.size == nr_inputs:
                ins.append(inputs)
        try:
            for j in range(nr_inputs):
                bins = [np.fromstring(format(int(i[j]), '032b'), dtype=np.uint8)-ord('0') for i in ins]
                inputs = np.array(bins).reshape(-1)
                abs_fft[j] = np.abs(np.fft.fft(inputs-np.mean(inputs)))
            fft_cnt = 1
            plt.clf()
            for j in range(nr_inputs):
                plt.subplot(1, nr_inputs, j+1)
                plt.plot(abs_fft[j]/fft_cnt)
            plt.show()
        except:
            raise
        
