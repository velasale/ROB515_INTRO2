import csv
import numpy as np
import matplotlib.pyplot as plt
from numpy.fft import fft, ifft

with open('signals.csv', newline='') as f:
    reader = csv.reader(f)
    data = list(reader)

signal1 = []
signal2 = []
signal3 = []

# Convert string representation of list into list
# source: https://www.geeksforgeeks.org/python-convert-a-string-representation-of-list-into-list/?ref=rp
for i in range(1000):
    x = data[0][i]
    res = x.strip('][').split(', ')
    signal1.append(int(res[0]))
    signal2.append(int(res[1]))
    signal3.append(int(res[2]))

plt.plot(signal3)

sr = 33

X = fft(signal1)
N = len(X)
n = np.arange(N)
T = N/sr
freq = n/T

plt.figure(figsize = (12, 6))
plt.subplot(121)

plt.stem(freq, np.abs(X), 'b', \
         markerfmt=" ", basefmt="-b")
plt.xlabel('Freq (Hz)')
plt.ylabel('FFT Amplitude |X(freq)|')
plt.xlim(0, 350)
plt.ylim(0, 500)

plt.show()