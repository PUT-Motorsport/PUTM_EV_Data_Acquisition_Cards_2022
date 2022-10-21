import numpy as np
import pandas as pd
from matplotlib import pyplot as plt

time = np.linspace(0, 10, 5000)
signal = np.sin(time)

noise = np.random.randn(*signal.shape)  / 40 # noise amplitude is 10% of the signal amplitude

signalWithNoise = signal + noise

# save to file
with open("test_data.csv", 'w') as file:
    for row in np.nditer(signalWithNoise):
        file.write(str(row) + ',\n')

# visualize

plt.plot(time, signal)
plt.plot(time, signalWithNoise)
plt.show()