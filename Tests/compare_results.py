import numpy as np
from matplotlib import pyplot as plt
import pandas as pd

before = pd.read_csv("test_data.csv").to_numpy()
after = pd.read_csv("cmake-build-debug/results.csv").to_numpy()

print(before)

time = np.linspace(0, 10, 5000 - 1)

plt.plot(time, before, label="Before Kalman")
plt.plot(time, after, label="After Kalman")

plt.show()