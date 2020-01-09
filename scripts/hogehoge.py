import pandas as pd
import numpy as np
from matplotlib import pyplot as plt
import seaborn as sns
import time

sns.set()
sns.set_style('whitegrid')
sns.set_palette('gray')

np.random.seed(2018)

fig = plt.figure()

# top
x1 = np.random.uniform(0, 100, 20)
y1 = x1 * np.random.uniform(1, 2, 20)

ax1 = fig.add_subplot(2, 1, 1)
ax1.scatter(x1, y1)
ax1.set_xlabel("weight [g]")
ax1.set_ylabel("length [cm]")
ax1.set_xlim(0, 110)
ax1.set_ylim(0, 190)

# bottom
x2 = np.array(['ERS1', 'ERS2', 'ETR1', 'ETR2', 'EIN4'])
y2 = np.array([12.0, 3.1, 11.8, 2.9, 6.2])
x2_position = np.arange(len(x2))

ax2 = fig.add_subplot(2, 1, 2)
ax2.bar(x2_position, y2, tick_label=x2)
ax2.set_xlabel("gene")
ax2.set_ylabel("expression [log(TPM)]")

# show plots
fig.tight_layout()
fig.show()
time.sleep(2)
