import matplotlib.pyplot as plt
import pandas as pd
import numpy as np


data = pd.read_csv('zero_noize_gyro_accel.csv')
print(len(data))
print(data)

fig, ax = plt.subplots(3, 2)

for head in data.head():
    # Computing IQR
    Q1 = data[head].quantile(0.25)
    Q3 = data[head].quantile(0.75)
    IQR = Q3 - Q1

    # Filtering Values between Q1-1.5IQR and Q3+1.5IQR
    filtered = data.query(f'(@Q1 - 1.5 * @IQR) <= {head} <= (@Q3 + 1.5 * @IQR)')[head]

    mu = filtered.mean()
    var = filtered.var()
    print(f'{head}: mu={mu:.6f}, sig^2={var:.6f}')

    i = 'XYZ'.find(head[-1])
    j = 1 if 'A' in head else 0

    filtered.hist(bins=36, ax=ax[i, j], density=True)

    from math import pi, sqrt, exp

    def f(x):
        return 1 / sqrt(2*pi*var) * exp(-((x-mu)**2)/(2*var))

    X = np.linspace(Q1-1.5*IQR, Q3+1.5*IQR, 100)
    # ax[i, j].plot(X, list(map(f, X)), color='red')

    ax[i, j].set_label(head)

plt.subplots_adjust(left=0.1,
                    bottom=0.1,
                    right=0.9,
                    top=0.9,
                    wspace=0.25,
                    hspace=0.6)

plt.show()
