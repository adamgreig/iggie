"""
This script plots ADC telemetry from a Saleae CSV dump.

Using the Saleae seems more reliable to capture multi-byte words
than the BMP's onboard UART at 3.5MBd.

Run with the path to the Saleae CSV file as the first and only argument.

Used to tune Kalman filters. Modify as required for channel in use.
"""

import sys
import numpy as np
import matplotlib.pyplot as plt

from tqdm import tqdm

with open(sys.argv[1], "r") as f:
    contents = f.read()

lines = contents.split("\n")[1:-1]
data = bytearray(len(lines))
for idx, line in enumerate(tqdm(lines)):
    try:
        word = int(line.split(",")[1])
        data[idx] = word
    except IndexError:
        print(idx, line)
        break

d = np.frombuffer(data, dtype="<u2")

# Modify as required for converting from ADC to amps/volts
# d = d.astype(float) * 3.3 * 0.04 / 4096
d = d.astype(float) * (3.3 * 200.6 / 4096.0) * 1.0244266

# Kalman constants
Q = 1e6
R = 1e0
dt = 1.10857e-5

# dd = d[int(2.5*1/dt):int(6*1/dt)]
dd = d[int(4*1/dt):int(6*1/dt)]
print(dd.var())

# Kalman init
x = np.zeros(2)
xp = np.zeros(2)
P = np.eye(2) * 1e-3
Pp = np.zeros((2, 2))
Q0 = np.zeros((2, 2))
Q0[0][0] = Q * (dt**4) / 4
Q0[0][1] = Q * (dt**3) / 2
Q0[1][0] = Q * (dt**3) / 2
Q0[1][1] = Q * (dt**2) / 1

# Kalman output storage
xout = np.empty(d.size)

for idx, z in enumerate(tqdm(d)):
    # Kalman predict
    xp[0] = x[0] + dt*x[1]
    xp[1] = x[1]
    Pp[0][0] = P[0][0] + P[1][0]*dt
    Pp[0][1] = P[0][1] + P[1][1]*dt
    Pp[1][0] = P[1][0] + P[1][1]*dt
    Pp[1][1] = P[1][1]
    Pp[0][0] += Pp[0][1]*dt
    Pp += Q0

    # Kalman update
    y = z - xp[0]
    k = 1 / (Pp[0][0] + R)
    x[0] = xp[0] + k * Pp[0][0] * y
    x[1] = xp[1] + k * Pp[1][0] * y
    P[0][0] = Pp[0][0] - k * Pp[0][0] * Pp[0][0]
    P[0][1] = Pp[0][1] - k * Pp[0][0] * Pp[0][1]
    P[1][0] = Pp[1][0] - k * Pp[1][0] * Pp[0][0]
    P[1][1] = Pp[1][1] - k * Pp[1][0] * Pp[0][1]

    xout[idx] = x[0]

t = np.linspace(0, d.size*dt, d.size)
plt.plot(t, d, '.')
plt.plot(t, xout, '.')
plt.show()
