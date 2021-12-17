# https://mmas.github.io/interpolation-scipy

import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt


x = [0, 0.01,0.02,0.03]
y = [0, 0.03, 0.03, 0]

xn = np.arange(0, 0.031, 0.001)
y0 = [0, 0.03, 0.03, 0]


f = interpolate.PchipInterpolator(x, y)
# f = interpolate.interp1d(x, y, kind='nearest')
# f = interpolate.lagrange(x, y)
# f = interpolate.interp1d(x, y, kind='linear')
# f = interpolate.BarycentricInterpolator(x, y)
# f = interpolate.interp1d(x, y, kind='quadratic')
# f = interpolate.interp1d(x, y, kind='cubic')


yn = f(xn)

Px = 0.015
z_hat = f(Px)
plt.plot(Px, z_hat, 'ro', label='punto muestreo')

plt.plot(x, y, 'ok', label='nudos')
plt.plot(xn, yn, label='valores interpolados')
plt.legend()
plt.show()