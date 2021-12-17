from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import numpy as np

plt.style.use('seaborn-poster')
y = [0, 0.01, 0.02, 0.03]
z = [0, 0.03, 0.03, 0]

Px = 0.015

f = interp1d(y, z, kind="quadratic")
z_hat = f(Px)
print(z_hat)

x_new = np.arange(0, 0.031, 0.001)
y_new = f(x_new)

plt.figure(figsize = (10,8))
plt.plot(y, z, '--ob')
plt.plot(Px, z_hat, 'ro')
plt.plot(x_new, y_new,'g')
plt.title('Linear Interpolation at y = ' + str(Px) + ': z= ' + str(np.round(z_hat,3)))
plt.xlabel('y')
plt.ylabel('z')
plt.show()