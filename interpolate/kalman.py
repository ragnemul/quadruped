# https://pythonnumericalmethods.berkeley.edu/notebooks/chapter17.05-Newtons-Polynomial-Interpolation.html
import matplotlib
import numpy as np
import matplotlib.pyplot as plt
import math
import filterpy.stats as stats

plt.style.use('seaborn-poster')

data = np.loadtxt('data.csv', delimiter=',')[0:50].astype(int)

x = np.arange(1, len(data) + 1, 1)
fs = [x[1] for x in data]
new_x = range(math.floor(min(x)), math.ceil(max(x))+1,5)


plt.figure(figsize=(12, 8))
plt.plot(x, fs, marker='o', linestyle='--', color='b', label='FSR1')
plt.title('Datos FSR')
plt.xlabel('medidas')
plt.ylabel('valor FSR')
plt.legend()
plt.xticks(new_x)
plt.show()

# media de la población
media = np.mean(fs, axis=0, dtype=np.float64)
# varianza de la población
var = np.var(fs, axis=0, dtype=np.float64, ddof=1)
# desviación estandar de la población
stdev = np.std(fs, axis=0, dtype=np.float64, ddof=1)

# pdf de FS1
stats.plot_gaussian_pdf(mean=media, variance=var,
                        xlim=(min(fs)-var, max(fs)+var), ylim=(0, 0.12));
plt.show()



from random import random
from collections import namedtuple
import book_plots



def update(prior, measurement):
    x, P = prior  # mean and variance of prior
    z, R = measurement  # mean and variance of measurement

    y = z - x  # residual
    K = P / (P + R)  # Kalman gain

    x = x + K * y  # posterior
    P = (1 - K) * P  # posterior variance
    return gaussian(x, P)


def predict(posterior, movement):
    x, P = posterior  # mean and variance of posterior
    dx, Q = movement  # mean and variance of movement
    x = x + dx
    P = P + Q
    return gaussian(x, P)



gaussian = namedtuple('Gaussian', ['mean', 'var'])
gaussian.__repr__ = lambda s: '𝒩(μ={:.3f}, 𝜎²={:.3f})'.format(s[0], s[1])

process_var = 0.000001

x = gaussian(fs[0], var)  # initial state
process_model = gaussian(0., process_var)

zs = fs
ps = []
estimates = []

for z in zs:
    prior = predict(x, process_model)
    x = update(prior, gaussian(z, var))

    # save for latter plotting
    estimates.append(x.mean)
    ps.append(x.var)

# plot the filter output and the variance
book_plots.plot_measurements(zs)
book_plots.plot_filter(estimates, var=np.array(ps))
book_plots.show_legend()
plt.ylim(min(fs)-5, max(fs)+5)
book_plots.set_labels(x='medidas', y='FSR')
plt.show()

plt.plot(ps)
plt.title('Variance')
print('Variance converges to {:.3f}'.format(ps[-1]))
plt.show()