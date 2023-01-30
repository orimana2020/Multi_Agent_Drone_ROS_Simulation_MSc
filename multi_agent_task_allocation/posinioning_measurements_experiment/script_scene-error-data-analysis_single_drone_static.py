import numpy as np
import matplotlib.pyplot as plt


lps_position = np.load('static_pos_fisrt_try.npy')
mean = np.mean(lps_position, axis=0)
print(mean)
var = np.var(lps_position, axis=0)
print(var)
fig = plt.figure()
ax = fig.subplots()

samples = lps_position[:,1]
# var_samples = var[1]

# x = np.linspace(min(samples), max(samples), 100)
# f = (1/(var_samples * 2* np.pi)** 0.5 ) * np.exp(-(x-mean[1])**2 / (2*var_samples))


ax.hist(lps_position[:,1])
# ax.scatter(x,f)
plt.show()


