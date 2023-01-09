import numpy as np

s = np.random.normal(0.17, 0.015)
coords = np.array([1,2,4])
random_err = np.random.normal(0.15, 0.015)
new_coords = coords + np.array([np.random.choice([-1,1]) * random_err/2**0.5, 
np.random.choice([-1,1]) * random_err/2**0.5,
np.random.choice([-1,1]) * np.random.normal(0.05, 0.005) ])
