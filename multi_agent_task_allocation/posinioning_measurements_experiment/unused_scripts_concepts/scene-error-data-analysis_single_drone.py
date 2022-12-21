import numpy as np
import os
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans


lps_position = np.load('cf_postion_errors_experiment2.npy')
goals = np.load('goals.npy')
samples_num = 20
measurments = []
i = 0

# analysing vicon data
# find k measurements closest to some goal
kmeans = KMeans(n_clusters=len(goals)).fit(lps_position)
centers = kmeans.cluster_centers_
first_center = np.array([x for x in lps_position if np.linalg.norm(centers[0] - x, ord=2) < 0.05])
avg = np.average(first_center, axis=0)
print(avg)
# preprocess data 