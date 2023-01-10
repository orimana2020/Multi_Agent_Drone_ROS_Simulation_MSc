import numpy as np


dic2 = np.load('task_38_data.npy',allow_pickle=True)
dic2 = dic2.item()

for dic in dic2:
    print(dic)

