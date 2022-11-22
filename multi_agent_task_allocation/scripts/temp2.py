import numpy as np
path = [[1.9 ,       0.7 ,       1.        ],
 [1.8      ,  0.7     ,   1.        ],
 [1.8      ,  0.7      ,  1.        ],
 [1.7      ,  0.7    ,    1.        ],
 [1.6      ,  0.7   ,     1.        ]]

a = np.array([1.8      ,  0.700002     ,   1.        ]) / 0.1
b = np.array([1.8      ,  0.7      ,  1.        ]) / 0.1
a = np.round(a)
b = np.round(b)
print((a==b).all())