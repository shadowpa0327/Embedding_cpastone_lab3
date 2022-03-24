import matplotlib.pyplot as plt
import numpy as np
data = np.load('magnet_data_lab3.npy')
print(data)




plt.plot(data[0],data[1])
plt.show()
