import numpy as np
import matplotlib.pyplot as plt

x = np.array([[1, 2, 4], [1, 5 ,6]])
y = np.array([[1, 2, 8], [3, 8, 9]])

plt.plot(x, y, color = 'k')

plt.show()

a = np.array([1, 2, 3, 4])
b = np.array([1, 2, 3])

a_nwe = a.reshape((2, 2))

print(a)
print(a_nwe)