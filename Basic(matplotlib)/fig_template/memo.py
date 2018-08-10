
import numpy as np
import matplotlib.pyplot as plt

x = np.array([[1, 2, 4], [1, 1 ,6]])
y = np.array([[1, 2, 5], [6, 1, 9]])

plt.plot(x, y, color = 'k')

plt.show()

a = np.array([1, 2, 3, 4])
b = np.array([1, 2, 3])

a_nwe = a.reshape((2, 2))

print(a)
print(a_nwe)

'''
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

step = np.pi / 10.0
xs = np.arange(0, np.pi * 2, step)

fig = plt.figure()
imgs = []

for i in range(len(xs)):
    lines = []
    for x in xs[0:i+1]:
        y = np.sin(x)
        img = plt.plot([x, x + step], [y, y], 'b-o')
        lines.extend(img)
    imgs.append(lines)

anim = animation.ArtistAnimation(fig, imgs, interval=100)
anim.save('result4.gif', 'imagemagick')
plt.show()
'''