import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

fig, ax = plt.subplots()

x = np.linspace(0, 2 * np.pi, 100)
y = np.linspace(0, 2 * np.pi, 100).reshape(-1, 1)

ims = []
for i in range(100):
    x += np.pi / 20
    y += np.pi / 20
    im = ax.imshow(np.sin(x) + np.cos(y), animated=True)
    if i == 0:
        ax.imshow(np.sin(x) + np.cos(y))
    ims.append([im])

ani = animation.ArtistAnimation(fig, ims, interval=100)

plt.show()