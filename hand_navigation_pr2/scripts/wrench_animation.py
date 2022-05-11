import pandas as pd
import matplotlib.pyplot as plt 
import matplotlib.animation as animation

data = pd.read_csv("wrench.csv")
data = data.iloc[:, 1:]

fig, ax = plt.subplots()

ax.set_xlim(0, len(data.index))
ax.set_ylim(data.values.min(), data.values.max())

colors=['blue', 'green','red']
plt.gca().set_prop_cycle(color=colors)
frames = []
for i in range(len(data.index)):
    sub = data.iloc[0:i+1, :]
    lines = ax.plot(sub)
    frames.append(lines)

ani = animation.ArtistAnimation(fig, frames, interval=30)

plt.show()
