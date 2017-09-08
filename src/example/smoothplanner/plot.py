import matplotlib.pyplot as plt
import sys

filename = sys.argv[1]
filename2 = sys.argv[2]
filename3 = sys.argv[3]
t = []
x = []
dx = []
ddx = []

data = open(filename)
for line in data:
    temp = eval(line)
    t.append(temp[1])
    x.append(temp[0])
data.close()

data = open(filename2)
for line in data:
    temp = eval(line)
    dx.append(temp[0])
data.close()

data = open(filename3)
for line in data:
    temp = eval(line)
    ddx.append(temp[0])
data.close()
fig = plt.figure(figsize=(8,5))
axes = fig.add_subplot(311)
axes.plot(t, x, label="$ x_t $", color="blue", linewidth=1)
axes.grid()
axes.set_xlabel("t")
axes.set_ylabel("x")
axes = fig.add_subplot(312)
axes.plot(t, dx, label="$ dx_t $", color="blue", linewidth=1)
axes.grid()
axes.set_xlabel("t")
axes.set_ylabel("dx")
axes = fig.add_subplot(313)
axes.plot(t, ddx, label="$ ddx_t $", color="blue", linewidth=1)
axes.grid()
axes.set_xlabel("t")
axes.set_ylabel("ddx")
plt.show()
