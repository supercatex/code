import matplotlib.pyplot as plt

with open("entropy_100_2.txt", "r") as f:
    data = [int(x.strip()) for x in f.readlines()]

x, y = [], []
for i in range(max(data) + 1):
    x.append(i)
    y.append(0)

sum_y = 0
plt.ion()
for i in range(len(data)):
    plt.cla()

    y[data[i]] += 1
    sum_y += data[i]
    if i % 10 == 0 or i == len(data) - 1:
        plt.bar(x, y, color="b")
        plt.ylim(0, 450)
        for j in range(len(x)):
            plt.text(x[j]-.25, y[j]+1.5, str(y[j]), color="blue")
        plt.text(0, 430, "Avg: %.4f" % (sum_y / (i + 1)), color="blue")
        plt.pause(0.001)
    if i == 0:
        plt.pause(1)
plt.ioff()
plt.show()
