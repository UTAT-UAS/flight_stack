import matplotlib.pyplot as plt
import numpy as np

with open("/home/uas/workspace/uas_ws/src/flight_stack/scripts/jerk_tuning/logs/log-2026-05-09T22:03:32.829994.csv", 'r') as f:
    data = f.read()
data = data.replace('\n', ',')
data = data.split(',')
columns = 13
data = [data[columns*i:columns*i+columns] for i in range(len(data)//columns)]

legend = data[0]
data = data[2:]
data_t = np.transpose(data)
print(data_t[-1])

#data_t = data_t[1:] # corrupted columns bruh
data = []
# pathtime, mj_vx, mj_vy, cc_tgt, px, py, pz, vx, vy, vz, ax, ay, az
enables = [0,0,0,0,0,0,0,0,0,0,1,1,0]
for i, series in enumerate(data_t):
    if not enables[i]: continue
    series = series = [float(x) for x in series]
    block = 1
    series = [sum(series[block*i:block*i+block]) / block for i in range(len(series) // block)]
    #seriesmin = [max(series[block*i:block*i+block]) / block for i in range(len(series) // block)]
    #seriesmax = [min(series[block*i:block*i+block]) / block for i in range(len(series) // block)]
    #data.append(series)
    #data.append(seriesmin)
    #data.append(seriesmax)
    #series = [series[i] for i in range(len(series) // 2 - 500, len(series) // 2 + 500)]
    data.append(series)

print(len(data[0]))
acc = data[-2]
data.append([acc[i + 1] - acc[i] for i in range(len(acc) - 1)])
acc = data[-2]
data.append([acc[i + 1] - acc[i] for i in range(len(acc) - 1)])

for i, series in enumerate(data):
    plt.plot(series, label=i)
plt.legend()
plt.show()
