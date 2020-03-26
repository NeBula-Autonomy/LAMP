import rosbag
import numpy as np
import matplotlib
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42
matplotlib.rcParams.update({'font.size': 16})
import matplotlib.pyplot as plt

bag = rosbag.Bag('/home/costar/map_points.bag')

time = []
n_points = []
surface_area = []

for topic, msg, t in bag.read_messages(topics=['/base1/lamp/map_data']):
    time.append(t.to_sec())
    n_points.append(int(float(msg.data[5:])/1000))
    surface_area.append(float(n_points[-1])*1000.0*0.1*0.1)


bag.close()

#normalize time
time = time - np.min(time)

fig,ax1 = plt.subplots()
color = 'b'
plt.plot(time,n_points, color=color, linewidth = 4)
plt.xlabel('time(s)')
ax1.set_ylabel('Number of point cloud points (x1000)', color=color)  # we already handled the x-label with ax1
ax1.tick_params(axis='y', labelcolor=color)
ax1.grid()

ax3 = ax1.twinx()
color = 'b'
plt.plot(time,surface_area, color=color, linewidth = 4)
plt.xlabel('time(s)')
ax3.set_ylabel('Surface area explored (m^2)', color=color)  # we already handled the x-label with ax1
ax3.tick_params(axis='y', labelcolor=color)
plt.tight_layout()

plt.show()