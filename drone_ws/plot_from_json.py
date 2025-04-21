import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D # <--- This is important for 3d plotting
import numpy as np



filename = 'poses_run6'

with open(filename, "r+") as f:
    r = f.read()
    data = json.loads(r)

local_x, local_y, local_t  = data['local']['x'], data['local']['y'], data['local']['t']
setpoint_x, setpoint_y, setpoint_t  = data['setpoint']['x'], data['setpoint']['y'], data['setpoint']['t']
box_x, box_y, box_t  = data['box']['x'], data['box']['y'], data['box']['t']

# find when we hit the 10 minute mark (600s)
box_t = np.array(box_t)
b = box_t - box_t[0]
b_clip = b[b <= 600]
idx = len(b_clip)

box_t = box_t[:idx] - box_t[0]
box_x = box_x[:idx]
box_y = box_y[:idx]

local_t = np.array(local_t)
b = local_t - local_t[0]
b_clip = b[b <= 600]
idx = len(b_clip)

local_t = local_t[:idx]- local_t[0]
local_x = local_x[:idx]
local_y = local_y[:idx]

setpoint_t = np.array(setpoint_t)
b = setpoint_t - setpoint_t[0]
b_clip = b[b <= 600]
idx = len(b_clip)

setpoint_t = setpoint_t[:idx]
setpoint_x = setpoint_x[:idx]
setpoint_y = setpoint_y[:idx]

ax = plt.figure(0).add_subplot(projection='3d')
# axv = plt.figure().add_subplot(projection='3d')
ax.plot(local_x, local_y, [t - local_t[0] for t in local_t], zdir='z', label='local_pose', c='b')
ax.plot(setpoint_x, setpoint_y, [t - setpoint_t[0] for t in setpoint_t], zdir='z', label='setpoint_pose', c='r')
# ax.plot(zebra_x, zebra_y, [t -zebra_t[0] for t in zebra_t], zdir='z', label='zebra_pose', c='g')
ax.plot(box_x, box_y, [t - box_t[0] for t in box_t], zdir='z', label='box_pose', c='k')
ax.legend()
ax.set_title('X, Y location over time')
ax.set_ylabel('Y coordinate (m)')
ax.set_xlabel('X coordinate (m)')
ax.set_zlabel('Time (s)')
# plt.show()
# l = np.array([[data['local']['x']],[data['local']['y']]])
# b = np.array([[data['box']['x']],[data['box']['y']]])
# error = np.linalg.norm(l - b, axis=1)

# # for i in range(len(data['local']['x'])):
# #     l = np.array([data['local']['x'][i], data['local']['y'][i]])
# #     b = np.array([data['box']['x'][i], data['box']['y'][i]])
# #     error.append(np.linalg.norm(b-l))

# plt.plot(data['local']['t'], error)
# plt.show()

ax1 = plt.figure(1)
box_x_interp = np.interp(local_t, box_t, box_x)
box_y_interp = np.interp(local_t, box_t, box_y)

l = np.array([[*local_x], [*local_y]])
b = np.array([[*box_x_interp], [*box_y_interp]])

error = np.linalg.norm(l - b, axis=0)
print(error)
print(sum(error))

plt.plot(local_t, error, label='error')
plt.legend()
plt.title('Euclidean error over time')
plt.xlabel('Time (s)')
plt.ylabel('Error (m)')
plt.show()