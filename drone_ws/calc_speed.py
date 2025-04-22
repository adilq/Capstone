import numpy as np
import json


fname = "poses_eval9"
with open(fname, "r+") as f:
    r = f.read()
    data = json.loads(r)

local_x, local_y, local_t  = data['local']['x'], data['local']['y'], data['local']['t']
setpoint_x, setpoint_y, setpoint_t  = data['setpoint']['x'], data['setpoint']['y'], data['setpoint']['t']
box_x, box_y, box_t  = data['box']['x'], data['box']['y'], data['box']['t']

speeds = []
for i in range(1, len(box_t)):
    # print(np.array([[box_x[i-1]],[box_y[i-1]]]), np.array([[box_x[i]],[box_y[i]]]))
    distance = np.linalg.norm(np.array([[box_x[i-1]],[box_y[i-1]]]) - np.array([[box_x[i]],[box_y[i]]]))
    time_diff = box_t[i] - box_t[i-1]
    speeds.append(distance/time_diff)
    
avg_speed = np.average(speeds)
print(avg_speed)