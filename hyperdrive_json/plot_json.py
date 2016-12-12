import json
import matplotlib.pyplot as plt
import numpy as np
from optparse import OptionParser

parser = OptionParser()
parser.add_option("-f", "--file", dest="file",
                  help="write report to FILE", metavar="file")

(options, args) = parser.parse_args()

# data_file = "1481558375.249912.json"
data_file = options.file

def json_fix(filename):
    json_raw = open(filename, 'r').read()
    json_data = ''
    if json_raw[1] != '[' and json_raw[-2] == ',':
        json_data = '[' + json_raw[0:-2] + ']'
    return json.loads(json_data)

data = json_fix(data_file)

N = len(data)
time = np.zeros(N)
speed = np.zeros(N)
rThrottle = np.zeros(N)
aSteerWheel = np.zeros(N)
BBrake = np.zeros(N, dtype=np.bool)
nEngine = np.zeros(N)
gLong = np.zeros(N)
nYaw = np.zeros(N)

for i, sample in enumerate(data):
    time[i] = sample[1]
    rThrottle[i] = sample[2]
    BBrake[i] = sample[3] 
    aSteerWheel[i] = sample[8]
    nEngine[i] = sample[12]
    speed[i] = sample[13]
    gLong[i] = sample[14]
    nYaw[i] = sample[16]

plt.figure()
ax1 = plt.subplot(4,1,1)		
plt.plot(time/1000, aSteerWheel, '-o')
plt.ylabel('aSteerWheel (deg)')
plt.xlabel('Time (s)')

ax2 = plt.subplot(4,1,2, sharex=ax1)
plt.plot(time/1000, rThrottle, '-o')
plt.ylabel('rThrottle (%)')
plt.xlabel('Time (s)')

ax3 = plt.subplot(4,1,3, sharex=ax1)
plt.plot(time/1000, nEngine, '-o')
plt.ylabel('nEngine (RPM)')
plt.xlabel('Time (s)')

ax4 = plt.subplot(4,1,4, sharex=ax1)
plt.plot(time/1000, speed, '-o')
plt.ylabel('Speed (kph)')
plt.xlabel('Time (s)')

plt.show()


