import json
import matplotlib.pyplot as plt
import numpy as np
from optparse import OptionParser
from lists_dictionary import makeDictOfArraysFromLists

parser = OptionParser()
parser.add_option("-f", "--file", dest="file",
                  help="write report to FILE", metavar="file")

(options, args) = parser.parse_args()
data_file = options.file

def json_fix(filename):
    json_raw = open(filename, 'r').read()
    json_data = ''
    if json_raw[1] != '[' and json_raw[-2] == ',':
        json_data = '[' + json_raw[0:-2] + ']'
    return json.loads(json_data)

def int_OFF_ON(str):
    if str == 'OFF':
        return 0
    elif str == 'ON':
        return 1
    return -1

def int_PRNDB(str):
    if str == 'P':
        return 0
    elif str == 'R':
        return 1
    elif str == 'N':
        return 2
    elif str == 'D':
        return 3
    elif str == 'B':
        return 4
    return -1

names = {
'time_ms': 1,
'accel_pedal': 2,
'brake_pedal': 3,
'at_shift': 5,
'steer_wheel': 8,
'engine_rpm': 12,
'speed_kph': 13,
'accel_long': 14,
'yaw_rate': 16,
'odometer': 17,
'fuel_mL': 18,
'lat_deg': 24,
'lon_deg': 26,
'altitude': 30,
'ev_mode': 43,
'battery_soc': 44
}

data = json_fix(data_file)
data_dict = makeDictOfArraysFromLists(data, names)

time = data_dict['time_ms']/1000
accel_pedal = data_dict['accel_pedal']
steer_wheel = data_dict['steer_wheel']
engine_rpm = data_dict['engine_rpm']
speed_kph = data_dict['speed_kph']
accel_long = data_dict['accel_long']
yaw_rate = data_dict['yaw_rate']
odometer = data_dict['odometer']
fuel_mL = data_dict['fuel_mL']
lat_deg = data_dict['lat_deg']
lon_deg = data_dict['lon_deg']
altitude = data_dict['altitude']
battery_soc = data_dict['battery_soc']
brake_pedal = np.array([int_OFF_ON(str) for str in data_dict['brake_pedal']])
ev_mode = np.array([int_OFF_ON(str) for str in data_dict['ev_mode']])
at_shift = np.array([int_PRNDB(str) for str in data_dict['at_shift']])

plt.figure()
ax1 = plt.subplot(3,2,1)
plt.plot(time, steer_wheel, '-o')
plt.ylabel('Steering Wheel Angle (deg)')
plt.xlabel('Time (s)')

ax2 = plt.subplot(3,2,3, sharex=ax1)
plt.plot(time, accel_pedal, '-o')
plt.ylabel('Accelerator Position (%)')
plt.xlabel('Time (s)')

ax3 = plt.subplot(3,2,5, sharex=ax1)
plt.plot(time, brake_pedal, '-o')
plt.ylabel('Brake Pedal Status (0-1)')
plt.xlabel('Time (s)')

ax4 = plt.subplot(2,2,2, sharex=ax1)
plt.plot(time, at_shift, '-o')
plt.ylabel('AT Shift Position (0-4)')
plt.xlabel('Time (s)')

ax5 = plt.subplot(2,2,4, sharex=ax1)
plt.plot(time, ev_mode, '-o')
plt.ylabel('EVM_MSG (0-1)')
plt.xlabel('Time (s)')

plt.figure()
ax6 = plt.subplot(2,2,1, sharex=ax1)
plt.plot(time, engine_rpm, '-o')
plt.ylabel('Engine Speed (RPM)')
plt.xlabel('Time (s)')

ax7 = plt.subplot(2,2,2, sharex=ax1)
plt.plot(time, speed_kph, '-o')
plt.ylabel('Vehicle Speed (kph)')
plt.xlabel('Time (s)')

ax8 = plt.subplot(2,2,3, sharex=ax1)
plt.plot(time, accel_long, '-o')
plt.ylabel('Long Accel. (m/s2)')
plt.xlabel('Time (s)')

ax9 = plt.subplot(2,2,4, sharex=ax1)
plt.plot(time, yaw_rate, '-o')
plt.ylabel('Yaw-rate (deg/s)')
plt.xlabel('Time (s)')

plt.figure()
ax10 = plt.subplot(2,2,1, sharex=ax1)
plt.plot(time, odometer, '-o')
plt.ylabel('Odometer (m)')
plt.xlabel('Time (s)')

ax11 = plt.subplot(2,2,2, sharex=ax1)
plt.plot(time, fuel_mL, '-o')
plt.ylabel('Fuel Consumption (mL)')
plt.xlabel('Time (s)')

ax12 = plt.subplot(2,2,3, sharex=ax1)
plt.plot(time, battery_soc, '-o')
plt.ylabel('Battery SOC (%)')
plt.xlabel('Time (s)')

ax13 = plt.subplot(2,2,4, sharex=ax1)
plt.plot(time, lat_deg, '-o', time, lon_deg, '-o')
plt.ylabel('GPS (deg)')
plt.xlabel('Time (s)')
plt.legend(['Lat','Long'])

plt.show()
