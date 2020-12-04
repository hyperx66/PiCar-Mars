import os
import time
import sys
import paho.mqtt.client as mqtt
import json
import ultra
import math

THINGSBOARD_HOST = '129.126.163.157'
ACCESS_TOKEN = 'AnFwv9i7F6OZo3q0jJcx'

# Data capture and upload interval in seconds. Less interval will eventually hang the DHT22.
INTERVAL=2

sensor_data = {'distance': 0, 'angle':0}

next_reading = time.time()

client = mqtt.Client()

# Set access token
client.username_pw_set(ACCESS_TOKEN)

# Connect to ThingsBoard using default MQTT port and 60 seconds keepalive interval
client.connect(THINGSBOARD_HOST, 1883, 60)

client.loop_start()

try:
    while True:
        #distance
        sensor_data['distance'] = ultra.checkdist()
        #angle
        angle = 2*(math.atan((1.5*27)/2*(ultra.checkdist())))
        sensor_data['angle'] = angle
        
        client.publish('v1/devices/me/telemetry', json.dumps(sensor_data), 1)
        next_reading += INTERVAL
        sleep_time = next_reading-time.time()
        if sleep_time > 0:
            time.sleep(sleep_time)
except KeyboardInterrupt:
    pass


client.loop_stop()
client.disconnect()