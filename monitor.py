#!/usr/bin/python3

# Protocol docs https://github.com/jblance/mpp-solar/blob/master/docs/protocols/DALY-Daly_RS485_UART_Protocol.pdf
# Protocol examples and more info https://diysolarforum.com/threads/decoding-the-daly-smartbms-protocol.21898/

import serial
import binascii
import time
import os
import paho.mqtt.client as mqtt

print('Starting BMS monitor...')

# ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # open serial port
ser = serial.Serial(os.environ['DEVICE'], 9600, timeout=1)  # open serial port

# connect to MQTT server
client = mqtt.Client(client_id=os.environ['MQTT_CLIENT_ID'])
client.username_pw_set(os.environ['MQTT_USER'], os.environ['MQTT_PASS'])
client.connect(os.environ['MQTT_SERVER'])

devId = os.environ['DEVICE_ID']
BASE_TOPIC = os.environ['MQTT_DISCOVERY_PREFIX'] + '/sensor/'
STATE_TOPIC = BASE_TOPIC + devId
STATUS_TOPIC = STATE_TOPIC + '_status'
deviceConf = '"device": {"manufacturer": "Dongfuan Daly Electronics", "name": "Smart BMS", "identifiers": ["' + devId + '"]}'

# 1. Battery SOC - Regular entity
socHaConf = '{"device_class": "battery", "name": "Battery SOC", "state_topic": "' + STATE_TOPIC +'/state", "unit_of_measurement": "%", "value_template": "{{ value_json.soc}}", "unique_id": "' + devId + '_soc", ' + deviceConf + ', "json_attributes_topic": "' + STATUS_TOPIC + '/state"}'
client.publish(STATE_TOPIC +'_soc/config', socHaConf, 0, True)

# 2. Battery Cycles - Regular entity
CYCLES_TOPIC = STATE_TOPIC + '_cycles'
cyclesHaConf = '{"name": "Battery Cycles", "state_topic": "' + CYCLES_TOPIC + '/state", "unit_of_measurement": "cycles", "value_template": "{{ value_json.cycles }}", "unique_id": "' + devId + '_cycles", ' + deviceConf + '}'
client.publish(CYCLES_TOPIC + '/config', cyclesHaConf, 0, True)

# 3. Residual Capacity - Regular entity
CAPACITY_TOPIC = STATE_TOPIC + '_capacity'
capacityHaConf = '{"device_class": "energy", "name": "Battery Residual Capacity", "state_topic": "' + CAPACITY_TOPIC + '/state", "unit_of_measurement": "mAh", "value_template": "{{ value_json.capacity }}", "unique_id": "' + devId + '_capacity", ' + deviceConf + '}'
client.publish(CAPACITY_TOPIC + '/config', capacityHaConf, 0, True)

# 4. Charging Status - Regular entity
CHARGE_STATUS_TOPIC = STATE_TOPIC + '_charge_status'
chargeStatusHaConf = '{"device_class": "enum", "name": "Battery Charging Status", "state_topic": "' + CHARGE_STATUS_TOPIC + '/state", "value_template": "{{ value_json.status }}", "options": ["idle", "charging", "discharging"], "unique_id": "' + devId + '_charge_status", ' + deviceConf + '}'
client.publish(CHARGE_STATUS_TOPIC + '/config', chargeStatusHaConf, 0, True)

# 5. Battery Voltage - Regular entity
voltageHaConf = '{"device_class": "voltage", "name": "Battery Voltage", "state_topic": "' + STATE_TOPIC +'/state", "unit_of_measurement": "V", "value_template": "{{ value_json.voltage}}", "unique_id": "' + devId + '_voltage", ' + deviceConf + '}'
client.publish(STATE_TOPIC + '_voltage/config', voltageHaConf, 0, True)

# 6. Battery Current - Regular entity
currentHaConf = '{"device_class": "current", "name": "Battery Current", "state_topic": "' + STATE_TOPIC +'/state", "unit_of_measurement": "A", "value_template": "{{ value_json.current}}", "unique_id": "' + devId + '_current", ' + deviceConf + '}'
client.publish(STATE_TOPIC + '_current/config', currentHaConf, 0, True)

# 7. Battery Temperature - Regular entity
TEMP_TOPIC = STATE_TOPIC + '_temp'
tempHaConf = '{"device_class": "temperature", "name": "Battery Temperature", "state_topic": "' + TEMP_TOPIC + '/state", "unit_of_measurement": "°C", "value_template": "{{ value_json.value}}", "unique_id": "' + devId + '_temp", ' + deviceConf + ', "json_attributes_topic": "' + TEMP_TOPIC + '/state"}'
client.publish(TEMP_TOPIC + '/config', tempHaConf, 0, True)

# 8. BMS Life Cycles - Regular entity
BMS_LIFE_TOPIC = STATE_TOPIC + '_bms_life'
bmsLifeHaConf = '{"name": "BMS Life Cycles", "state_topic": "' + BMS_LIFE_TOPIC + '/state", "unit_of_measurement": "cycles", "value_template": "{{ value_json.life }}", "unique_id": "' + devId + '_bms_life", ' + deviceConf + '}'
client.publish(BMS_LIFE_TOPIC + '/config', bmsLifeHaConf, 0, True)

# 9. Number of Temperature Sensors - Diagnostic entity
BMS_TEMP_TOPIC = STATE_TOPIC + '_bms_temp'
bmsTemperatureHaConf = '{"entity_category": "diagnostic", "name": "Temperature Sensors Count", "state_topic": "' + BMS_TEMP_TOPIC + '/state", "value_template": "{{ value_json.temperature }}", "unique_id": "' + devId + '_bms_temp", ' + deviceConf + '}'
client.publish(BMS_TEMP_TOPIC + '/config', bmsTemperatureHaConf, 0, True)

# 10. MOS Status - Regular entity
MOS_TOPIC = STATE_TOPIC + '_mos'
mosHaConf = '{"name": "MOS status", "state_topic": "' + MOS_TOPIC + '/state", "value_template": "{{ value_json.value}}", "unique_id": "' + devId + '_mos", ' + deviceConf + ', "json_attributes_topic": "' + MOS_TOPIC + '/state"}'
client.publish(MOS_TOPIC + '/config', mosHaConf, 0, True)

# 11. Cell Balance (Voltage Delta) - Diagnostic entity
CELLS_TOPIC = STATE_TOPIC + '_balance'
cellsHaConf = '{"entity_category": "diagnostic", "device_class": "voltage", "name": "Battery Cell Balance", "state_topic": "' + CELLS_TOPIC + '/state", "unit_of_measurement": "V", "value_template": "{{ value_json.diff}}", "json_attributes_topic": "' + CELLS_TOPIC + '/state", "unique_id": "' + devId + '_balance", ' + deviceConf + '}'
client.publish(CELLS_TOPIC + '/config', cellsHaConf, 0, True)

def cmd(command):
    res = []
    ser.write(command)
    while True:
        s = ser.read(13)
        if (s == b''):
            break
        # print(binascii.hexlify(s, ' '))
        res.append(s)
    return res

def publish(topic, data):
    try:
        client.publish(topic, data, 0, False)
    except Exception as e:
        print("Error sending to mqtt: " + str(e))

def extract_cells_v(buffer):
    return [
        int.from_bytes(buffer[5:7], byteorder='big', signed=False),
        int.from_bytes(buffer[7:9], byteorder='big', signed=False),
        int.from_bytes(buffer[9:11], byteorder='big', signed=False)
    ]

def get_cell_balance(cell_count):
    res = cmd(b'\xa5\x40\x95\x08\x00\x00\x00\x00\x00\x00\x00\x00\x82')
    if len(res) < 1:
        print('Empty response get_cell_balance')
        return
    cells = []
    for frame in res:
        cells += extract_cells_v(frame)
    cells = cells[:cell_count]
    json = '{'
    sum = 0
    for i in range(cell_count):
        cells[i] = cells[i]/1000
        sum += cells[i]
        json += '"cell_' + str(i+1) + '":' + str(cells[i]) + ','
    json += '"sum":' + str(round(sum, 1)) + ','
    json += '"avg":' + str(round(sum/16, 3)) + ','
    min_v = min(cells)
    max_v = max(cells)
    json += '"min":' + str(min_v) + ','
    json += '"minCell":' + str(cells.index(min_v) + 1) + ','
    json += '"max":' + str(max_v) + ','
    json += '"maxCell":' + str(cells.index(max_v) + 1) + ','
    json += '"diff":' + str(round(max_v - min_v, 3))
    json += '}'
    # print(json)
    publish(CELLS_TOPIC + '/state', json)

def get_battery_status():
    res = cmd(b'\xa5\x40\x94\x08\x00\x00\x00\x00\x00\x00\x00\x00\x81')
    if len(res) < 1:
        print('Empty response get_battery_status')
        return
    buffer = res[0]
    batt_string = int.from_bytes(buffer[4:5], byteorder='big', signed=False)
    # this temperature seems to be the number of temperature sensors
    temp_sensors = int.from_bytes(buffer[5:6], byteorder='big', signed=False)
    charger = 'true' if int.from_bytes(buffer[6:7], byteorder='big', signed=False) == 1 else 'false'
    load = 'true' if int.from_bytes(buffer[7:8], byteorder='big', signed=False) == 1 else 'false'
    # dido = buffer[8:9]
    cycles = int.from_bytes(buffer[9:11], byteorder='big', signed=False)

    json = '{'
    json += '"batt_string":' + str(batt_string) + ','
    json += '"temp_sensors":' + str(temp_sensors) + ','
    json += '"charger":' + charger + ','
    json += '"load":' + load + ','
    json += '"cycles":' + str(cycles)
    json += '}'
    # print(json)
    publish(STATUS_TOPIC +'/state', json)

    # Add this to publish the cycles as a separate sensor
    cyclesJson = '{"cycles":' + str(cycles) + '}'
    publish(CYCLES_TOPIC + '/state', cyclesJson)

    # Add this to publish the number of temperature sensors
    bmsTemperatureJson = '{"temperature":' + str(temp_sensors) + '}'
    publish(BMS_TEMP_TOPIC + '/state', bmsTemperatureJson)

def get_battery_temp():
    res = cmd(b'\xa5\x40\x92\x08\x00\x00\x00\x00\x00\x00\x00\x00\x7f')
    if len(res) < 1:
        print('Empty response get_battery_temp')
        return
    buffer = res[0]
    maxTemp = int.from_bytes(buffer[4:5], byteorder='big', signed=False) - 40
    maxTempCell = int.from_bytes(buffer[5:6], byteorder='big', signed=False)
    minTemp = int.from_bytes(buffer[6:7], byteorder='big', signed=False) - 40
    minTempCell = int.from_bytes(buffer[7:8], byteorder='big', signed=False)

    json = '{'
    json += '"value":' + str((maxTemp + minTemp) / 2) + ','
    json += '"maxTemp":' + str(maxTemp) + ','
    json += '"maxTempCell":' + str(maxTempCell) + ','
    json += '"minTemp":' + str(minTemp) + ','
    json += '"minTempCell":' + str(minTempCell)
    json += '}'
    # print(json)
    publish(TEMP_TOPIC +'/state', json)

def get_battery_mos_status():
    res = cmd(b'\xa5\x40\x93\x08\x00\x00\x00\x00\x00\x00\x00\x00\x80')
    if len(res) < 1:
        print('Empty response get_battery_mos_status')
        return
    buffer = res[0]
    valueByte = int.from_bytes(buffer[4:5], byteorder='big', signed=False)
    value = 'discharging' if valueByte == 2 else ('charging' if valueByte == 1 else 'idle')
    chargeMOS = int.from_bytes(buffer[5:6], byteorder='big', signed=False)
    dischargeMOS = int.from_bytes(buffer[6:7], byteorder='big', signed=False)
    BMSLife = int.from_bytes(buffer[7:8], byteorder='big', signed=False)
    residualCapacity = int.from_bytes(buffer[8:12], byteorder='big', signed=False)

    json = '{'
    json += '"value":"' + value + '",'
    json += '"chargingMOS":' + str(chargeMOS) + ','
    json += '"dischargingMOS":' + str(dischargeMOS) + ','
    json += '"BMSLife":' + str(BMSLife) + ','
    json += '"residualCapacity":' + str(residualCapacity)
    json += '}'
    # print(json)
    publish(MOS_TOPIC +'/state', json)

    # Add this to publish the charging status as a separate sensor
    chargeStatusJson = '{"status":"' + value + '"}'
    publish(CHARGE_STATUS_TOPIC + '/state', chargeStatusJson)

    # Add this to publish the residual capacity as a separate sensor
    capacityJson = '{"capacity":' + str(residualCapacity) + '}'
    publish(CAPACITY_TOPIC + '/state', capacityJson)

while True:
    get_battery_status()
    get_cell_balance(int(os.environ['CELL_COUNT']))
    get_battery_status()
    get_battery_temp()
    get_battery_mos_status()
    time.sleep(1)

ser.close()
print('done')
