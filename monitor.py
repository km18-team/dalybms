#!/usr/bin/python3

# Protocol docs https://github.com/jblance/mpp-solar/blob/master/docs/protocols/DALY-Daly_RS485_UART_Protocol.pdf
# Protocol examples and more info https://diysolarforum.com/threads/decoding-the-daly-smartbms-protocol.21898/

import serial
import time
import os
import paho.mqtt.client as mqtt
import json

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

# 3. Battery Energy (Wh) - New sensor
ENERGY_TOPIC = STATE_TOPIC + '_energy'
energyHaConf = '{"device_class": "energy", "name": "Battery Energy", "state_topic": "' + ENERGY_TOPIC + '/state", "unit_of_measurement": "Wh", "value_template": "{{ value_json.energy }}", "unique_id": "' + devId + '_energy", ' + deviceConf + '}'
client.publish(ENERGY_TOPIC + '/config', energyHaConf, 0, True)

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

# 7. Battery Temperature - uses data from monomer temperature command
TEMP_TOPIC = STATE_TOPIC + '_temp'
tempHaConf = '{"device_class": "temperature", "name": "Battery Temperature", "state_topic": "' + TEMP_TOPIC + '/state", "unit_of_measurement": "°C", "value_template": "{{ value_json.value}}", "unique_id": "' + devId + '_temp", ' + deviceConf + ', "json_attributes_topic": "' + TEMP_TOPIC + '/state"}'
client.publish(TEMP_TOPIC + '/config', tempHaConf, 0, True)

# 8. BMS Life Cycles - Regular entity
BMS_LIFE_TOPIC = STATE_TOPIC + '_bms_life'
bmsLifeHaConf = '{"name": "BMS Life Cycles", "state_topic": "' + BMS_LIFE_TOPIC + '/state", "unit_of_measurement": "cycles", "value_template": "{{ value_json.life }}", "unique_id": "' + devId + '_bms_life", ' + deviceConf + '}'
client.publish(BMS_LIFE_TOPIC + '/config', bmsLifeHaConf, 0, True)

# 10. MOS Status - Regular entity
MOS_TOPIC = STATE_TOPIC + '_mos'
mosHaConf = '{"name": "MOS status", "state_topic": "' + MOS_TOPIC + '/state", "value_template": "{{ value_json.value}}", "unique_id": "' + devId + '_mos", ' + deviceConf + ', "json_attributes_topic": "' + MOS_TOPIC + '/state"}'
client.publish(MOS_TOPIC + '/config', mosHaConf, 0, True)

# Diagnostic entities - using the correct ent_cat format
# Residual Capacity in mAh - Diagnostic entity
CAPACITY_TOPIC = STATE_TOPIC + '_capacity'
capacityHaConf = '{"entity_category": "diagnostic", "device_class": "energy", "name": "Battery Residual Capacity", "state_topic": "' + CAPACITY_TOPIC + '/state", "unit_of_measurement": "mAh", "value_template": "{{ value_json.capacity }}", "unique_id": "' + devId + '_capacity", ' + deviceConf + '}'
client.publish(CAPACITY_TOPIC + '/config', capacityHaConf, 0, True)

# Number of Temperature Sensors - Diagnostic entity
BMS_TEMP_TOPIC = STATE_TOPIC + '_bms_temp'
bmsTemperatureHaConf = '{"entity_category": "diagnostic", "name": "Temperature Sensors Count", "state_topic": "' + BMS_TEMP_TOPIC + '/state", "value_template": "{{ value_json.temperature }}", "unique_id": "' + devId + '_bms_temp", ' + deviceConf + '}'
client.publish(BMS_TEMP_TOPIC + '/config', bmsTemperatureHaConf, 0, True)

# First, add the configuration for the new Wh sensor
ENERGY_TOPIC = STATE_TOPIC + '_energy'
energyHaConf = '{"device_class": "energy", "name": "Battery Energy", "state_topic": "' + ENERGY_TOPIC + '/state", "unit_of_measurement": "Wh", "value_template": "{{ value_json.energy }}", "unique_id": "' + devId + '_energy", ' + deviceConf + '}'
client.publish(ENERGY_TOPIC + '/config', energyHaConf, 0, True)

# Cell Balance (Voltage Delta) - Diagnostic entity
CELLS_TOPIC = STATE_TOPIC + '_balance'
cellsHaConf = '{"entity_category": "diagnostic", "device_class": "voltage", "name": "Battery Cell Balance", "state_topic": "' + CELLS_TOPIC + '/state", "unit_of_measurement": "V", "value_template": "{{ value_json.diff}}", "json_attributes_topic": "' + CELLS_TOPIC + '/state", "unique_id": "' + devId + '_balance", ' + deviceConf + '}'
client.publish(CELLS_TOPIC + '/config', cellsHaConf, 0, True)

current_voltage = 0
residual_capacity_mah = 0

def cmd(command):
    try:
        res = []
        ser.write(command)
        while True:
            s = ser.read(13)
            if (s == b''):
                break
            res.append(s)
        return res
    except serial.SerialException as e:
        print(f"Serial communication error: {e}")
        time.sleep(5)  # Wait before retrying
        return []

def publish(topic, data):
    try:
        client.publish(topic, data, 0, False)
    except Exception as e:
        print("Error sending to mqtt: " + str(e))

def publish_json(topic, data_dict):
    try:
        client.publish(topic, json.dumps(data_dict), 0, False)
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

def get_battery_state():
    global current_voltage, residual_capacity_mah
    res = cmd(b'\xa5\x40\x90\x08\x00\x00\x00\x00\x00\x00\x00\x00\x7d')
    if len(res) < 1:
        print('Empty response get_battery_state')
        return
    buffer = res[0]

    # Extract values from correct byte positions
    # The first 4 bytes are likely header/command bytes
    voltage = int.from_bytes(buffer[4:6], byteorder='big', signed=False) / 10  # Bytes 0-1: Total voltage
    current_voltage = voltage  # Update the global voltage value

    aquisition = int.from_bytes(buffer[6:8], byteorder='big', signed=False) / 10  # Bytes 2-3: Acquisition voltage

    # Current with offset of 30000
    current = (int.from_bytes(buffer[8:10], byteorder='big', signed=False) - 30000) / 10  # Bytes 4-5: Current

    # SOC in 0.1%
    soc = int.from_bytes(buffer[10:12], byteorder='big', signed=False) / 10  # Bytes 6-7: SOC

    json = '{'
    json += '"voltage":' + str(voltage) + ','
    json += '"aquisition":' + str(aquisition) + ','
    json += '"current":' + str(round(current, 1)) + ','
    json += '"soc":' + str(soc)
    json += '}'
    print(json)
    publish(STATE_TOPIC +'/state', json)

    # Calculate and publish energy here too, as a fallback
    if current_voltage > 0 and residual_capacity_mah > 0:
        energy_wh = round((residual_capacity_mah * current_voltage) / 1000, 2)
        energyJson = '{"energy":' + str(energy_wh) + '}'
        publish(ENERGY_TOPIC + '/state', energyJson)

def get_battery_temperature():
    res = cmd(b'\xa5\x40\x96\x08\x00\x00\x00\x00\x00\x00\x00\x00\x83')

    if len(res) >= 1:
        # Process temperature data from all response frames
        temps = []

        for frame in res:
            if len(frame) < 5:  # Make sure we have enough data
                continue

            frame_num = int.from_bytes(frame[4:5], byteorder='big', signed=False)
            print(f"Processing temperature frame {frame_num}")

            # Byte1~byte7 (or less) contain the temperature values in this frame
            # In our data structure, they start at index 5
            for i in range(7):  # Maximum 7 temperature values per frame
                if 5+i < len(frame):
                    temp_value = int.from_bytes(frame[5+i:6+i], byteorder='big', signed=False) - 40

                    # Apply sanity check
                    if -40 <= temp_value <= 100:
                        temps.append(temp_value)
                        print(f"  Temp sensor {len(temps)}: {temp_value}°C")
                    else:
                        print(f"  Invalid temperature reading: {temp_value}°C")

        if temps:
            # Calculate average temperature
            avg_temp = sum(temps) / len(temps)

            # Create temperature data object
            temp_data = {
                "value": round(avg_temp, 1),
                "count": len(temps),
                "min": min(temps),
                "max": max(temps)
            }

            # Add individual temperature values
            for i, temp in enumerate(temps):
                temp_data[f"temp_{i+1}"] = temp

            # Publish as JSON
            publish_json(TEMP_TOPIC + '/state', temp_data)

            # Also publish temperature count to the BMS temp topic
            publish_json(BMS_TEMP_TOPIC + '/state', {"temperature": len(temps)})
            return True

    # Fallback to basic temperature method if detailed method failed
    res = cmd(b'\xa5\x40\x92\x08\x00\x00\x00\x00\x00\x00\x00\x00\x7f')
    if len(res) < 1:
        print('Empty response for temperature data')
        return False

    buffer = res[0]
    maxTemp = int.from_bytes(buffer[4:5], byteorder='big', signed=False) - 40
    maxTempCell = int.from_bytes(buffer[5:6], byteorder='big', signed=False)
    minTemp = int.from_bytes(buffer[6:7], byteorder='big', signed=False) - 40
    minTempCell = int.from_bytes(buffer[7:8], byteorder='big', signed=False)

    # Sanity check
    if not (-40 <= maxTemp <= 100 and -40 <= minTemp <= 100):
        print(f'Invalid temperature data: max={maxTemp}, min={minTemp}')
        return False

    avg_temp = (maxTemp + minTemp) / 2

    temp_data = {
        "value": avg_temp,
        "maxTemp": maxTemp,
        "maxTempCell": maxTempCell,
        "minTemp": minTemp,
        "minTempCell": minTempCell
    }

    publish_json(TEMP_TOPIC + '/state', temp_data)
    return True

def get_battery_mos_status():
    global residual_capacity_mah, current_voltage
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
    residual_capacity_mah = residualCapacity  # Update the global capacity value

    json = '{'
    json += '"value":"' + value + '",'
    json += '"chargingMOS":' + str(chargeMOS) + ','
    json += '"dischargingMOS":' + str(dischargeMOS) + ','
    json += '"BMSLife":' + str(BMSLife) + ','
    json += '"residualCapacity":' + str(residualCapacity)
    json += '}'
    # print(json)
    publish(MOS_TOPIC +'/state', json)

    # Publish the charging status as a separate sensor
    chargeStatusJson = '{"status":"' + value + '"}'
    publish(CHARGE_STATUS_TOPIC + '/state', chargeStatusJson)

    # Calculate and publish the energy in Wh
    if current_voltage > 0:  # Make sure we have a valid voltage
        energy_wh = round((residualCapacity * current_voltage) / 1000, 2)
        energyJson = '{"energy":' + str(energy_wh) + '}'
        publish(ENERGY_TOPIC + '/state', energyJson)

while True:
    try:
        get_battery_state()
        get_cell_balance(int(os.environ['CELL_COUNT']))
        get_battery_temperature()  # New consolidated function
        get_battery_mos_status()
        time.sleep(5)  # Increased polling interval to reduce load
    except Exception as e:
        print(f"Error in main loop: {e}")
        time.sleep(10)  # Wait longer after an error

ser.close()
print('done')
