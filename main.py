import machine
import time
from machine import Pin
from mqtt import MQTTClient
import ubinascii
import micropython
import rp2
from array import array
import secrets
# SETTINGS

moistureSensor = machine.ADC(26)
led = Pin(16, Pin.OUT)
relayPin = Pin(14, Pin.OUT)
relayPin.on()


MANUAL_PUMP_STATUS = False
MOISTURE_THRESHOLD = 40
PUMP_STATUS = False

UPLOAD_FREQUENCY = 60 * 5 # Frequency in seconds to upload moisture data
PUMP_ACTIVATION_INTERVAL = 60 # Frequency in seconds between pump cycles
PUMP_ACTIVATION_DURATION = 5 # Time in seconds that the pump should be active 


# WIFI
WIFI_SSID = secrets.WIFI_SSID
WIFI_PASSWORD = secrets.WIFI_PASSWORD



# Adafruit IO (AIO) Configuration
AIO_SERVER = "io.adafruit.com"
AIO_PORT = 1883
AIO_USERNAME = secrets.AIO_USERNAME
AIO_KEY = secrets.AIO_KEY
AIO_CLIENT_ID = ubinascii.hexlify(machine.unique_id())
AIO_SOIL_MOISTURE_FEED = "hiruzh/feeds/soil-moisture"
# END SETTINGS

# FUNCTIONS

# Function to connect pico to wifi
def do_connect():
    import network
    from time import sleep
    import machine
    wlan = network.WLAN(network.STA_IF) # Put modem in station mode
    if not wlan.isconnected(): # If not connected to wifi
        print('connecting to network...')
        wlan.active(True) 
        # set power mode to get WIfFi power saving off if needed
        wlan.config(pm = 0xa11140)
        wlan.connect(WIFI_SSID, WIFI_PASSWORD) # Connect to wifi
        print("Connecting to " + WIFI_SSID)
        # Check if connected to wifi otherwise wait
        while not wlan.isconnected() and wlan.status() >= 0:
            print(".", end="")
            sleep(1)
    # Print IP address
    ip = wlan.ifconfig()[0]
    print("/nConnected on {}".format(ip))
    return ip

# Callback function to respond to messages from Adafruit IO
def sub_cb(topic, msg): # sub_cb means "callback subroutine"
    print((topic, msg))

    if topic == AIO_MOISTURE_THRESHOLD_FEED.encode("utf-8"):
        try:
            global MOISTURE_THRESHOLD
            new_threshold = int(msg)
            if new_threshold != MOISTURE_THRESHOLD:
                MOISTURE_THRESHOLD = new_threshold
        
        except ValueError:
            print("Invalid value received for moisture threshold from Adafruit IO, value: " + str(msg))

    # if topic == AIO_MANUAL_CONTROL_FEED.encode("utf-8"):
    #     try:
    #         global MANUAL_PUMP_STATUS
    #         if(msg == b"ON"):
    #             MANUAL_PUMP_STATUS = True
    #             print("manual pump on")
    #         elif(msg == b"OFF"):
    #             MANUAL_PUMP_STATUS = False
    #             print("manual pump off")
    #         else:
    #             print("Unexpected message from Adafruit IO")
    #     except ValueError:
    #         print("Invalid value received for manual pump control from Adafruit IO, value: " + str(msg))
    # if topic == AIO_PUMP_STATUS_FEED.encode("utf-8"):
    #     try:
    #         global PUMP_STATUS
    #         if(msg == b"ON"):
    #             PUMP_STATUS = True
    #             print("pump on")
    #         elif(msg == b"OFF"):
    #             PUMP_STATUS = False
    #             print("pump off")
    #         else:
    #             print("Unexpected message from Adafruit IO")
    #     except ValueError:
    #         print("Invalid value received for pump status from Adafruit IO, value: " + str(msg))

   

lastPublishTime = 0
# Function to publish moisture level to Adafruit IO
def send_moisture_level(moistureLevel):
    global lastPublishTime, UPLOAD_FREQUENCY
    currentTime = time.time()
    if currentTime - lastPublishTime >= UPLOAD_FREQUENCY:
        try:
            client.publish(AIO_SOIL_MOISTURE_FEED, str(moistureLevel))
            lastPublishTime = time.time()
            print("Sent moisture level to Adafruit IO, value: ", str(moistureLevel))
        except Exception as e:
            print("Failed to send moisture level to Adafruit IO, error: " + str(e))

# Function to map values from one range to another
def map_value(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# Function to constrain a value between a minimum and maximum value
def constrain(value, min_val, max_val):
    return min(max(value, min_val), max_val)


# Function to measure moisture level
dryValue = 65500
wetValue = 41000
def measureMoisture():
    value = moistureSensor.read_u16()
    moistureLevel = constrain(map_value(value, dryValue, wetValue, 0, 100), 0, 100)
    return moistureLevel

def controlPump():
    checkPumpDuration()
    if (measureMoisture() < MOISTURE_THRESHOLD):
        activatePumpCycle()

pumpOn = False
lastPumpActivationTime = 0


#Turn pump on
def turnPumpOn():
    global pumpOn, lastPumpActivationTime
    relayPin.off()
    pumpOn = True
    lastPumpActivationTime = time.time()
    print("Pump turned on")

def turnPumpOff():
    global pumpOn
    relayPin.on()
    pumpOn = False
    print("Pump turned off")

def checkPumpDuration():
    global pumpOn, lastPumpActivationTime, PUMP_ACTIVATION_DURATION
    if pumpOn:
        currentTime = time.time()
        if currentTime - lastPumpActivationTime >= PUMP_ACTIVATION_DURATION:
            turnPumpOff()

def activatePumpCycle():
    global lastPumpActivationTime
    currentTime = time.time()
    if currentTime - lastPumpActivationTime >= PUMP_ACTIVATION_INTERVAL:
        turnPumpOn()

# Try WiFi connection
try:
    ip = do_connect()
except KeyboardInterrupt:
    print("Interrupted")

# Use the MQTT protocol to connect to Adafruit IO
client = MQTTClient(AIO_CLIENT_ID, AIO_SERVER, AIO_PORT, AIO_USERNAME, AIO_KEY)

# Subscribed messages will be delivered to this callback
client.set_callback(sub_cb)
client.connect()
client.subscribe(AIO_MANUAL_CONTROL_FEED)
client.subscribe(AIO_MOISTURE_THRESHOLD_FEED)
client.subscribe(AIO_PUMP_STATUS_FEED)
print("Connected to %s, subscribed to %s, %s, %s " % (AIO_SERVER, AIO_MOISTURE_THRESHOLD_FEED, AIO_MANUAL_CONTROL_FEED, AIO_PUMP_STATUS_FEED))




try:
    turnPumpOff()
    measureMoisture()
    while True:
        # Check for new messages
        #client.check_msg()
        # Send moisture level to Adafruit IO
        send_moisture_level(measureMoisture())
        # See if the pump needs activating
        controlPump()

        time.sleep(1)

    # turnPumpOn()
    # measureMoisture()
    # while True:
    #     # Check for new messages
    #     client.check_msg()
    #     # Send moisture level to Adafruit IO
    #     send_moisture_level(measureMoisture())

    #     controlPump()
    #     # Sleep for 5 seconds
    #     time.sleep(1)
finally:
    client.disconnect()
    print("Disconnected from Adafruit IO")





    

