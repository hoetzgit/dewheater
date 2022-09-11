#
#  dew-heater-control
#
#  2021-10-18 C. Collins created
#
#  This code controls a relay which supplies power to a dew heater circuit. It has been tested with a hacked band dew heater (as referenced below) and
#  with resistor based circuits too.
#
#  A DHT22 or BME820 sensor is used to monitor temperature and humidity. When dew point cut-in set point is reached then the dew heater relay is closed.
#  When the cut-out set point is reached the dew heater relay is opened. Both the cut-in and cut-out set points are defined in the configuration file as
#  an offset from degrees Celsius of the dew point. This method of temperature control is primitive, but is sufficient for this purpose.
#
#
#  The original design was based upon a dew heater like the one at the link below:
#
#         https://www.amazon.com/dp/B08LGN222F?psc=1&ref=ppx_yo2_dt_b_product_details
#
#
#

import sys
import RPi.GPIO as GPIO
import time
import json
import Adafruit_DHT
from meteocalc import dew_point

import smbus2
import bme280

DHT_SENSOR = Adafruit_DHT.DHT22
ON = 1
OFF = 0


class ConfigClass:

    def __init__(self):
        self.loadConfig()
        #        self.setup()
        GPIO.setmode(GPIO.BCM)

    # def checkConfig(self):
    # future edits...

    def loadConfig(self):
        try:
            with open('/home/pi/dewheater/dewheaterconfig.json', 'r') as f:
                self.configFile = json.load(f)
                print(json.dumps(self.configFile, indent=4, sort_keys=True))
                self.debug = self.configFile['debug']
                self.dhtPin = self.configFile['dhtPin']
                self.sensorType = self.configFile['sensorType']
                if (self.sensorType != "BME280" and self.sensorType != "DHT22"):
                    sys.stderr.write("\nInvalid sensor type: %s" %self.sensorType)
                    raise Exception(1)
                self.bmeAddress = self.configFile["bmeAddress"]
                self.bmePort = self.configFile["bmePort"]
                self.dewHeaterPin = self.configFile['dewHeaterPin']
                self.dewHeaterCutinOffset = self.configFile['dewHeaterCutinOffset']
                self.dewHeaterCutoutOffset = self.configFile['dewHeaterCutoutOffset']
                self.dewHeaterMaxTemp = self.configFile['dewHeaterMaxTemp']
                self.dewHeaterMinTemp = self.configFile['dewHeaterMinTemp']
                self.dewHeaterMaxTimeOn = self.configFile['dewHeaterMaxTimeOn']
                self.dewPtCheckDelay = self.configFile['dewPtCheckDelay']
                self.fakeDewPoint = self.configFile['fakeDewPoint']
                self.fakeDewPointSamples = self.configFile['fakeDewPointSamples']
                self.invertOnOff = self.configFile['invertOnOff']
                if (self.invertOnOff):
                    self.relayOn = GPIO.LOW
                    self.relayOff = GPIO.HIGH
                else:
                    self.relayOn = GPIO.HIGH
                    self.relayOff = GPIO.LOW

        except:
            sys.stderr.flush()
            sys.exit("\nError opening or parsing config file, exiting")

        return (True)


config = ConfigClass()


class DHTClass:

    def __init__(self):
        GPIO.setup(config.dhtPin, GPIO.IN)

    def getData(self):
        self.humidity, self.temperature = Adafruit_DHT.read_retry(DHT_SENSOR, config.dhtPin)
        return self.humidity, self.temperature


dht = DHTClass()


class BME820Class:
    def __init__(self):
        self.port = config.bmePort
        self.address = int(config.bmeAddress,0)
        self.bus = smbus2.SMBus(config.bmePort)
        self.bme_calibration_params = bme280.load_calibration_params(self.bus, self.address)

    def getData(self):
        data = bme280.sample(self.bus, self.address, self.bme_calibration_params)
        self.humidity = data.humidity
        self.temperature = data.temperature
        return self.humidity, self.temperature


bme = BME820Class()


class ConditionsClass:

    def __init__(self):
        self.fakeDewPointCounter = 0
        self.dewPointMet = False
        self.temp_actual = -1

    def update(self):
        if (config.sensorType == "BME280"):
            self.humidity, self.temperature = bme.getData()

        else:
            if (config.sensorType == "DHT22"):
                self.humidity, self.temperature = dht.getData()

            else:
                sys.stderr.write("\nInvalid temperature sensor type: %s" % config.sensorType)
                return

        if self.humidity is not None and self.temperature is not None:
            if ((self.temperature >= -100) and (self.temperature <= 200)) and (
                    (self.humidity >= 0) and (self.humidity <= 100)):
                self.temp_actual = self.temperature  # set actual temp for use when fakeDewPoint is true
                self.dewPoint = dew_point(self.temperature, self.humidity)

                if (config.fakeDewPoint):
                    self.fakeDewPointCounter += 1
                    if (self.fakeDewPointCounter < config.fakeDewPointSamples):
                        self.temperature = self.dewPoint.c - 2
                    else:
                        config.fakeDewPoint = False  # fake done, clear flag

                if (self.temperature <= self.dewPoint.c + config.dewHeaterCutinOffset):
                    self.dewPointMet = True
                else:
                    if (self.temperature >= self.dewPoint.c + config.dewHeaterCutoutOffset):
                        self.dewPointMet = False
            else:
                sys.stderr.write("\nError calculating dew point, input out of range:")
                sys.stderr.write("\nTemp = %3.1fC" % self.temperature)
                sys.stderr.write("\nHumdidity = %3.1f%%\n" % self.humidity)
                return


conditions = ConditionsClass()


class DewHeaterClass:
    def __init__(self):
        GPIO.setup(config.dewHeaterPin, GPIO.OUT)
        self.cycleRelay()  # cycle the relay just as a start up test, if you dont hear it clicking then it aint working
        self.status = OFF
        self.off(True)  # force off in case left on by previous session
        self.minTempOn = False
        self.maxTempOff = False
        self.temp_actual = 0.0
        self.dewHeaterMaxTimeOnStatus = False
        self.timeOn = 0.0

    def on(self, forced=False):

        if (forced):
            GPIO.output(config.dewHeaterPin, config.relayOn)
            self.status = ON
            return

        if (self.maxTempOff):
            print("Max temp reached, 'on' command ignored")
            return

        GPIO.output(config.dewHeaterPin, config.relayOn)
        self.status = ON
        self.timeStampOn = time()

    def off(self, forced=False):
        if (forced):
            GPIO.output(config.dewHeaterPin, config.relayOff)
            self.status = OFF
            return

        if (self.minTempOn):
            print("Min temp on, 'off' command ignored")
            return

        GPIO.output(config.dewHeaterPin, config.relayOff)
        self.status = OFF
        self.timeStampOn = 0

    def cycleRelay(self):
        self.status = OFF
        GPIO.output(config.dewHeaterPin, config.relayOn)
        time.sleep(1)
        GPIO.output(config.dewHeaterPin, config.relayOff)
        time.sleep(1)
        GPIO.output(config.dewHeaterPin, config.relayOn)
        time.sleep(1)
        GPIO.output(config.dewHeaterPin, config.relayOff)

    def checkTemps(self):
        conditions.update()

        if (conditions.temp_actual > config.dewHeaterMaxTemp):  # use temp_actual for when fakeDewPoint is set
            self.maxTempOff = True
            self.off(True)
            return ()
        else:
            if (self.maxTempOff):
                self.maxTempOff = False
                self.on(False)

        if (conditions.temperature < config.dewHeaterMinTemp):
            self.minTempOn = True
            self.on(False)
            return ()
        else:
            if (self.minTempOn):
                self.minTempOn = False
                self.off(False)
                return

        if conditions.dewPointMet:
            if (not self.maxTempOff):
                self.on(False)
        else:
            if (not self.minTempOn):
                self.off(False)

    def checkMaxTimeOn(self):
        if (dewHeater.status == ON):
            hoursOn = (time() - dewHeater.timeStampOn) / 3600
            if (hoursOn > config.dewHeaterMaxTimeOn):
                self.off(True)


dewHeater = DewHeaterClass()


def dispalySatus():
    print("====================================================")
    print("Sensor Type = %s" % config.sensorType, end='')
    print("  Dew heater state = ", end='')
    if (dewHeater.status == ON):
        print("ON ", end='')
    else:
        print("OFF ", end='')
    print(" invertOnOff = %s" % (config.invertOnOff))

    print("Temp = %3.1fC, temp_actual = %3.1fC, Humidity %3.1f%%, Dew Point = %3.1fC" % (
        conditions.temperature, conditions.temp_actual, conditions.humidity, conditions.dewPoint.c))

    print("MinTempOn set point = %3.1fC, MinTempOn = %s" % (config.dewHeaterMinTemp, dewHeater.minTempOn))
    print("MaxTempOff set point = %3.1fC, MaxTempOff = %s" % (config.dewHeaterMaxTemp, dewHeater.maxTempOff))
    print("MdewHeaterMaxTimeOn = %3.1fH, dewHeaterMaxTimeOn status = %s" % (config.dewHeaterMaxTimeOn, dewHeater.dewHeaterMaxTimeOnStatus))
    print("Dew point met = %s, fakeDewPoint = %s, fakeDewPointCounter = %i " % (
    conditions.dewPointMet, config.fakeDewPoint, conditions.fakeDewPointCounter))
    print("====================================================")


def main():
    while True:
        dewHeater.checkTemps()
        dewHeater.checkMaxTimeOn()
        dispalySatus()
        time.sleep(config.dewPtCheckDelay)


if __name__ == "__main__":
    main()
