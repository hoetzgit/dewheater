## Dewheater

Dew heater controller for allksycam.

### Overview

This code controls a relay which powers a dew heater circuit. A hacked USB powered dew heater band and resistor based
design have been tested. The USB dew heater band hack consists of nothing more than removing the switch from the dew
heater and directly connecting the power leads to the NO side of a relay. This code should work with other designs also, but
may not have been tested yet.

A BME280 or DHT22 sensor is used to monitor temperature and humidity. When dew point cut-in set point is reached then
the dew heater relay is closed. When the cut-out set point is reached the dew heater relay is opened. Both the cut-in
and cut-out set points are defined in the configuration file as an offset from degrees Celsius of the dew point.
This offset allows for a rough degree of hysteresis control. This method of temperature control is primitive, but is
sufficient for this purpose.

Configuration parameters in dewheaterconfig.json:

"debug": true # debug on/off

"sensorType": "BME280", Set to either "BME280" or "DHT22" depending on sensor type installed.

"dhtPin": 4, GPIO pin used for DHT22 if installed, BCM mode

"bmePort": 1, Port used by BME280 if installed

"bmeAddress": "0x76", BMD280 address if installed

"dewHeaterPin": 23, # Dew Heater relay control pin, BCM mode

"dewHeaterCutinOffset": 1.0, # Dew Heater cut-in (on) offset in degrees C. This offset is added to the calculated dew
point.

"dewHeaterCutoutOffset": 1.0, # Dew Heater cut-out (off) offset in degrees C. This offset is added to the calculated dew
point.

"dewHeaterMaxTemp": 35, # Dew Heater max temp, dew heater relay is opened if this temp is reached, but control is not
shutdown thus the dew heater relay may be closed later if a set point is met.

"dewHeaterMinTemp": 3, # Dew Heater min temp, dew heater relay closed at this temp regardless of dew point calculations.
This parameter is intended to force the dew heater on in cold conditions regardless of whether the dew point has been
reached.

"dewHeaterMaxTimeOn": 6.5, # Maximum time limit for dew heater relay to be in an "on" state. Time is specified in
decimal hours. When this time limit is exceeded the relay will be turned off with "Force=True". To reset, restart
dewheater service.
This is a safety feature intended to prevent the dew heater from staying on indefinitely. If dewHeaterMaxTimeOn equals
zero then no check of maximum on time is performed (effectively max time on is infinite).

"dewHeaterOnOffDelay": 5, # Delay between on/off cycle, used only by dewheatertest.py

"dewPtCheckDelay": 5 # Time in seconds to wait between each dew point calculation (this includes reading the sensor
and making the dew point calculation). This parameter also determines how frequently status output is generated.

"fakeDewPoint": false # enables dew point faking for test purposes. If enabled, temperature will be set to
the dew point minus 2C.

"fakeDewPointSamples": 20 # number of samples for which dew point will be faked, after that samples are normal.

"invertOnOff": false # invert hi/low relay control signals for relay on/off to support relays wired to close on low
signal and open on high.

Sensor Placement. The current code assumes that the BME or DHT sensor is under the acrylic dome. This allows for
monitoring of actual conditions under the dome. The dew point offset values are implemented so that cut in/out points
can be offset
relative to what works best for your installation to keep the dome dew free, and to adjust for hysteresis (which is
considerable in my configuration). Note that features like max temp shutoff will only work properly with the sensor
under
the acrylic dome.

Modules

	dewheater.py  	   The main module, designed to be run as a service

	dewheatertest.py   Simple timed on/off test script. Delay between on/off cycle is controlled by "dewHeaterOnOffDelay" parameter above.
	                   Current master version only supports BME sensors. This will be fixed as soon in a future release. 

	dewheateron.py     Closes dew heater relay unconditionally (no cut in/out points, no timer). CAUTION: It does NOT open the relay again!

	dewheateroff.py    Opens dew heater relay unconditionally (no cut in/out points, no timer).
	
	



  



