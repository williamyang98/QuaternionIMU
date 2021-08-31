## Scripts
live_async_ekf.py runs the client which takes raw sensor readings from the Arduino server and passes it to an extended Kalman filter. 
The raw readings undergo conversion to their actual SI values and units before being given to the filter.
The output from the filter is the orientation quaternion of the sensors (normalised). 
This is viewed in realtime in pygame with a coloured cubed.
