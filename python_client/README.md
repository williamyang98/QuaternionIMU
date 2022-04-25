## Scripts
live_async_ekf.py runs the client which takes raw sensor readings from the Arduino server and passes it to an extended Kalman filter. 
The raw readings undergo conversion to their actual SI values and units before being given to the filter.
The output from the filter is the orientation quaternion of the sensors (normalised). 
This is viewed in realtime in pygame with a coloured cubed.

## Controls
| Key | Description |
| --- | --- |
| F6 | Hold and release to start then stop calibration to forward direction |
| F5 | Make IMU stop sending measurements | 
| F4 | Make IMU start sending measurements | 
| F12 | Disconnect serial |

## Linux setup
Linux requires the bluetooth device be registered to a file handler, then permissions setup on that file.

1. Connect HC06 and pair it to computer
2. Register the HC06 as a serial device

<code>sudo rfcomm bind 0 [bluetooth address]</code>

E.g. <code>sudo rfcomm bind 0 98:D3:41:F9:42:32</code>

3. Setup permssions on rfcomm serial device [Link to source](https://arduino.stackexchange.com/questions/21215/first-time-set-up-permission-denied-to-usb-port-ubuntu-14-04)

<code>
sudo usermod -a -G dialout $USER

ls -l /dev/rfcomm0

sudo chmod a+rw /dev/rfcomm0

</code>

4. Run program using <code>/dev/rfcomm0</code> as the port name

<code>python3 live_async_ekf.py --port /dev/rfcomm0</code>