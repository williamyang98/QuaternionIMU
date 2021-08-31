import asyncio
import os
import numpy as np
import pandas as pd
import argparse
import serial
from pynput.keyboard import Key, Listener

import struct
import traceback
from timeit import default_timer

from async_serial_client import AsyncSerialClient
from async_i2c import AsyncI2C
from async_mpu6050 import MPU6050
from async_gy271 import GY271

from ekf_client import EKFClient
from cube_renderer import CubeRenderer
from convert_readings import MeasurementConverter


# listen to key inputs and set flags on our objects
class HookedListener:
    def __init__(self, ekf_client, async_serial):
        self.ekf_client = ekf_client
        self.async_serial = async_serial

    def on_press(self, key):
        if key == Key.f4:
            print("Starting measurements")
            self.async_serial.start_measurements()
        elif key == Key.f5:
            print("Stopping measurements")
            self.async_serial.stop_measurements()
        elif key == Key.f12:
            self.async_serial.is_running = False
            print("Force closing the serial task")
        elif key == Key.f6:
            if not self.ekf_client.is_calibrating:
                self.ekf_client.set_calibrate(True)
                print("Start calibration")

    def on_release(self, key):
        if key == Key.f6:
            if self.ekf_client.is_calibrating:
                self.ekf_client.set_calibrate(False)
                print("End calibration")

# attach measurement listeners
class MeasurementPacketListener:
    def __init__(self, ekf_client, ekf_converter):
        self.ekf_converter = ekf_converter
        self.ekf_client = ekf_client
    
        self.compass_data = []
        self.gyro_data = []
    
    def on_compass_packet(self, packet):
        if len(packet) != 10:
            print(f"Unknown compass packet: {packet}")
            return        
        
        dt_us = struct.unpack("<L", bytes(packet[0:4]))[0]
        x,y,z = struct.unpack(">hhh", bytes(packet[4:10]))

        dt = dt_us * 1e-6
        m_xyz = np.array((x,y,z), dtype=np.float32).reshape((3,1))
        m_xyz = self.ekf_converter.convert_magnetometer(m_xyz)
        self.ekf_client.on_compass(dt, m_xyz)
        self.compass_data.append((dt,*m_xyz.flatten()))

    def on_gyro_packet(self, packet):
        if len(packet) != 18:
            print(f"Unknown gyro packet: {packet}")
            return

        dt_us = struct.unpack("<L", bytes(packet[0:4]))[0]
        ax,ay,az,temp,p,q,r = struct.unpack(">hhhhhhh", bytes(packet[4:18]))

        dt = dt_us * 1e-6
        a_xyz = np.array((ax,ay,az), dtype=np.float32).reshape((3,1))
        pqr = np.array((p,q,r), dtype=np.float32).reshape((3,1))
        a_xyz = self.ekf_converter.convert_accelerometer(a_xyz)
        pqr = self.ekf_converter.convert_gyroscope(pqr)
        self.ekf_client.on_gyro(dt, a_xyz, pqr)
        self.gyro_data.append((dt, *a_xyz.flatten(), *pqr.flatten()))

# print out details of an invalid packet
def on_invalid_packet(packet):
    n = packet[0]
    content = packet[1:] 
    content_hex = ' '.join((f'{x:02X}' for x in content))
    content_ascii = ''.join((f'{chr(x)}' for x in content))
    print(f"[!] Invalid packet (len={n:d}) {content_hex} ")
    print(f"    ASCII: {content_ascii} ")

def export_data(compass_data, gyro_data, args):
    print(f"Recorded entries: gyro={len(gyro_data)}, compass={len(compass_data)}")

    if len(gyro_data) == 0 and len(compass_data) == 0:
        exit(1)

    if len(gyro_data) >= 2:
        total_us = gyro_data[-1][0] - gyro_data[0][0]
        Ts_ms = (total_us/len(gyro_data)) * 1e3
        print(f"Gyro sampled at avg of Ts={Ts_ms:.2f}ms")

    if len(compass_data) >= 2:
        total_us = compass_data[-1][0] - compass_data[0][0]
        Ts_ms = (total_us/len(compass_data)) * 1e3
        print(f"Compass sampled at avg of Ts={Ts_ms:.2f}ms")

    # find available filename
    if args.override is None:
        i = 0
        while True:
            gyro_out_filename    = args.output.format(sensor='gyro', i=i)
            compass_out_filename = args.output.format(sensor='compass', i=i)
            if not os.path.exists(gyro_out_filename) and not os.path.exists(compass_out_filename):
                break
            i += 1
    # override filename
    else:
        gyro_out_filename    = args.output.format(sensor='gyro', i=args.override)
        compass_out_filename = args.output.format(sensor='compass', i=args.override)

    # save files
    gyro_df = pd.DataFrame(gyro_data, columns=['dt (s)', 'Ax (ms-2)', 'Ay (ms-2)', 'Az (ms-2)', 'p (rads-1)', 'q (rads-1)', 'r (rads-1)'])
    compass_df = pd.DataFrame(compass_data, columns=['dt (s)', 'Mx (Gauss)', 'My (Gauss)', 'Mz (Gauss)'])

    gyro_df.to_csv(gyro_out_filename, index=None)
    compass_df.to_csv(compass_out_filename, index=None)

# on a separate async task, take readings from ekf and update cube render
async def thread_data_bus(async_serial, cube_renderer, ekf_client):
    while async_serial.is_running:
        E = ekf_client.ekf.E
        Erot = E.copy()
        Erot[1:,:] *= -1
        # print("\r{0:3.2f} {1:3.2f} {2:3.2f} {3:3.2f}".format(*E.flatten()), end='')
        rotm = ekf_client.ekf.find_observation_matrix(Erot).squeeze()
        cube_renderer.rotation_matrix = rotm
        await asyncio.sleep(0.01)
    cube_renderer.is_running = False

async def main(args):
    # serial
    ser = serial.Serial()
    ser.baudrate = args.baudrate
    ser.port = args.port
    async_serial = AsyncSerialClient(ser)

    # measurement and ekf
    ekf_client = EKFClient()
    ekf_client.ekf.Q = 1e-1*np.eye(4)
    ekf_client.use_paired_measurements = False
    ekf_client.is_calibrating = False
    ekf_converter = MeasurementConverter()
    ekf_converter.bias_magnetometer = np.array([-0.1, 0.05, 0]).reshape((3,1))
    measurement_packet_listener = MeasurementPacketListener(ekf_client, ekf_converter)

    # i2c bus for configuration
    async_i2c = AsyncI2C(async_serial)
    mpu6050 = MPU6050(async_i2c)
    gy271 = GY271(async_i2c)

    # attach packet listeners
    # async_serial.listen_header(0xFF, lambda d: print(f"[*] Alive packet ({d[0]:02X})"))
    async_serial.listen_header(0x02, lambda d: print(f"[C] Device not ready ({d[0]:02X})"))
    async_serial.listen_header(0x01, measurement_packet_listener.on_compass_packet)
    async_serial.listen_header(0x03, measurement_packet_listener.on_gyro_packet)
    async_serial.listen_header(0x04, lambda d: print(f"[*] Start ACK"))
    async_serial.listen_header(0x05, lambda d: print(f"[*] STOP ACK"))
    async_serial.listen_header(0x06, on_invalid_packet)
    async_serial.listen_header(0x07, async_i2c.on_read_ack)
    async_serial.listen_header(0x08, async_i2c.on_write_ack)

    # render our ekf readings on a separate thread
    cube_renderer = CubeRenderer()

    async def setup_imu():
        dt0 = default_timer()
        # setup our gyro and accelerometer
        async def send_fetch_config(send_coro, fetch_coro):
            await send_coro
            res = await fetch_coro
            return res

        tasks = []
        tasks.append(mpu6050.set_clock_source(0))
        tasks.append(send_fetch_config(mpu6050.set_accel_fullscale_range(3), mpu6050.get_accel_sensitivity()))
        tasks.append(send_fetch_config(mpu6050.set_gyro_fullscale_range(3), mpu6050.get_gyro_sensitivity()))
        tasks.append(send_fetch_config(gy271.set_config_A(3,6,0), gy271.get_config_A()))
        tasks.append(send_fetch_config(gy271.set_config_B(7), gy271.get_config_B()))
        tasks.append(gy271.get_identity())

        comb_res = await asyncio.gather(*tasks)
        res = comb_res[1:]

        ekf_converter.gain_accelerometer, ekf_converter.gain_gyroscope = res[0:2]
        gy271_config_a, gy271_config_b, gy271_identity = res[2:5]

        print(f"MPU6050 Gain: "+\
              f"accelerometer={ekf_converter.gain_accelerometer:.2e} ms-2 "+\
              f"gyroscope={ekf_converter.gain_gyroscope:.2e} degs-1 ")

        ekf_converter.gain_magnetometer = gy271_config_b.gain
        print(gy271_config_a)
        print(gy271_config_b)
        print(f"GY271 ID: {gy271_identity}")

        dt1 = default_timer()
        print(f"Setup took {dt1-dt0:.3f} seconds")

        # start measurmements
        print("Starting measurements automatically")
        async_serial.start_measurements()

    # start key listener thread
    hooked_listener = HookedListener(ekf_client, async_serial)
    key_listener = Listener(on_press=hooked_listener.on_press, on_release=hooked_listener.on_release)
    key_listener.start()

    try:
        # cube renderer is stopped by hooked listener
        render_tasks = asyncio.gather(
            cube_renderer.run(),
            thread_data_bus(async_serial, cube_renderer, ekf_client))

        await async_serial.open() 
        await asyncio.gather(
            async_serial.run(),
            setup_imu())
        await render_tasks
        
    except Exception as ex:
        print(f"Exception in run")
        print(traceback.format_exc())
    finally:
        # try to close cleanly
        try:
            async_serial.stop_measurements()
            async_serial.close()
        except Exception as ex:
            print(f"Exception in run")
            print(traceback.format_exc())

    # print stats
    gyro_data = measurement_packet_listener.gyro_data
    compass_data = measurement_packet_listener.compass_data
    export_data(compass_data, gyro_data, args)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM11")
    parser.add_argument("--baudrate",  default=38400)
    parser.add_argument("--output", default="./data/data_{sensor}_{i}.csv")
    parser.add_argument("--override", default=None)

    args = parser.parse_args()

    try:
        args.output.format(sensor="sensor", i=0)
    except:
        print("Output file names must have formatting keys for 'sensor' and 'i'")
        exit(1)

    asyncio.run(main(args))
