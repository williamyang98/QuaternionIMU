import asyncio

# asynchronously send commands over the arduino's i2c bus using the serial port
# this is async because it can take a few milliseconds to complete an i2c operation
class AsyncI2C:
    def __init__(self, serial_client):
        self.serial_client = serial_client

        self.read_pending_set = {}
        self.read_pending = {}
        self.write_pending_set = {}
        self.write_pending = {}

        self.unknown_read_acks = []
        self.unknown_write_acks = []

    # read data
    async def read(self, addr, reg, n):
        d = [0x03, addr, reg, n]
        d = [0xFF&x for x in d]

        self.send_command(d)
        key = (addr, reg)

        sem = self.read_pending_set.setdefault(key, asyncio.Semaphore(0))
        await sem.acquire()

        data = self.read_pending[key].pop()
        return data

    # write data
    async def write(self, addr, reg, data):
        n = len(data)
        d = [0x04, addr, reg, n, *data]
        d = [0xFF&x for x in d]

        self.send_command(d)
        key = (addr, reg)

        sem = self.write_pending_set.setdefault(key, asyncio.Semaphore(0))
        await sem.acquire()

        data = self.write_pending[key].pop()
        return data

    # we place i2c read acknowledge packets here 
    # contains (addr, register, total_bytes_read, data[total_bytes_read])
    def on_read_ack(self, packet):
        addr, reg, n = packet[:3]
        key = (addr, reg)
        if key not in self.read_pending_set:
            self.unknown_read_acks.append(packet)
            return

        datas = self.read_pending.setdefault(key, [])

        if n == 0:
            datas.append(None)
        else:
            data = packet[3:3+n]
            datas.append(data)

        sem = self.read_pending_set[key]
        sem.release()

    # we place i2c write information packets here
    # contains (addr, register, total_bytes_written)
    def on_write_ack(self, packet):
        addr, reg, n = packet[:3]
        key = (addr, reg)
        if key not in self.write_pending_set:
            self.unknown_write_acks.append(packet)
            return

        datas = self.write_pending.setdefault(key, [])
        datas.append(n)
        sem = self.write_pending_set[key]
        sem.release()

    def send_command(self, command):
        self.serial_client.send_command(command)