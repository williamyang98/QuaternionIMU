from cobs import cobs_encode, cobs_decode
import asyncio

class AsyncSerialClient:
    def __init__(self, ser):
        self.ser = ser
        self.rx_encoded = []
        self.listeners = {}
        self.fallback_listeners = set([])
        self.is_running = True

    async def open(self):
        print("Attempting to connect")
        self.ser.open()
        print("Successfully connected")

    # run this asynchronously in our event loop    
    async def run(self):
        while self.ser.is_open and self.is_running:
            await asyncio.sleep(0.0005)

            x = self.ser.read_all()
            if x is None or len(x) == 0:
                continue
                
            x = list(x)
            self.consume_rx_packets()
            self.rx_encoded.extend(list(x))
    
    def close(self):
        try:
            if self.ser.is_open:
                self.ser.close()
            print("Terminated connection")
        except Exception as ex:
            print(f"Encountered error when closing serial: {str(ex)}")
    
    def extract_packets(self, encoded):
        packets = []
        while True:
            try:
                i = encoded.index(0x00)
                decoded = cobs_decode(encoded[:i+1])
                encoded = encoded[i+1:]
                packet = decoded[:-1]
                packets.append(packet)
            except ValueError:
                break
        
        return (encoded, packets)
    
    def listen_header(self, header, callback):
        observers = self.listeners.setdefault(header, set([]))
        observers.add(callback)
    
    def listen_header_fallback(self, callback):
        self.fallback_listeners.add(callback)

    def consume_rx_packets(self):
        encoded, packets = self.extract_packets(self.rx_encoded)
        self.rx_encoded = encoded

        for packet in packets:
            header = packet[0]
            content = packet[1:]

            if header in self.listeners:
                callbacks = self.listeners[header]
            else:
                callbacks = self.fallback_listeners

            for callback in callbacks:
                callback(content)

    def start_measurements(self):
        self.send_command([0x01, 0xA4])
    
    def stop_measurements(self):
        self.send_command([0x02, 0x4A])

    def send_command(self, command):
        # start off with padding character to terminate 
        # debugging message from hc06 module
        tx_encoded = [0x00,]
        tx_encoded.extend(cobs_encode(command))
        self.ser.write(bytearray(tx_encoded))