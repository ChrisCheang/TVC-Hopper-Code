import asyncio
import json

import time
import datetime
from datetime import date
from datetime import datetime

import numpy as np

start = time.time()


class JSONSenderProtocol(asyncio.DatagramProtocol):
    def __init__(self):
        self.transport = None
        self.counter = 0

    def connection_made(self, transport):
        self.transport = transport
        asyncio.create_task(self.send_loop())

    async def send_loop(self):
        while True:
            now = time.time()
            dt = now - start
            tvcinput = {
                "tvcs":{
                    "tvc0":{
                        "channel":0,
                        "state":"calibrate", #idle, calibrate, arm, lock, test_procedure, demand_pos
                        "gimbal_angle_0":0*np.pi/180,
                        "gimbal_angle_1":0*np.pi/180
                    }
                },
                "timestamp": f"{now}",
                "counter": self.counter
            }
            data = json.dumps(tvcinput).encode()
            self.transport.sendto(data)
            print(f"Sent: {tvcinput}")
            self.counter += 1
            await asyncio.sleep(0.001)  # Need to keep this or else the time between send and recieve jumps to 0.1s

    def error_received(self, exc):
        print(f"Error: {exc}")

    def connection_lost(self, exc):
        print("Sender closed")


async def main():
    print("Sending UDP message...")

    loop = asyncio.get_running_loop()

    transport, protocol = await loop.create_datagram_endpoint(
        lambda: JSONSenderProtocol(),
        remote_addr=('127.0.0.1', 9999)
    )

    try:
        await asyncio.sleep(3600)
    finally:
        transport.close()

if __name__ == "__main__":
    asyncio.run(main())
