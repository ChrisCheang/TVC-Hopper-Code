import asyncio
import json

import time
import datetime
from datetime import date
from datetime import datetime


class UDPServerProtocol:
    def connection_made(self, transport):
        self.transport = transport
        print("UDP server ready and listening")

    def datagram_received(self, data, addr):
        try:
            message = json.loads(data.decode())
            print(f"Received JSON from {addr}: {message}")
            
            dt = datetime.now() - datetime.strptime(message["timestamp"], '%Y-%m-%d %H:%M:%S.%f')
            dt = dt.total_seconds()
            print(dt)
        except json.JSONDecodeError:
            print(f"Received invalid JSON from {addr}: {data.decode()}")
    
    def error_received(self, exc):
        print(f"Error received: {exc}")

    def connection_lost(self, exc):
        print("Connection closed")

async def main():
    print("Starting UDP server...")
    loop = asyncio.get_running_loop()
    transport, protocol = await loop.create_datagram_endpoint(
        lambda: UDPServerProtocol(),
        local_addr=('127.0.0.1', 9999)
    )

    try:
        await asyncio.sleep(3600)  # Keep server running for 1 hour
    finally:
        transport.close()

if __name__ == "__main__":
    asyncio.run(main())
