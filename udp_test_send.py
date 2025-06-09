import asyncio
import json

import time
import datetime
from datetime import date
from datetime import datetime


async def main():
    print("Sending UDP message...")

    tvcinput = {
        "type": "greeting",
        "message": "Hello from sender!",
        "timestamp": f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')}"
    }
    
    message = json.dumps(tvcinput)
    loop = asyncio.get_running_loop()

    transport, protocol = await loop.create_datagram_endpoint(
        asyncio.DatagramProtocol,
        remote_addr=('127.0.0.1', 9999)
    )

    transport.sendto(message.encode())
    print(f"JSON packet sent")

    await asyncio.sleep(1)  # Give the message time to be sent
    transport.close()

if __name__ == "__main__":
    asyncio.run(main())
