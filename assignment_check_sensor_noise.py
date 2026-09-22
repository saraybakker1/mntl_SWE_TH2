import asyncio
import json
import time

import numpy as np
import websockets

"""
This file estimates the sensor noise based on the available sensor data.
"""

URI = "ws://91.99.103.188:8765"


async def main():
    values = {}
    last_report = time.monotonic()

    async with websockets.connect(URI) as ws:
        print("Connected")

        while True:
            message = await ws.recv()

            data = json.loads(message)

            if data.get("message_type") != "sensors":
                continue

            for sensor in data["sensors"]:
                name = sensor["name"]
                current = np.asarray(sensor["data"], dtype=float)

                # Initialize storage for this sensor
                if name not in values:
                    values[name] = []

                values[name].append(current)

            # Print statistics every second
            if time.monotonic() - last_report >= 1.0:
                print("\n========== SENSOR NOISE ==========")

                for name, sensor_values in values.items():
                    samples = np.asarray(sensor_values)

                    if len(samples) < 2:
                        continue

                    mean = np.mean(samples, axis=0)
                    noise = np.std(samples, axis=0)

                    print(f"\n{name}")
                    print(f"  samples: {len(samples)}")
                    print(f"  mean:    {mean}")
                    print(f"  noise:   {noise}")

                print("==================================")

                # Reset for next second
                values.clear()
                last_report = time.monotonic()


if __name__ == "__main__":
    asyncio.run(main())