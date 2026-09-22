import asyncio
import json
import time

import numpy as np
import websockets

"""
This file checks the update frequency of the available sensor data. 
"""


URI = "ws://91.99.103.188:8765"


async def main():
    previous_data = {}
    previous_change_time = {}

    message_count = {}
    change_count = {}
    unchanged_count = {}

    change_intervals = {}

    last_report = time.monotonic()

    async with websockets.connect(URI) as ws:
        print("Connected")

        while True:
            message = await ws.recv()
            receive_time = time.monotonic()

            data = json.loads(message)

            if data.get("message_type") != "sensors":
                continue

            for sensor in data["sensors"]:
                name = sensor["name"]
                current = np.asarray(sensor["data"], dtype=float)

                # Initialize counters
                message_count.setdefault(name, 0)
                change_count.setdefault(name, 0)
                unchanged_count.setdefault(name, 0)
                change_intervals.setdefault(name, [])

                message_count[name] += 1

                # Check whether the value changed
                changed = (
                    name not in previous_data
                    or not np.array_equal(
                        current,
                        previous_data[name],
                    )
                )

                if changed:
                    change_count[name] += 1

                    if name in previous_change_time:
                        dt = (
                            receive_time
                            - previous_change_time[name]
                        )
                        change_intervals[name].append(dt)

                    previous_change_time[name] = receive_time

                else:
                    unchanged_count[name] += 1

                previous_data[name] = current.copy()

            # Print statistics every second
            if time.monotonic() - last_report >= 1.0:
                print("\n========== SENSOR UPDATE RATES ==========")

                for name in message_count:
                    messages = message_count[name]
                    changes = change_count[name]
                    unchanged = unchanged_count[name]

                    # Message rate over the last second
                    message_rate = messages

                    # Change rate
                    change_rate = changes

                    if change_intervals[name]:
                        avg_change_dt = np.mean(
                            change_intervals[name][-100:]
                        )
                        effective_rate = 1.0 / avg_change_dt
                    else:
                        effective_rate = 0.0

                    print(f"\n{name}")
                    print(f"  messages:   {messages}")
                    print(f"  changed:    {changes}")
                    print(f"  unchanged:  {unchanged}")
                    print(f"  change rate: {change_rate:.1f} Hz")
                    print(
                        f"  effective:   "
                        f"{effective_rate:.2f} Hz"
                    )

                print("==========================================")

                # Reset statistics for the next second
                message_count.clear()
                change_count.clear()
                unchanged_count.clear()

                last_report = time.monotonic()


if __name__ == "__main__":
    asyncio.run(main())