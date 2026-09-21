import asyncio
import json
import websockets


URI = "ws://91.99.103.188:8765"


async def receive_sensors(ws):
    async for message in ws:
        data = json.loads(message)

        if data.get("message_type") != "sensors":
            continue

        for sensor in data["sensors"]:
            if sensor["name"] == "gps":
                print("GPS:", sensor["data"])


async def send_commands(ws):
    while True:
        command = {
            "v_left": 1.0,
            "v_right": 1.0,
        }

        await ws.send(json.dumps(command))
        print("SENT:", command)

        await asyncio.sleep(0.02)  # 50 Hz


async def main():
    async with websockets.connect(URI) as ws:
        print("Connected")

        await asyncio.gather(
            receive_sensors(ws),
            send_commands(ws),
        )


if __name__ == "__main__":
    asyncio.run(main())