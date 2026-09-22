import asyncio
import websockets

from monumental_swe_th2.sensor_client import SensorClient
from monumental_swe_th2.state_estimator import StateEstimator
from monumental_swe_th2.visualization import StatePlotter
from monumental_swe_th2.controller import Controller
from monumental_swe_th2.control_client import CommandClient

"""
This is the main file, run this file to start the loop taking in sensor data and sending control actions. 

Run via: 
uv run python assignment.py 
"""


URI = "ws://91.99.103.188:8765"
dt = 0.02

async def control_loop(
    sensor_client,
    command_client,
    estimator,
    controller,
    visual,
    ws,
):
    while True:
        # Get the MOST RECENT state received by receive_sensors()
        state = sensor_client.get_state()

        estimator.update(state)

        wheel_velocity, target = controller.update(state)

        visual.update(state, target, wheel_velocity)

        if wheel_velocity is not None:
            v_left, v_right = wheel_velocity

            await command_client.send_wheel_velocity(
                ws,
                v_left,
                v_right,
            )

        await asyncio.sleep(dt)


async def main():

    sensor_client = SensorClient(URI)
    command_client = CommandClient(URI)
    visual = StatePlotter(max_points=500, max_wheel_velocity=2.0)
    estimator = StateEstimator()

    controller = Controller(
        wheel_base=0.5,
        max_wheel_velocity=1.0,
        lookahead_distance=1.0,
        max_velocity=1.0,
        dt = dt
    )

    async with websockets.connect(URI) as ws:
        print("Connected")

        await asyncio.gather(
            sensor_client.receive(ws),
            control_loop(
                sensor_client,
                command_client,
                estimator,
                controller,
                visual,
                ws,
            ),
        )


asyncio.run(main())