from monumental_swe_th2.sensor_client import SensorClient
from monumental_swe_th2.state_estimator import StateEstimator
from monumental_swe_th2.visualization import StatePlotter
from monumental_swe_th2.controller_simple import PathController
from monumental_swe_th2.control_client import CommandClient
import asyncio

async def main():
    client = SensorClient("ws://91.99.103.188:8765")
    command_client = CommandClient("ws://91.99.103.188:8765")

    estimator = StateEstimator()
    visual = StatePlotter(max_points=500)
    iter = 0
    controller = PathController(
        wheel_base=0.5,
        max_wheel_velocity=2.0,
        lookahead_distance=0.3,
        max_velocity=2.0,
    )
    await client.start()
    await command_client.connect()

    try:
        while True:
            state = client.get_state()
            estimator.update(state)
            wheel_velocity, target = controller.update(state)
            visual.update(state, target)
            print("current_time:", state.current_time) #for debugging
            if wheel_velocity is not None:

                # dummy test:
                v_left = 2
                v_right = 0
                # v_left, v_right = wheel_velocity

                await command_client.send_wheel_velocity(
                    v_left,
                    v_right,
                )
            iter = iter+1
            await asyncio.sleep(0.02)  # 50 Hz
    finally:
        await client.stop()


asyncio.run(main())
# u = controller(position, velocity, acceleration, gyro)