# monumental_SWE_TH2
Take home assignment of Momumental SWE TH2, by Saray Bakker

Link to assignment: [Link][https://terraformco.notion.site/SWE-TH2-Path-following-wagon-dc822fca0a404e9dae1142f3fac8f695]

## Install:
Clone this repository:
```bash
git clone git@github.com:saraybakker1/monumental_SWE_TH2.git
cd monumental_SWE_TH2
```

### Installation via uv (recommended)
This project uses [`uv`](https://docs.astral.sh/uv/) for Python environment and dependency management.
When uv is not installed, please install via:
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```
When uv is installed, install the requirements, and run the example via:
```bash
uv sync
source .venv/bin/activate
uv run python assignment.py
```
To update the dependencies:
```bash
uv lock --upgrade
uv sync
```

### Installation via requirements.txt
When you don't have access to uv, from the project root, create a virtual environment and activate it:
```bash
python3.12 -m venv .venv 
source .venv/bin/activate
````
Install all required dependencies using requirements.txt:
```bash
pip install -r requirements.txt
```
Run the example via:
```bash
python assignment.py
```

## Small explanation of approach:
The assignment is to track an Lemniscate of Gerono parametrized path using a differential drive robot. 
Via a websocket, data comes in, containing:
- The (x, y) position via GPS, ~1 Hz
- The z-axis rotational velocity via the gyro, ~20Hz
- The local acceleration (a_x, a_y) via the accelerometer, ~20 Hz
Sensor update frequencies can be checked by running the script: assignment_check_sensor_updates.py
This sensor data is noisy, and we can analyze the mean and standard deviation of the noise by running the script:
assignment_check_sensor_noise.py.
Actions are send via the websocket to the robot at 50 Hz, and the actuators also contain noise. 

The proposed solution in assignment.py consist of the following components:
- Sensor Client: This sensor client reads the sensor data and stores it into RobotState.
- State Estimator using a Kalman filter:
  - The orientation of the robot is estimated from integrating the z-axis rotational velocity from the original position, combined with the estimated orientation from the GPS to avoid sensor drift.  
  - Velocities (in global frame) is estimated from the acceleration.
  - Position between GPS-updates: Estimate the 
- Controller: 

## Future steps:
Because I unfortunately only had a few hours to finish this assignment, there are many things to be left for future work:
Short term:
- State estimation: As observed, the sensor data is very noisy and we currently don't have an estimate of the actuator noise. I would proceed with properly analyzing the sensor-noise, delays and actuator-noise.
- Controller: 

Long term:
- State estimation: Analyze the state-of-the-art in state estimation using noisy data, e.g [Link][https://link.springer.com/content/pdf/10.1007/s10846-021-01383-5.pdf]. This could also include adding more sensor data (when available) for better localization, as the (drifted) estimated orientation is a bottleneck.
- Model of the robot: Create a model, perform system identification, 
- 









