# SWE TH2: Path-following wagon
Take home assignment of SWE TH2, by Saray Bakker

Link to assignment: (private)

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
pip install -e .
```
Run the example via:
```bash
python assignment.py
```

## Small explanation of approach

The assignment is to track a Lemniscate of Gerono parametrized path using a differential-drive robot.

Via a websocket, sensor data is received containing:

* The `(x, y)` position from GPS, approximately 1 Hz.
* The z-axis rotational velocity from the gyroscope, approximately 20 Hz.
* The local acceleration `(a_x, a_y)` from the accelerometer, approximately 20 Hz.

Sensor update frequencies can be checked by running `assignment_check_sensor_updates.py`. The sensor data is noisy, and the mean and standard deviation of the noise can be analyzed by running `assignment_check_sensor_noise.py`.

Control commands are sent to the robot at 20 Hz. The actuators also introduce noise, which is currently not explicitly modelled.

The proposed solution in `assignment.py` consists of the following components:

### Sensor Client

The Sensor Client receives the sensor data from the websocket and stores it in a `RobotState` object.

### State Estimator

A Kalman-filter-based state estimator is used to combine the different sensor measurements.

* **Orientation:** The orientation is propagated using the z-axis angular velocity from the gyroscope. Since integrating gyroscope measurements leads to drift, the orientation is periodically corrected using the direction of movement obtained from consecutive GPS measurements. Note that this correction becomes unreliable when the robot moves very slowly, is stationary or takes turns. 
* **Velocity:** Global-frame velocity is estimated by transforming the local accelerometer measurements using the estimated orientation and integrating the acceleration.
* **Position:** Between the relatively sparse GPS measurements, the robot position is propagated using the estimated velocity and acceleration. GPS measurements are subsequently used to reset the position.

### Controller

The controller currently uses a pure-pursuit-style path tracking approach:

* Estimate the closest point on the path to the current robot position.
* Restrict the search range to avoid jumping to a distant section of the path or taking shortcuts at the crossing of the lemniscate.
* Select a point ahead of the closest point as the lookahead target.
* Calculate the required linear and angular velocity to reach this target.
* Convert the desired linear and angular velocity into left and right wheel velocities.
* Limit the wheel velocity to avoid exceeding the wheel velocity or wheel acceleration limits.

The controller currently focuses on **geometric path tracking** rather than explicitly tracking the time parameterization of the trajectory.

Option 2: An alternative is implemented, combining feedforward control with feedback:

* Use the time-dependent reference velocity from the parametrized path as a feedforward component.
* Add a feedback component based on the position and orientation tracking error.

However, this approach depends more strongly on an accurate state estimate and therefore requires further tuning and validation.

### Goal / stopping criterion

The robot is considered to have reached the goal when its estimated position is within a specified tolerance of the final path position. The desired wheel velocities are then set to zero.

### Control Client

The Control Client sends the calculated left and right wheel velocities to the robot through the websocket at the control frequency.

### Visualization

A real-time visualization is provided showing:

* Robot and reference path position.
* Robot orientation.
* Estimated velocity.
* Measured acceleration.
* Desired velocity.
* Other relevant controller/state-estimation information.

The visualization is displayed in a separate pop-up window to allow the estimator and controller to be monitored while the robot is running.

## Future steps

Because I unfortunately only had a few hours to finish this assignment (having received the assignment on the 21th 11h, and going on holidays on the 23th), there are several areas that could be improved with more development time.

### Short term

**State estimation:**
The sensor data is quite noisy, and actuator noise is currently not explicitly characterized. I would first perform a more systematic analysis of the sensor noise, sensor delays, correlations between measurements, and actuator noise. The Kalman filter could then be tuned using these measured characteristics rather than approximate values.

**Controller:**
The controller currently ignores the time-dependent part of the parametrized path. I would investigate option 2, the feedforward + feedback approach described above, while also tuning the lookahead distance as a function of velocity to improve tracking performance.

**Safety and robustness:**
Additional handling should be added for invalid, missing, delayed, or implausible sensor measurements. Sensor failures should not result in unsafe actuator commands. Beyond enforcing limits on the desired velocity, also sensor inputs and intermediate states should be checked if physically feasible and within the expected limits.

**Testing:**
Unit tests and integration tests should be added for the individual components of the system. In particular, I would test:

* Coordinate-frame transformations.
* Angle wrapping.
* Kalman filter prediction and correction.
* GPS position and heading updates.
* Path nearest-point and lookahead selection.
* Wheel velocity conversion.
* Wheel acceleration limiting.
* Controller behavior near the end of the path.
* Individual sensor failures and missing measurements.

Simple simulated tracking tests should also be used to tune the controller and detect oscillations or unstable behavior before running it on the physical robot.

### Long term

**State estimation:**
A more extensive investigation of state-of-the-art state estimation methods for noisy and asynchronous sensor data would be useful, e.g. [State estimation reference](https://link.springer.com/content/pdf/10.1007/s10846-021-01383-5.pdf).

Additional available sensor information could also be incorporated to improve localization. In particular, orientation is currently an important limitation because gyroscope integration causes drift and GPS only provides an indirect heading measurement while the robot is moving.

**Robot model:**
A more accurate robot model should be developed, including system identification of the actual robot. The robot reportedly has four wheels rather than a conventional two-wheel differential-drive base, so the actual wheel configuration and its kinematics should be investigated and modelled.

Actuator dynamics, delays, wheel slip, and other sources of uncertainty could then be incorporated into the model.

**Controller:**
Once a sufficiently accurate robot model and uncertainty model are available, more advanced control architectures could be investigated. For example, Model Predictive Path Integral Control could be considered for handling the nonlinear dynamics, actuator constraints, and uncertainty.







