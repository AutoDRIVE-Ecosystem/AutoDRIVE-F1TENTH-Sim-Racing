# AutoDRIVE Devkit

<p align="justify">
This directory hosts ROS 2 API (a meta-package), which supports modular algorithm development targetted towards autonomous driving. It uses client libraries for Python and C++, and can be therefore exploited by the users to develop their algorithms flexibly.
</p>

## SETUP

1. Clone the `AutoDRIVE-F1TENTH-Sim-Racing` repository.
    ```bash
    $ git clone https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-F1TENTH-Sim-Racing.git
    ```
2. Give executable permissions to the Python scripts.
   ```bash
   $ cd autodrive_devkit
   $ sudo chmod +x *.py
   ```
4. Install the necessary dependencies as mentioned below.
    [AutoDRIVE Devkit's ROS 2 API](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-F1TENTH-Sim-Racing/tree/main/autodrive_devkit) has the following dependencies (tested with Python 3.8, 3.9 and 3.10):
    
    - Websocket-related dependencies for communication bridge between [AutoDRIVE Simulator](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-F1TENTH-Sim-Racing/tree/main/autodrive_simulator) and [AutoDRIVE Devkit](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-F1TENTH-Sim-Racing/tree/main/autodrive_devkit) (version sensitive):
    
      | Package            | Python 3.8 | Python 3.9 | Python 3.10 |
      |--------------------|------------|------------|-------------|
      | eventlet           | 0.33.3     | 0.33.3     | 0.33.3      |
      | Flask              | 1.1.1      | 1.1.1      | 1.1.1       |
      | Flask-SocketIO     | 4.1.0      | 4.1.0      | 4.1.0       |
      | python-socketio    | 4.2.0      | 4.2.0      | 4.2.0       |
      | python-engineio    | 3.13.0     | 3.13.0     | 3.13.0      |
      | greenlet           | 1.0.0      | 1.0.0      | 1.1.0       |
      | gevent             | 21.1.2     | 21.1.2     | 21.12.0     |
      | gevent-websocket   | 0.10.1     | 0.10.1     | 0.10.1      |
      | Jinja2             | 3.0.3      | 3.0.3      | 3.0.3       |
      | itsdangerous       | 2.0.1      | 2.0.1      | 2.0.1       |
      | werkzeug           | 2.0.3      | 2.0.3      | 2.0.3       |
    
    - Generic dependencies for data processing and visualization (usually any version will do the job):
    
      | Package               | Tested Version |
      |-----------------------|----------------|
      | numpy                 | 1.13.3         |
      | pillow                | 5.1.0          |
      | opencv-contrib-python | 4.5.1.48       |
  
    - Install dependencies using `requirements.txt` file (use the file specific to your Python version &#8594; check using `python3 --version`):

      ```bash
      $ pip3 install -r requirements_python_3.8.txt # Python 3.8
      $ pip3 install -r requirements_python_3.9.txt # Python 3.9
      $ pip3 install -r requirements_python_3.10.txt # Python 3.10
      ```

## USAGE

- **Bringup:**
    - **Headless Mode Bringup:**
      ```bash
      $ ros2 launch autodrive_f1tenth simulator_bringup_headless.launch.py
      ```
      **[OR]**
    - **RViz Mode Bringup:**
      ```bash
      $ ros2 launch autodrive_f1tenth simulator_bringup_rviz.launch.py
      ```

- **Teleoperation:**
  ```bash
  $ ros2 run autodrive_f1tenth teleop_keyboard
  ```