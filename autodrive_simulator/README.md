# AutoDRIVE Simulator

<p align="justify">
This directory hosts AutoDRIVE Simulator executable (for F1TENTH), which supports vehicle and environment simulation as well as communication via a unified API. It equally prioritizes backend physics and frontend graphics to achieve high-fidelity simulation in real-time, and can be therefore exploited by the users to deploy and test their algorithms flexibly.
</p>

## SETUP

1. Clone the `AutoDRIVE-F1TENTH-Sim-Racing` repository.
    ```bash
    $ git clone https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-F1TENTH-Sim-Racing.git
    ```
2. Give executable permissions to the simulator.
   ```bash
   $ cd autodrive_simulator
   $ sudo chmod +x AutoDRIVE\ Simulator.x86_64
   ```
3. [OPTIONAL] To leverage the simulator's headless and background rendering capabilities, install `Xvfb` (X virtual framebuffer), which is a display server that emulates a virtual framebuffer, allowing users to run X11 applications without a physical display or input devices.
   ```bash
   $ sudo apt update
   $ sudo apt install xvfb
   ```

## USAGE

- **Graphics Mode Bringup:**
  ```bash
  $ ./AutoDRIVE\ Simulator.x86_64
  ```
  **[OR]**
- **Headless Mode Bringup:**
  ```bash
  $ xvfb-run ./AutoDRIVE\ Simulator.x86_64 -ip 127.0.0.1 -port 4567
  ```
  **[OR]**
- **No-Graphics Mode Bringup:**
  ```bash
  $ ./AutoDRIVE\ Simulator.x86_64 -batchmode -nographics -ip 127.0.0.1 -port 4567
  ```