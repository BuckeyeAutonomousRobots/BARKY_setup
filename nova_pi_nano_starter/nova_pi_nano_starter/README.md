# NOVA VS Code workspace: Raspberry Pi ↔ Nano, stage 1

This project implements the first milestone: the Nano streams IMU readings and status,
the Pi exposes them through ROS2, and test joint-angle packets receive acknowledgements.
**There is no servo/PCA9685 output code and no arming capability.** Keep servo power
disconnected during these tests. The watchdog here detects host-link loss; it is not
yet a motion watchdog or a complete safety controller.

## What is included

- `firmware/src/main.cpp`: Nano C++ firmware.
- `firmware/include/protocol.h`: platform-independent packet parser.
- `platformio.ini`: Nano board/framework/library configuration.
- `nova.code-workspace`: open this in VS Code.
- `.vscode/tasks.json`: build, upload, monitor, and Pi ROS2 tasks.
- `scripts/`: portable task launchers.
- `ros2_ws/src/nova_hardware/`: installable `ament_python` ROS2 package.
- `tests/`: protocol, mocked bridge, and host-compiled firmware checks.
- `PROTOCOL.md`: exact packet format and limitations.

Hardware: Raspberry Pi 4 with an existing ROS2 installation, Arduino Nano 33 BLE
Sense Rev2, and a USB data cable. The Rev2 IMU library differs from the original
Sense board. This stage does not need the GY-521 or PCA9685 connected.

## 1. Open and upload entirely from VS Code

Arduino IDE is not required. The firmware is ordinary C++ (`main.cpp`) built by
PlatformIO. It still uses the Arduino framework's board support and IMU library;
changing editors does not remove those runtime dependencies. Replacing the framework
with Zephyr or another SDK would be a separate firmware port.

On your PC:

1. Extract this ZIP into a new folder, instead of merging into an older starter.
2. Install VS Code and the **PlatformIO IDE** extension (`platformio.platformio-ide`).
3. Use **File > Open Workspace from File** and select `nova.code-workspace`.
4. Wait for PlatformIO setup to finish. It downloads the toolchain and the IMU library
   from `platformio.ini`; first build needs internet. Do not create a new project.
5. Open `firmware/src/main.cpp` to edit the firmware.
6. Connect the Nano to your PC with a USB data cable.
7. In the PlatformIO sidebar, select the `nano33ble` environment, then **Build**.
8. Select **Upload**. USB bootloader upload is configured by the board definition.
9. Optionally use **Monitor** at 115200 to see `$I,...` and `$S,1,...` frames.
   Close the monitor before letting the bridge open that same port.

The project also provides **Terminal > Run Task** entries named `Nano: Build firmware`,
`Nano: Upload firmware (USB on this host)`, and `Nano: Serial monitor`. These custom
shortcuts require Python on PATH (`python` on Windows, `python3` on Linux/macOS).
PlatformIO's own Build/Upload buttons use its bundled Python and are the fallback.

Equivalent commands from a **PlatformIO Core CLI** terminal, at project root:

```bash
pio run
pio run --target upload
pio device monitor
```

If multiple serial devices confuse detection, set `upload_port` and `monitor_port`
in `platformio.ini` to the actual device, e.g. `COM5` on Windows. Do not copy a PC
COM port setting to the Pi. Leave these unset for automatic detection when moving
between machines. If needed, double-tap Nano reset and select its bootloader port.
If build or upload fails, capture the exact error before changing board settings.

## VS Code on the PC and on the Pi

Use a local VS Code window for a Nano plugged into your PC. After uploading,
move the Nano USB cable to the Pi to run the bridge. Install **Remote - SSH** in
VS Code, connect to your Pi, and open the project folder there. Install the Python
extension in the remote window for the Pi code.

A Remote-SSH window runs terminals/tasks on the Pi. It cannot see a Nano connected
only to the PC. To upload while the Nano remains connected to the Pi, PlatformIO
must also be installed on the Pi and its ARM-host toolchain availability verified.
The simplest initial route is PC upload, then Pi runtime.

The workspace provides `Pi: Build ROS2 package (Remote-SSH)` and
`Pi: Run hardware bridge (Remote-SSH)` tasks. They ask for your installed ROS
name and serial port and source the matching `/opt/ros/<distro>/setup.bash`.
Install dependencies below before using these tasks. They run on Linux/the Pi,
not in local Windows PowerShell. For a nonstandard/source-built ROS setup,
use the manual sourced-terminal commands below instead.

## 2. Put this project on the Pi

Copy the ZIP to your Pi home folder, using your existing SSH/SFTP connection.
For example, on your PC, substituting your real username and hostname:

```text
scp nova_pi_nano_starter.zip YOUR_PI_USER@YOUR_PI_HOST:~/
```

On the Pi:

```bash
cd ~
unzip nova_pi_nano_starter.zip
cd ~/nova_pi_nano_starter/ros2_ws
```

Use your existing ROS2 distribution. Check it in a ROS-sourced terminal:

```bash
echo "$ROS_DISTRO"
ros2 --help
```

If the distribution is blank, list `/opt/ros` and source the installed distribution's
`setup.bash`, for example `source /opt/ros/jazzy/setup.bash` **only if Jazzy is installed**.
Do not install or switch ROS distributions just for this starter. The package uses
ordinary Python ROS2 interfaces, intended for Humble/Jazzy and later; actual runtime
compatibility still needs to be verified on your Pi.

For an Ubuntu Pi using apt-installed ROS2, install the dependencies:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-serial unzip \
  "ros-${ROS_DISTRO:?Source ROS first}-rclpy" \
  "ros-${ROS_DISTRO}-sensor-msgs" "ros-${ROS_DISTRO}-std-msgs" \
  "ros-${ROS_DISTRO}-std-srvs"
```

For a source-built ROS installation or another OS, use its dependency process instead.

Build and source:

```bash
cd ~/nova_pi_nano_starter/ros2_ws
colcon build --symlink-install --packages-select nova_hardware
source install/setup.bash
```

## 3. Find the Nano USB port

Connect the Nano to the Pi with a USB data cable:

```bash
ls -l /dev/serial/by-id/
ls -l /dev/ttyACM*
```

If available, use its `/dev/serial/by-id/...` path rather than the changeable
`/dev/ttyACM0` number. Do not select another device's port.

For `Permission denied` on typical Ubuntu setups:

```bash
sudo usermod -aG dialout "$USER"
```

Then log out of SSH/desktop sessions and reconnect so group membership takes effect.
Do not run the bridge as root as a workaround.

## 4. Start the bridge

In a ROS-sourced terminal, with the workspace sourced:

```bash
cd ~/nova_pi_nano_starter/ros2_ws
source install/setup.bash
ros2 run nova_hardware hardware_bridge --ros-args -p port:=/dev/ttyACM0
```

Replace `/dev/ttyACM0` with your detected port. Leave this terminal running.

In **each additional terminal**, source your installed ROS setup and then:

```bash
source ~/nova_pi_nano_starter/ros2_ws/install/setup.bash
```

Inspect status:

```bash
ros2 topic echo /hardware/status
```

Look for `serial_open: true`, `status_fresh: true`, `imu_fresh: true` in the JSON
string, and `nano.host_heartbeat_alive: true`. `armed` and `servo_output` always
remain false. Status is JSON carried in `std_msgs/msg/String` for this first stage.

Inspect IMU data in another terminal:

```bash
ros2 topic echo /imu/data_raw --qos-reliability best_effort
ros2 topic hz /imu/data_raw
```

The first command runs until Ctrl+C; run the rate command afterward or in another
terminal. Expect approximately 50 Hz, depending on scheduling/USB buffering.
At rest, acceleration magnitude should be near 9.81 m/s². Gyro should be near zero
but may have bias. Tilt the board to check axis directions. Data remains in the
sensor's native axes (`imu_link`), not automatically in the robot body frame.

## 5. Send a test command

Start watching acknowledgements:

```bash
ros2 topic echo /hardware/test_ack
```

In another sourced terminal:

```bash
ros2 service call /hardware/send_test std_srvs/srv/Trigger '{}'
```

This sends twelve zero test angles. A successful service response means **sent**,
not yet acknowledged. Confirm a `DRY_RUN` receipt on `/hardware/test_ack` or
`last_test_ack` in status. No servo should move.

For a custom packet:

```bash
ros2 topic pub --once /hardware/test_targets std_msgs/msg/Float64MultiArray \
  '{data: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'
```

The twelve slots are provisionally FL/FR/RL/RR, with hip-roll/hip-pitch/knee per leg.
Match this explicitly to your URDF before adding actuation. The ±3 rad transport
bound is NOT a mechanical safe range. Accepted values are not applied to servos.

## 6. Check failures before proceeding

- Unplug USB with the bridge running: status becomes disconnected or stale. It must
  not keep claiming fresh IMU data. Reconnect: telemetry resumes; old tests are not replayed.
- Stop the Pi bridge. If you then view Nano status in Serial Monitor without rebooting
  it, `host_heartbeat_alive` becomes 0 after 300 ms without valid H packets. Opening
  the port may reset some board/core configurations; either way it starts unarmed.
- Submit 11 angles or a value outside ±3 rad: the bridge rejects the test.
- Press Nano reset: expect either a reconnect or an uptime-regression indication.
  Reset detection from uptime is diagnostic, not a future arming handshake.

## Moving or versioning the project

Copy the project source, or put it in Git. `platformio.ini` carries Nano dependencies;
`package.xml` carries ROS dependencies. Generated `.pio`, ROS `build/install/log`,
and Python cache folders are ignored and must be rebuilt on the destination.
Do not copy a Windows build or an x86 ROS installation onto the Pi. Rebuild with
PlatformIO for the Nano and colcon on the Pi. Porting to another microcontroller
still requires changes to board configuration and hardware-specific IMU/USB code.

## What comes next

After these checks, send the Pi's `echo "$ROS_DISTRO"`, `/hardware/status` output,
and any errors. Then add per-joint calibration, PCA9685 output, explicit arming,
an independent **motion-command** watchdog, and a supervisor for supported poses.
Do not turn the heartbeat watchdog into a motion watchdog: a healthy bridge can
continue sending heartbeats even when its upstream motion producer has failed.

For the later robot, use named custom target/status messages, session/sequence
validation, a known starting pose, and a physical servo-power cutoff. This starter
does not implement those later motion-stage interfaces or a standing sequencer.

## Validation and limitations

Run portable checks from this project's root:

```bash
python3 -m unittest discover -s tests -v
```

The firmware tests require `g++` and compile the actual `main.cpp` with fake Arduino
and IMU interfaces. Bridge tests mock ROS and serial. They verify framing, CRC,
cross-language command handling, rejection, watchdog expiry, SI conversion,
acknowledgements, stale-status rejection, disconnect/reconnect, and reset handling.
They do not prove USB behavior or real ROS2 integration.

The actual PlatformIO ARM build succeeded for `nano33ble`: Nordic nRF52 platform
11.0.0, Arduino Mbed framework 4.6.0, and BMI270/BMM150 library 1.2.4. These versions
are pinned in `platformio.ini`. The build used 45,088 bytes of RAM and 113,540 bytes
of flash. The bundled framework emitted a default LF crystal clock configuration
warning; it did not prevent linking. No physical Nano or Pi was attached, and real
ROS2 execution was not tested. Upload and runtime verification remain to be done.

IMU values are uncalibrated, with unknown covariance and no orientation estimate.
ROS timestamps are host receipt times; Nano sample timestamps are not synchronized
to ROS time. Add calibration, mounting transform, timestamp alignment, and filtering
before using this stream for a walking policy. IMU initialization failure is reported;
automatic sensor recovery is not implemented (reset/reflash after troubleshooting).

## References

- [PlatformIO Nano board configuration](https://docs.platformio.org/en/latest/boards/nordicnrf52/nano33ble.html)
- [PlatformIO in VS Code](https://docs.platformio.org/en/latest/integration/ide/vscode.html)
- [VS Code Remote-SSH](https://code.visualstudio.com/docs/remote/ssh)
- [Arduino Nano 33 BLE Sense Rev2](https://docs.arduino.cc/hardware/nano-33-ble-sense-rev2)
- [Arduino BMI270/BMM150 library](https://github.com/arduino-libraries/Arduino_BMI270_BMM150)
- [ROS Imu message](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Imu.msg)
- [pySerial API](https://pyserial.readthedocs.io/en/latest/pyserial_api.html)
