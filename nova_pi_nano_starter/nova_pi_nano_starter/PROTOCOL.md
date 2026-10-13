# Stage-1 protocol v1

115200 baud setting, USB CDC, ASCII. No arbitrary debug text on the data port.
Each frame is `$PAYLOAD*CCCC\n`, where CCCC is four hex digits of CRC-16/CCITT-FALSE
(polynomial 0x1021, initial 0xFFFF, no reflection, xor-out 0), calculated over the
ASCII payload only. Known vector `123456789` -> `29B1`. Maximum line 255 bytes,
excluding newline. A fresh `$` resynchronizes; oversized/corrupt lines are dropped.
No CR before LF. Counters/timestamps are unsigned 32-bit decimal.

| Direction | Payload | Meaning |
|---|---|---|
| Pi → Nano | `H,seq` | Host-link heartbeat; Pi sends every 100 ms |
| Pi → Nano | `C,seq,q0,...,q11` | Twelve finite test angles, each within ±3 radians |
| Nano → Pi | `A,seq,DRY_RUN` | Parsed test command, not actuated |
| Nano → Pi | `I,ms,ax,ay,az,gx,gy,gz` | IMU snapshot; m/s² and rad/s |
| Nano → Pi | `S,1,ms,imu_ok,link_alive,have_cmd,last_seq,rx_errors,tx_drops` | Status every 200 ms |

`ms` is Nano uptime, wraps every ~49.7 days. I combines latest fresh accel/gyro
readings; they are not guaranteed simultaneous. Frame timestamps use millis().
Accelerometer/gyro are sampled when the library reports availability. Telemetry
is downsampled to at most 50 Hz, and snapshots older than 50 ms are not sent.

`link_alive` expires after 300 ms without valid H frames. C packets never refresh
it. In this non-actuating stage, C is accepted even if link_alive is false.
`have_cmd` distinguishes sequence zero from no command. S carries the latest
accepted sequence if the immediate A packet was dropped due to a busy TX buffer.

The Nano's bounded output buffer may drop a new frame if busy; `tx_drops` reports
this. Input errors increment `rx_errors`. No commands are retransmitted by the Pi;
an unacknowledged test times out after 1 second. The bridge supports up to eight
pending tests, suitable for manual testing, not a motion stream.

No ordering/replay rejection or authenticated session is implemented in this
transport demonstration. CRC detects corruption, not malicious traffic. No pulse,
arming, joint-state feedback, or servo-applied-target message exists in this stage.
Add motion-specific semantics before extending this protocol to powered servos.
