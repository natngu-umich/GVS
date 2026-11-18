# GVS Project – Context Handoff

Paste this at the start of a new chat, then attach the current versions of the files listed in section 3.

---

## 1. What the project is

I'm building a Unity VR system that counteracts VR cybersickness with **Galvanic Vestibular Stimulation (GVS)**. When the player's virtual vehicle or rig accelerates sideways, a small current is applied across the mastoids, behind the ears. That gives the inner ear a matching roll / sideways sensation. This is a school project, and my teacher reviews it.

- **PC side:** Unity on Windows. BLE goes through `BleWinrtDll.dll`, a WinRT BLE wrapper.
- **Device side:** an ESP32 named `ESP32-Brocoli`, MAC `20:50:0d:d9:81:92`. It has:
  - an MCP4725 12-bit DAC on `Wire1` (SDA 18, SCL 19), driving a Howland current pump to the electrodes;
  - an INA219 current sensor on `Wire`.
- **Safety limit:** current must stay under 3.5 mA. The code hard-caps it at **3000 µA**, in Unity and in the firmware.

## 2. BLE protocol

| Item | UUID | Properties |
|---|---|---|
| Service | `19b10000-e8f2-537e-4f6c-d104768a1214` | |
| Notify ("sensor") characteristic, ESP32 → Unity | `19b10001-e8f2-537e-4f6c-d104768a1214` | READ, NOTIFY, INDICATE |
| Write ("DAC") characteristic, Unity → ESP32 | `19b10002-e8f2-537e-4f6c-d104768a1214` | WRITE, WRITE_NR |

**Packet:** 10 bytes, packed, little-endian, the same in both directions.

```
uint8_t command;    // 0 = STOP, 1 = START, 2 = DATA
uint8_t direction;  // 0 / 1 polarity
int32_t frequency;  // unused for now (0 = DC); the unit is whatever the firmware defines
int32_t amplitude;  // microamps
```

**Sequence:** Unity sends START, then a DATA packet every tick at 100 Hz by default. This is a heartbeat, sent whether or not the value changed. Unity sends STOP on shutdown. The firmware should zero its output on STOP, on BLE disconnect, and when no packet has arrived for 200 ms (watchdog).

## 3. Files in the Unity project (current state)

1. **`GVSController.cs`** (~1060 lines). The main script, standalone: it does **not** use `GVSLink`. It contains:
   - motion tracking and math;
   - the BLE connect sequence (scan by MAC or exact name, or use a fixed `deviceId`; discover the service and characteristics; subscribe to notify; START handshake);
   - a dedicated 100 Hz sender thread;
   - receiving packets from the notify characteristic;
   - a manual test mode;
   - shutdown.
   I added one setting myself: `maxStepPerPacketMicroAmps` (default 100), a cap on how much the current can change per packet, on top of the per-second ramp.
2. **`BLE.cs`**. The managed wrapper around the DLL, patched by Claude. `GVSController` **needs this patched version**: it uses `BLE.PacketWriter`, `BLE.ScanDevices(onFound, onFinished)`, `BLE.WaitForScanToFinish`, `BLE.ResetNativeState` and `BLE.DrainNotifications(max, callback)`. Its bool returns are marshalled as `I1`.
3. **`GVSJoystickTest.cs`**. A small script for testing. Put it on the same GameObject as `GVSController`.
   - It sets `manualMode = true` and drives `manualInput` (−1..+1) from the VR thumbstick's X axis.
   - It uses `UnityEngine.XR` `InputDevices` (`primary2DAxis`, `gripButton`).
   - The signal only flows while grip is held, unless `requireGrip` is false.
   - Not yet tested on the headset. If `controllerFound` stays false, the input method needs adapting to my XR setup.
4. **Possibly still in the project, from an earlier branch:**
   - **`GVSLink.cs`**: a shared sending engine that was split out of the controller.
   - **`Demo.cs`**: a uGUI BLE test bench (scan, select, subscribe, sliders, live streaming through `GVSLink`).
   - I later went back to the standalone `GVSController`, so the current controller does not use `GVSLink`. `Demo.cs` still needs `GVSLink.cs` to compile. Ask me whether these are still in use.

**Nothing has been compiled by Claude.** Its environment had no C# compiler or Unity, so every change was checked by hand only. Test each change in Unity.

## 4. How GVSController works

**Motion → target, on the main thread**
1. **Velocity of `motionSource`**, which must be the vehicle or rig root, **not the HMD camera**, so real head motion doesn't drive stimulation. The `SamplingMode` setting picks how it's sampled:
   - `RigidbodyVelocity`: `rb.GetPointVelocity(headPosition)`, in FixedUpdate;
   - `FixedUpdatePosition`: position difference, in FixedUpdate;
   - `LateUpdatePosition`: position difference, in LateUpdate;
   - `Auto` picks one of these.
2. **World-space acceleration:** `a = (v − v_last) / dt`.
3. **Spike / teleport rejection:** if |a| > `spikeRejectAccel` (50) or the speed exceeds `teleportSpeedThreshold` (50 m/s), the tick contributes 0. Call `ResetKinematicsState()` from teleport or respawn code.
4. **Ear-to-ear component:** `a_lat = Vector3.Dot(a, headTransform.right)`. Acceleration is computed from the vehicle first and only then projected onto the head's current right axis. Head turning therefore changes which component is felt but can't create acceleration by itself. Front-back and vertical components relative to the head are discarded, since there is a single L/R channel. Optional: `yawRateGain` adds yaw rate to the signal (off by default).
5. **Smoothing:** an EMA with `α = 1 − e^(−smoothingSpeed·dt)` (smoothingSpeed 50, a ~20 ms time constant).
6. **Mapping to current:** below `accelDeadzone` (0.5 m/s²) the target is 0. Above it, `amp = cap × (|a| − 0.5)/(15 − 0.5)`, with `maxExpectedAccel` = 15. The sign sets the direction, and `invertDirection` flips it. The result is a signed target in µA, handed to the sender with a timestamp.

**Manual mode:** if `manualMode` is on, the tracking is skipped and `manualInput × maxExpectedAccel` is used as `a_lat`, through the same smoothing, mapping, ramp and cap. With `manualRelativeToRig`, it's multiplied by `dot(motionSource.right, head.right)`, which demonstrates the head-turn compensation.

**Sender thread** (deadline-scheduled at `sendRateHz`, 100):
- **Zeroing:** the target becomes 0 if it is stale (> `staleTimeoutMs`, 100 ms, since it was last updated), if the app is paused, unfocused (`zeroOutputWhenUnfocused`) or disabled, or if the link is lost.
- **Hard cap:** clamped to `maxAmplitudeMicroAmps` and the hard limit of 3000.
- **Ramp limit:** the signed value moves at most `slewRateMicroAmpsPerSec` (10000; 0 → 3 mA takes 300 ms) and at most `maxStepPerPacketMicroAmps` per packet. Polarity changes pass through zero.
- **Every tick:** it sends DATA, and START first if it needs re-arming.
- **Link loss:** after 10 consecutive failures the link is marked lost, retried at 2 Hz, and START is re-sent on recovery.
- **Hung writes:** a write blocked for more than 10 s is treated as the DLL hang. That thread is abandoned and replaced, at most 3 times.
- **Notifications:** about 10 times a second it drains up to 16 notifications into a queue.

**Receiving:** `Update()` parses each 10-byte notification into `lastReceivedPacket` (Inspector) and `GVSController.LatestReceivedPacket` (static), and logs it if `logReceivedPackets` is on. Payloads that aren't 10 bytes log a warning and are ignored.

**Shutdown** (OnApplicationQuit / OnDestroy), on a worker thread bounded at 3 s: stop the sender, send STOP (with one retry), wait 100 ms, then call native `Quit()` only if no write is still in flight.

**Telemetry fields:** `rawLateralAccel`, `smoothedLateralAccel`, `targetAmplitudeSigned`, `currentOutputAmplitude/Direction`, `measuredSendRateHz`, `consecutiveWriteFailures`, `rejectedSpikes`, `notificationsReceived`, `isBleReady`.

## 5. Facts about BleWinrtDll.dll (from Claude's disassembly)

- **Write type:** `SendData` uses WriteWithoutResponse. A "successful" write means Windows queued it, not that the ESP32 received it.
- **Always blocks:** `SendData` ignores its `block` argument and always waits for the write to finish.
- **Rare permanent hang:** a missed-signal race in the DLL can make `SendData` occasionally never return. That is the reason for the hung-write recovery.
- **Misleading error text:** a failed write reports "Error subscribing to characteristic …". It's really a write error.
- **Editor second-Play bug:** `Quit()` sets an internal flag that only `StartDeviceScan()` clears. That broke a second Play in the editor when using a fixed `deviceId`; `BLE.ResetNativeState()` works around it.
- **Notification queue:** notifications queue inside the DLL until `PollData` reads them. Undrained, the queue grows forever.
- **Shared state:** the DLL is shared, so don't run `Demo` and `GVSController` in the same scene.

## 6. Current firmware (a template, will change)

My current sketch:
- **BLE callbacks:** `onWrite` validates the packet (10 bytes, command ≤ 2, direction ≤ 1, amplitude ≥ 0) and puts it in a 1-slot FreeRTOS queue. `onDisconnect` queues a STOP.
- **DACTask** (priority 3, core 1) receives from the queue with a 200 ms timeout and calls `writeHardware`.
- **writeHardware** maps the signed current bipolarly: `map(amp·dir, −3000, 3000, 0, 4096)`. So 0 µA means DAC mid-scale (2048).
- **StatusTask** sends a GVS packet on the notify characteristic: `{CMD_DATA, lastDir, 0, measured_uA}`.
- The ramp limit and polarity-through-zero are commented out in the firmware.

**Known problems Claude found in this firmware, not fixed yet** (the first four are safety issues):

1. **Watchdog never fires.** It only zeroes if `output_uA != 0`, but `output_uA` is never updated, so the last current stays on when packets stop.
2. **`map(..., 0, 4096)` overshoots the 12-bit DAC.** Max is 4095. With the Adafruit library, a value of 4096 wraps to 0, so a full positive command would output full **negative**. Use 4095.
3. **Start-up outputs full negative.** `setup()` calls `dac.setVoltage(0)`, which under the bipolar mapping means −3000 µA at start-up, not zero. Zero is 2048, and it depends on the analog stage really being centred at mid-scale.
4. **START/STOP are ignored for DATA.** The `armed` check is commented out, so DATA drives the output even after STOP or before START.
5. **StatusTask never pauses.** Its `vTaskDelay` sits before the `for(;;)`, so it reads the INA219 and notifies non-stop. That floods BLE and Unity's log; it needs a delay inside the loop. Its connected check also runs only once, outside the loop.
6. **Per-call Serial printing.** `writeHardware` prints every call (100+/s), which causes the serial delay we saw earlier.
7. **Packet direction byte is wrong when negative.** `lastDir` is −1/+1, so the notify packet's direction byte becomes 255 instead of 0. Also, `p.direction` is reused after a watchdog timeout.
8. **Calibration constant is unused and the wrong unit.** `UA_PER_DAC_COUNT` (0.000805) is never used and is volts per count, not µA per count. The real µA-per-count of the Howland stage still needs measuring.

## 7. My preferences and history

- **Code size:** I prefer lean changes. An earlier rewrite nearly doubled the controller, and I asked why. Explain what you add and don't add unrequested features.
- **Explanations:** explain the math and logic when asked. I'm a student, and my teacher reviews this.
- **Things already answered:**
  - Send rate: 100 Hz in the current controller.
  - The perceived lag mostly came from the 10000 µA/s ramp limit.
  - The serial delay came from per-packet printing on the ESP32.
  - The head-relative math is explained in section 4.
- **Manual testing:** my teacher wanted a manual way to induce acceleration. Claude suggested the joystick → direct acceleration mapping (`GVSJoystickTest`) instead of a constantly accelerating "ghost" object, which would run away and trip the teleport filter. Other ideas offered: preset waveforms (step, ramp, sine), or a visible cart on a rail with capped speed.

## 8. Open to-dos

- Test the current `GVSController` + `GVSJoystickTest` in Unity with the headset.
- Fix the firmware safety issues in section 6, then calibrate µA per DAC count with the INA219.
- Decide whether `Demo.cs` / `GVSLink.cs` stay in the project.
- Confirm the polarity sign by feel (`invertDirection`).