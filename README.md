# Daisy Blind

ESPHome firmware for stepper-motor window blinds that share one power run. Each blind
runs on its own ESP8266 NodeMCU with an A4988 and a 28BYJ-48, wired like
[The Hookup's BlindsMCU](https://github.com/thehookup/Motorized_MQTT_Blinds). The
blinds find each other over WiFi and take turns drawing motor current, so the shared
wire never sees every motor at once, yet they start and stop together and look like they
move in unison.

**Install from your browser:** <https://dogtreatfairy.github.io/esphome-daisy-blind/>

## What's in the repo

| Path | What it is |
|---|---|
| `daisy-blind.yaml` | Small entry point that your ESPHome dashboard imports when you adopt a device. It pulls `daisy-blind-core.yaml` fresh from GitHub on every build. |
| `daisy-blind-core.yaml` | The actual configuration. |
| `daisy-blind.factory.yaml` | Wraps the core with adoption, USB WiFi provisioning and self-update. GitHub Actions compiles this into the firmware the web installer flashes. |
| `components/daisy_blind/` | External component: stepper driver, peer discovery, firing-order coordination. |
| `static/` | The GitHub Pages installer site. |
| `reference/blinds_original_hookup.yaml` | The single-blind ESPHome port this started from, unchanged, for reference. |

## Wiring

Same as BlindsMCU:

| NodeMCU | A4988 |
|---|---|
| D7 | STEP |
| D6 | DIR |
| D5 | ENABLE (active low, hence `inverted: true` on `sleep_pin`) |

If your board drives the A4988 SLEEP pin instead of ENABLE, remove `inverted: true`
from `sleep_pin` in your adopted config.

## Getting a blind running

1. **Flash.** Open <https://dogtreatfairy.github.io/esphome-daisy-blind/> in Chrome or
   Edge, plug the NodeMCU in over USB, and press Install. When flashing finishes the
   installer offers to connect the board to your WiFi (Improv over serial). Do that, or
   skip it and join the `daisy-blind-xxxxxx` access point the board opens and pick your
   network from the page that appears.
2. **Adopt.** In Home Assistant, open the ESPHome dashboard. The board appears under
   **Discovered**. Press **Adopt**. The dashboard writes a small config for it that
   imports `daisy-blind.yaml` from this repo as a package, generates an API encryption
   key, and reflashes it over the air. Home Assistant then discovers the device and asks
   for that key, which is already in the dashboard config.
3. **Repeat** for each board. One firmware serves all of them because the hostname
   includes the MAC address.
4. **Calibrate** each blind from its web page or Home Assistant device page (below).
5. **Rename** a blind by editing the `friendly_name` substitution in the config the
   dashboard created for it, then Install. Leave `name` alone: that is the hostname Home
   Assistant and the dashboard use to find the board, and changing it makes the device
   look like a new one. Renaming the device in Home Assistant's UI also works.

Later updates come three ways: the ESPHome dashboard rebuilds from the latest `main`
whenever you press Install, the device's own **Firmware update** entity pulls the latest
published release, and the device web page has a firmware upload form.

Adopted blinds never need re-adopting when this repo changes. Every Install from the
ESPHome dashboard fetches the latest configuration and component code from GitHub.

If you'd rather compile yourself, copy `daisy-blind.yaml` into your ESPHome directory
and install it over USB. Everything else is fetched from GitHub automatically.

## How the current sharing works

Think of the blinds as cylinders in an engine. Every blind broadcasts a small UDP
beacon on the LAN once a second (every 300 ms while moving) saying which **Group** it
belongs to, its **Firing order**, and whether it is moving. Nothing else is configured.

When a blind receives a position command it waits a quarter of a second so that peers
that got the same command can announce themselves, then looks at who else in the group
is moving:

- **Nobody else:** it moves at full speed, no gaps. Opening blind 2 on its own is never
  slowed down by the others.
- **Others too, "Take turns" mode (default):** a fixed one-second cycle is divided evenly
  between the blinds that are moving. Four movers get 250 ms each, two get 500 ms each.
  Each blind steps only in its own share, in firing order, and puts its A4988 to sleep in
  between, so exactly one motor is energised at any instant. All of them start together
  and finish at about the same time.
- **Others too, "One at a time" mode:** the blind with the lowest firing order moves to
  its target while the rest wait, then the next one goes, and so on. If any blind in the
  group is set to this mode, the whole group uses it.

The set of movers is re-evaluated continuously, so a new command arriving mid-move just
changes who is in the cycle. A blind that has already started keeps its place ahead of a
newcomer in one-at-a-time mode, so nothing pauses halfway.

Slot boundaries are agreed by adopting the millisecond clock of the peer with the lowest
MAC address. No NTP or Home Assistant involvement is needed, and it works without
internet. Ties in firing order are broken by MAC address, so leaving every blind at
order 1 still works; set 1, 2, 3, 4 if you care which fires first.

Bluetooth isn't an option here because the ESP8266 has no Bluetooth radio, and it
wouldn't help anyway: the coordination needs tens of milliseconds of accuracy and LAN
broadcast delivers that comfortably.

## Settings

Open `http://daisy-blind-xxxxxx.local/` or the device page in Home Assistant. The device
page is laid out for a phone, follows its light or dark setting, and is split into Blind,
Calibration, Coordination, Setup and Device sections. Everything below is stored on the
device and survives reboots and power loss.

| Setting | Meaning |
|---|---|
| **Label** | A friendly name that peers show in their "Peers in group" list, e.g. `Kitchen left`. |
| **Group** | Blinds with the same group number share the current budget. Default 1. |
| **Firing order** | This blind's place in the cycle, 1–8. Default 1. |
| **Coordination mode** | Take turns (interleaved, all move together) or One at a time. |
| **Invert direction** | Flip if the blind opens when you ask it to close. |
| **Open limit (steps)** | Step count that means fully open. Default 785. |
| **Closed limit (steps)** | Step count that means fully closed. Default 0. |
| **Motor speed (steps per second)** | Step rate. Default 250. |
| **Torque limit** | Motor strength, 10 to 100%. Default 100. Lower it so a blind that reaches an end stop stalls instead of stripping its gears. See below. |
| **Nudge size (steps)** | How far the two Nudge buttons move the blind. Default 10. |
| **Mark as fully closed / open** | Tell the blind it is exactly at its closed or open limit right now. Use this to correct a blind that has lost track. |
| **Hotspot LED** | Blink the on-board LED while the blind has no WiFi connection. Default on. |
| **Group control** | When on, opening or closing this blind also commands every blind in the group, and this blind follows their commands. Leave off if Home Assistant controls each blind individually. |

### Calibrating a blind

There is no automatic homing. Driving a 28BYJ-48 into a hard stop strips its plastic
gears, and neither the A4988 nor the motor can sense a stall, so calibration is manual
and the blind never deliberately hits an end.

1. Type a value into **Manual position (steps)** to move the blind roughly to fully
   closed. If it goes the wrong way, toggle **Invert direction**. Use **Nudge toward
   open** and **Nudge toward closed** to creep the last few steps.
2. Press **Save current position as closed limit**.
3. Move to fully open the same way and press **Save current position as open limit**.

### Correcting a blind that has lost track

If a blind is physically somewhere other than where it thinks it is, don't command it
open or closed. Nudge it by hand from the device page until it is exactly fully closed,
then press **Mark as fully closed**. The same works for fully open with **Mark as fully
open**. The limits stay as they are; only the blind's idea of where it is changes.

### How the blind remembers

The blind keeps its position and every setting on this page in one record of its own.
The record carries a marker, a sequence number and a checksum, and at boot the blind scans
the whole settings area of flash for it. So it's found even when a firmware update changes
the order ESPHome stores other things in, which is what used to make blinds forget. The
position is cached every second while moving and written to flash when a move finishes.
Before any firmware update or restart, the blind stops and writes its exact position.

If a blind ever boots with no trustworthy position, **Position known** turns off. The blind
then refuses open, close and position commands until you use **Mark as fully closed** or
**Mark as fully open**. Nudges and Manual position still work so you can get it there.

### Pairing blinds

Give each blind the same **Group**. The **Peers in group** readout lists every other blind
it can hear, with its firing order, label, hostname and position. **Coordination status**
shows this blind's place in the cycle and which mode is in effect. That is all pairing
takes; there is no master to configure.

### Setting the torque limit

The 28BYJ-48's plastic gearbox can strip itself if the motor keeps pushing at an end stop.
The torque limit reduces the motor's current by rapidly switching the A4988's ENABLE pin,
so a weaker motor stalls harmlessly instead. Stalling doesn't damage a stepper. Lower
current also means less heat. The only cost of going too low is that the blind may skip
steps on a stiff part of its travel and lose track of where it is.

Tune each blind on its own:

1. Make sure the blind's position is correct, then set **Torque limit** to 80%.
2. Run a full close and a full open. Watch that it reaches both ends cleanly and smoothly.
3. Lower it by 10% and repeat, until a run stutters, buzzes in place, or stops short.
4. If it lost track, nudge it back and press **Mark as fully closed**.
5. Set the limit about 20% above the last level that worked, and run it a few more times.

The effect depends on the motor, driver and supply, so the percentage isn't a precise force.
It needs the ENABLE pin wired to `sleep_pin`, as in the standard wiring. Every move also
starts and ends at 180 steps per second rather than crawling, because a stepper is strongest
at low speed and the ends are where it meets the stops.

### Homing

Homing drives the blind into its closed end stop at a deliberately weak **Homing torque**,
lets it stall there, takes that stop as a fixed reference, and then sends the blind back to
where it last was (or to wherever you commanded it while it was homing). It runs at full
speed with no slow approach, because a stepper is weakest when fast.

**Set the homing torque before you use it.** It must be strong enough to move the blind but
too weak to strip the gears when it reaches the stop:

1. With the blind's position correct, set **Homing torque** to 20% and press **Home now**.
2. Watch it. It should close, buzz briefly against the stop without forcing it, then return.
3. If it didn't reach the closed end, raise it by 5% and try again. Use the lowest value
   that reaches the stop every time.

The first successful home records the stop at your current **Closed limit**. If the stop is
a little past where you consider fully closed, nudge the blind there after homing and press
**Save current position as closed limit**. The open limit can be re-saved the same way.

| Setting | Meaning |
|---|---|
| **Home now** | Home this blind. |
| **Home whole group** | Home every blind in the group, one at a time. |
| **Home after power loss** | Home automatically after the power comes back. Not after a restart or update. Off by default. |
| **Homing torque** | Motor strength while homing, 10 to 100%. Default 30. |
| **Homing overtravel (steps)** | Extra steps beyond the expected distance, so the stop is always reached. Default 150. |

After a power cut, homing waits until WiFi is up and the other blinds have been heard, so a
group comes back one at a time. Pressing Stop while homing cancels it and leaves the position
unknown until the blind is homed or marked.

## Home Assistant

Each blind appears as a `cover` with position, plus all the settings above as config
entities. To move several blinds together from Home Assistant, either create a cover group
in Home Assistant (they interleave automatically), or enable **Group control** on the blinds
and command any one of them.

## Notes on the electrical side

The A4988 is a chopper driver, so a moving or holding motor draws roughly the current set
by its Vref regardless of step rate. The coordination cuts the peak load to one motor at a
time, and every burst of motion ramps up from 180 steps per second rather than starting at
full speed, which is where most skipped steps and the sharpest current spike come from.
If a blind still skips steps, lower Vref on that driver or lower **Motor speed**.

The NodeMCU's on-board LED blinks while the blind is not connected to WiFi, which is when
the setup hotspot is up, and turns off once connected. The **Hotspot LED** switch disables
it.

### WiFi fallback

If the known network can't be reached for 30 seconds the `daisy-blind-xxxxxx` hotspot
comes up so you can reach the device or point it at a new network. The blind keeps
scanning for the known network the whole time and drops the hotspot as soon as it
reconnects, so a router reboot heals itself. While someone is actually using the captive
portal the reconnect attempts back off so they don't interrupt setup. After a power cycle
the blind always tries the known network first.

### Firmware version

**Firmware version** on the device page shows the release the blind is running, such as
`26.9.3`. Boards flashed from the installer report the release they were built from.
Boards rebuilt by the ESPHome dashboard report the latest published release at the time
they were compiled, which the Release workflow stamps into `daisy-blind-core.yaml`.

The sync traffic is unauthenticated UDP that stays on your LAN; anyone on the LAN could
send a move command. The factory firmware ships without an API key so the dashboard can
adopt it; adoption adds one.

## Releasing (maintainers)

Pushing to `main` runs **Release Drafter**, which drafts a CalVer release, and **Build**,
which compiles the factory firmware and attaches it to the draft. When the draft no longer
carries the "do not publish" notice, run the **Release** workflow from the Actions tab.
That publishes the release and **Publish Pages** redeploys the installer site with the new
firmware. This is the same pipeline as
[esphome-project-template](https://github.com/esphome/esphome-project-template).
