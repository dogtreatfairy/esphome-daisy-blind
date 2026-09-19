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
| `daisy-blind.yaml` | Core configuration. This is what your ESPHome dashboard imports when you adopt a device. |
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

If you'd rather compile yourself, copy `daisy-blind.yaml` into your ESPHome directory
and install it over USB. The component is fetched from GitHub automatically.

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

Open `http://daisy-blind-xxxxxx.local/` or the device page in Home Assistant. Everything
below is stored on the device and survives reboots and power loss.

| Setting | Meaning |
|---|---|
| **Label** | A friendly name that peers show in their "Peers in group" list, e.g. `Kitchen left`. |
| **Group** | Blinds with the same group number share the current budget. Default 1. |
| **Firing order** | This blind's place in the cycle, 1–8. Default 1. |
| **Coordination mode** | Take turns (interleaved, all move together) or One at a time. |
| **Invert direction** | Flip if the blind opens when you ask it to close. |
| **Open limit (steps)** | Step count that means fully open. Default 750. |
| **Closed limit (steps)** | Step count that means fully closed. Default 0. |
| **Motor speed (steps per second)** | Step rate. Default 250. |
| **Nudge size (steps)** | How far the two Nudge buttons move the blind. Default 10. |
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

The step counter is arbitrary; only the two saved limits matter. The blind's position is
written to flash within a second during a move and within five seconds of any change, so
after a power cut it comes back knowing where it was to within a few steps. If a blind
ever loses steps under load, just nudge it back and re-save the limit it drifted from.

### Pairing blinds

Give each blind the same **Group**. The **Peers in group** readout lists every other blind
it can hear, with its firing order, label, hostname and position. **Coordination status**
shows this blind's place in the cycle and which mode is in effect. That is all pairing
takes; there is no master to configure.

## Home Assistant

Each blind appears as a `cover` with position, plus all the settings above as config
entities. To move several blinds together from Home Assistant, either create a cover group
in Home Assistant (they interleave automatically), or enable **Group control** on the blinds
and command any one of them.

## Notes on the electrical side

The A4988 is a chopper driver, so a moving or holding motor draws roughly the current set
by its Vref regardless of step rate. The coordination cuts the peak load to one motor at a
time. If a blind still skips steps, lower Vref on that driver or lower **Motor speed**.

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
