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
| `components/daisy_blind/` | External component: stepper driver, peer discovery, slot sync, homing. |
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

Later updates come three ways: the ESPHome dashboard rebuilds from the latest `main`
whenever you press Install, the device's own **Firmware update** entity pulls the latest
published release, and the device web page has a firmware upload form.

If you'd rather compile yourself, copy `daisy-blind.yaml` into your ESPHome directory
and install it over USB. The component is fetched from GitHub automatically.

## How the current sharing works

Every blind broadcasts a small UDP beacon on the LAN once a second (every 300 ms while
moving). From those beacons each device builds a list of peers in its **Sync group**.

When a blind has to move it only steps during its own **time slot**. The slot cycle is
`slot length × number of blinds currently moving`. With the default 250 ms slots and four
blinds moving, blind 1 steps during 0–250 ms of each second, blind 2 during 250–500 ms,
and so on. Between slots the A4988 is put to sleep, so only one motor is energised at any
instant. Idle blinds are not part of the cycle, so a blind moving alone runs at full
speed with no gaps.

All devices in a group agree on slot boundaries by adopting the millisecond clock of the
peer with the lowest MAC address. No NTP or Home Assistant involvement is needed and it
works without internet.

Slot assignment is automatic (sorted by MAC address). If you prefer fixed slots, set
**Sync slot** to 1–4 on each blind. Don't mix Auto and fixed within one group.

The trade-off: with four blinds moving, each one only steps a quarter of the time, so a
full travel takes about four times longer than a lone blind. Raise **Motor speed** if the
motors cope, or shorten the slot length for smoother-looking motion.

## Settings

Open `http://daisy-blind-xxxxxx.local/` or the device page in Home Assistant. Everything
below is stored on the device and survives reboots and power loss.

| Setting | Meaning |
|---|---|
| **Label** | A friendly name that peers show in their "Peers in group" list, e.g. `Kitchen left`. The Home Assistant device name is renamed in Home Assistant itself. |
| **Invert direction** | Flip if the blind opens when you ask it to close. |
| **Open limit (steps)** | Steps from home to fully open. Default 750. |
| **Closed limit (steps from home)** | Position considered fully closed. Usually 0. Set a few steps if you want the blind to back off the hard stop. |
| **Homing distance (steps)** | How far the blind drives toward the closed stop when homing. Must exceed the full travel so it is guaranteed to hit the stop. Default 1500. |
| **Motor speed (steps per second)** | Step rate. Default 250. |
| **Sync group** | Blinds with the same group number share the current budget. Default 1. |
| **Sync slot** | Auto, or a fixed slot 1–4. |
| **Sync slot length (ms)** | Length of each blind's turn. Default 250. |
| **Sync with peers** | Turn off to ignore peers and always move at full speed. |
| **Sleep driver between slots** | De-energise the A4988 while waiting for the next turn. Leave on to actually reduce current. |
| **Home after power loss** | Re-home automatically 20 s after a cold power-up (not after a software restart or OTA update). |
| **Group control** | When on, opening or closing this blind also commands every blind in the group, and this blind follows their commands. Leave off if Home Assistant controls each blind individually. |

### Calibrating a blind

1. Press **Home now**. The blind drives into the closed stop and declares that position 0.
2. Type a value into **Manual position (steps)** to jog the blind. Increase it until the
   blind is exactly where you want fully open. If it goes the wrong way, toggle
   **Invert direction**, home again and repeat.
3. Press **Save current position as open limit**.
4. Optionally jog back toward closed to the point you want as fully closed and press
   **Save current position as closed limit**. Leave this at 0 to use the hard stop.
5. Set **Homing distance** to comfortably more than the open limit (1.5× to 2× is fine).

Position is remembered in flash and restored after a restart. After a power failure, if
**Home after power loss** is on, all blinds in the group re-home together (taking turns in
their slots) about 20 seconds after power returns, and report as closed.

### Pairing blinds

Give each blind the same **Sync group**. The **Peers in group** readout lists every other
blind it can hear, with its label, hostname and position. **Sync status** shows which slot
this blind has and whose clock it follows. That is all pairing takes; there is no master to
configure.

## Home Assistant

Each blind appears as a `cover` with position, plus all the settings above as config
entities. To move several blinds together from Home Assistant, either create a cover group
in Home Assistant (they interleave automatically), or enable **Group control** on the blinds
and command any one of them.

## Notes on the electrical side

The A4988 is a chopper driver, so a moving or holding motor draws roughly the current set
by its Vref regardless of step rate. The slot scheme cuts the peak load to one motor at a
time. If a blind still skips steps, lower Vref on that driver, lower **Motor speed**, or
increase **Homing distance** so homing always reaches the stop.

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
