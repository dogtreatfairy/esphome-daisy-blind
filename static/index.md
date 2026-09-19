# Daisy Blind

ESPHome firmware for stepper-motor window blinds (ESP8266 NodeMCU + A4988 + 28BYJ-48,
wired like The Hookup's BlindsMCU). Several blinds sharing one power run find each
other over WiFi and take turns drawing motor current, so they move together without
starving each other.

Source, wiring and full documentation: [github.com/dogtreatfairy/esphome-daisy-blind](https://github.com/dogtreatfairy/esphome-daisy-blind)

# Install

Plug the NodeMCU into this computer with a USB cable, then press the button. This
works in Chrome or Edge.

<esp-web-install-button manifest="firmware/daisy-blind.manifest.json"></esp-web-install-button>

<script type="module" src="https://unpkg.com/esp-web-tools@10/dist/web/install-button.js?module"></script>

After flashing, the installer offers to connect the board to your WiFi. If you skip
that, the board opens an access point named `daisy-blind-xxxxxx`; join it and pick
your network in the page that appears.

# Adopt into ESPHome

Once the board is on your network, the ESPHome dashboard in Home Assistant lists it
under **Discovered**. Press **Adopt**. The dashboard creates a config for it that
imports this project, adds an encryption key, and reflashes it over the air. Home
Assistant then offers the device for integration.

# Configure

Open `http://daisy-blind-xxxxxx.local/` or the device page in Home Assistant. Move the
blind to fully closed with **Manual position** and the **Nudge** buttons, press **Save
current position as closed limit**, do the same for open, and give every blind on the
same power wire the same **Group**. The README on GitHub covers every setting.
