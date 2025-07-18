# VIBROSCYPEK

### Ever wanted to use a traditional Polish cheese for wireless communication?
### Here's an answer to your quandary - the VIBROSCYPEK.

> *A shitpost manifested into physical reality.*

<img src="https://github.com/user-attachments/assets/5308cf97-fc00-426b-bfac-abad41bb7d20" alt="A photo of the two VIBROSCYPEK devices" width="500">


## What is this?

The VIBROSCYPEK is a pair of wireless communicators built into 3D-printed shells shaped like *oscypek*, a traditional Polish smoked sheep cheese. Assembled from pieces of e-cigs and crap from a drawer, this project is a glorious testament to junk-drawer engineering.

And yes, they actually talk to each other.

## Features

* **Wireless Communication:** Two *oscypki* can communicate with each other.
* **Simple Pairing Mechanism:** When two devices are brought very close together, they use the signal strength (RSSI) to determine the pairing partner.
* **Custom Enclosure:** A lovingly crafted, 3D-printed *oscypek* shell. Because why not?
* **(Vibro) Haptic Feedback:** The name implies it, right?

## The Tech

This project was a great excuse to learn a new protocol.

* **Hardware:** ESP32-C3 Board
* **Communication:** [ESP-NOW](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html)
    * A nifty connectionless Wi-Fi communication protocol from Espressif that acts as a simple abstraction layer over standard Wi-Fi/Bluetooth, making peer-to-peer communication incredibly easy.

## The Build

Assembled with love, solder, and whatever was available.

* 2x ESP32 development boards
* 2x Vibration motors
* 2x Momentary switches
* 2x Slide switches (for a hard power disconnect)
* 2x Tiny BMS PCBs
* 2x Rechargeable Li-Ion batteries (did you know the 'disposable' e-cigs that people throw out come with a perfectly re-usable battery?!)
* The disembodied guts of various e-cigarettes and other drawer treasures
* 3D printed `oscypek.stl` files (TODO: upload to printables.com)

---
