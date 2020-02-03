QMK Converters
======================

My collection of vintage keyboard converters for use with QMK.

## Converters
att56k_usb - USB converter for AT&T 56K series keyboards

## Build
The converters here are intended to be used with QMK (Quantum Mechanical Keyboard Firmware) which can be found on Github at https://github.com/qmk/qmk_firmware

To build a converter, copy the converter folder into the
`qmk_firmware/keyboard/converter` directory of QMK.

From the root of QMK, build the converter:
`$ make converter/<converter>:default`

To use a custom keymap:
`$ make converter/<converter>:<keymap>`
