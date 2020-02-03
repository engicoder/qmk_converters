AT&T 56K 122 key terminal keyboards
======================

## Interfacing

The 56k keyboards use a 16 bit data packet. Depending on the keyboard the 
transmission maybe synchronous with a 9.6Kz clock frequency or ansynchronous
using one start bit and 9600 baud. In the synchronous version the data is ready on the falling edge of the clock line.

The connector is a 6P6C modular jack with the following pinout
1. clock (out)
2. clicker (in)
3. data (out)
4. Vcc +5V (in)
5. Enable (in)
6. Ground (in)

The clicker line is active low and drives an electronic buzzer. It is not a direct drive and only seems to work correctly when given a ~0.3ms pulse. 
The enable line is active low. If not pulled low, the keyboard will not generate any output. 

The data frame is 16 bits of which the top 8 bits are always 0x7F.
The lower 8 bits contain the scan code. MAKE or BREAK is indicated in the 
most significant bit (bit 7), a value of 1 indicating a MAKE

## Build

From the root qmk_firmware directory execute:

    $ make converter/att56k_usb

To specify a custom keymap in the folder <custom_keymap>, execute

    $ make converter/att56k_usb:custom_keymap

## Keymap
The default keymap is based on the standard PC 122 key layout.

The F24 key has been mapped to cycle through the clicker modes:
    OFF -> MAKE -> MAKE/BREAK
