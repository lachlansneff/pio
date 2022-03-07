# An open-source implementation of PIO

PIO (aka programmable IO) is a versatile hardware interface. At runtime, it can be programmed
to support a large number of IO standards, including ones that haven't been invented yet!

A PIO block is essentially a small group of simple processors that independently execute sequential
programs that manipulate GPIOs and transfer data with precise timing.

PIO originated as a feature of the Raspberry Pi 2040 MCU.

This is a re-implementation of PIO in Amaranth, an HDL written in Python. Currently, this is still
a work-in-progress and contributions are welcome!
