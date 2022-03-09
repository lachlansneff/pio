from enum import Enum
from typing import Iterable
from amaranth import *
from amaranth.lib.io import Pin
from .utils import bitmask, rotate_right

class Outset(Enum):
    OUT = 0
    SET = 1

class OutputDest(Enum):
    PINS = 0
    PINDIRS = 1

class MappedOutput(Record):
    def __init__(self):
        layout = [
            ("pins_data", 32),
            ("pins_mask", 32),
            ("pindirs_data", 32),
            ("pindirs_mask", 32),
        ]
        super().__init__(layout)
    

class OutputMapping(Elaboratable):
    def __init__(self):
        # Inputs
        self.outset_stb = Signal()
        self.outset_dest = Signal(OutputDest)
        self.outset_inst = Signal(Outset)
        self.outset_data = Signal(32)

        self.sideset_stb = Signal()
        self.sideset_data = Signal(32)

        self.set_count = Signal(3)
        self.set_base = Signal(5)
        self.out_count = Signal(5)
        self.out_base = Signal(5)
        self.sideset_count = Signal(5)
        self.sideset_base = Signal(5)
        self.sideset_dest = Signal(OutputDest)

        # Outputs
        self.out = MappedOutput()

    def elaborate(self, platform):
        m = Module()

        outset_data = Signal(32)
        outset_mask = Signal(32)
        sideset_data = Signal(32)
        sideset_mask = Signal(32)

        with m.If(self.outset_stb):
            base = Mux(self.outset_inst == Outset.SET, self.set_base, self.out_base)
            count = Mux(self.outset_inst == Outset.SET, self.set_count, self.out_count)
            mask = bitmask(count)
            rotated_data = rotate_right(self.outset_data & mask, base)
            rotated_mask = rotate_right(mask, base)

            m.d.comb += [
                outset_data.eq(rotated_data),
                outset_mask.eq(rotated_mask),
            ]

        with m.If(self.sideset_stb):
            mask = bitmask(self.sideset_count)
            rotated_data = rotate_right(self.sideset_data & mask, self.sideset_base)
            rotated_mask = rotate_right(mask, self.sideset_base)
            m.d.comb += [
                sideset_data.eq(rotated_data),
                sideset_mask.eq(rotated_mask),
            ]
        
        with m.Switch(Cat(self.outset_dest, self.sideset_dest)):
            with m.Case(0b00): # sideset = PINS, outset = PINS
                m.d.comb += [
                    self.out.pins_data.eq((outset_data & ~sideset_mask) | sideset_data),
                    self.out.pins_mask.eq(outset_mask | sideset_mask),
                ]
            with m.Case(0b01): # sideset = PINS, outset = PINDIRS
                m.d.comb += [
                    self.out.pindirs_data.eq(outset_data),
                    self.out.pindirs_mask.eq(outset_mask),
                    self.out.pins_data.eq(sideset_data),
                    self.out.pins_mask.eq(sideset_mask),
                ]
            with m.Case(0b10): # sideset = PINDIRS, outset = PINS
                m.d.comb += [
                    self.out.pins_data.eq(outset_data),
                    self.out.pins_mask.eq(outset_mask),
                    self.out.pindirs_data.eq(sideset_data),
                    self.out.pindirs_mask.eq(sideset_mask),
                ]
            with m.Case(0b11): # sideset = PINDIRS, outset = PINDIRS
                m.d.comb += [
                    self.out.pindirs_data.eq((outset_data & ~sideset_mask) | sideset_data),
                    self.out.pindirs_mask.eq(outset_mask | sideset_mask),
                ]

        return m

class InputMapping(Elaboratable):
    """
    Input GPIO mapping.

    Attributes
    ----------
    raw_gpio : Signal(32), in
    in_base : Signal(5), in
    mapped_gpio : Signal(32), out
    """
    def __init__(self):
        self.in_data = Signal(32)
        self.in_base = Signal(5)
        self.mapped_data = Signal(32)


    def elaborate(self, platform):
        m = Module()

        m.d.comb == self.mapped_data.eq(rotate_right(self.in_data, self.in_base))

        return m

class GpioMapping(Elaboratable):
    """Do priority select of mapped GPIO outputs from state machines and fanout GPIO inputs

    Attributes
    ----------
    mapped_outputs : list(MappedOutput), in
    gpio_in : Signal(32), out
    """
    def __init__(self, sm_count: int, pins: Iterable[Pin]):
        self.mapped_outputs = [MappedOutput() for _ in range(sm_count)]

        self.gpio_in = Signal(32)

        self._pins = list(pins)
        assert len(self._pins) <= 32
        for pin in self._pins:
            if not (hasattr(pin, 'i') or hasattr(pin, 'o') or hasattr(pin, 'oe')):
                raise TypeError("`pins` must be an iterable of type Pin or a record with 'i', 'o', and 'oe' signals")
        
    def elaborate(self, platform):
        m = Module()

        pin_levels = Signal(32)
        oe = Signal(32)
        pin_levels_reg = Signal(32)
        oe_reg = Signal(32)

        m.d.sync += [
            pin_levels_reg.eq(pin_levels),
            oe_reg.eq(oe),
        ]

        for i, pin in enumerate(self._pins):
            m.d.comb += [
                self.gpio_in[i].eq(Mux(~oe[i], pin.i, 0)),
                pin.o.eq(pin_levels_reg[i]),
                pin.oe.eq(oe_reg[i]),
            ]

            for mapped_output in self.mapped_outputs:
                with m.If(mapped_output.pins_mask[i]):
                    m.d.comb += [
                        pin_levels[i].eq(mapped_output.pins_data[i]),
                        pin.o.eq(mapped_output.pins_data[i]),
                    ]
                with m.If(mapped_output.pindirs_mask[i]):
                    m.d.comb += [
                        oe[i].eq(mapped_output.pindirs_data[i]),
                        pin.oe.eq(mapped_output.pindirs_data[i]),
                    ]

        return m

