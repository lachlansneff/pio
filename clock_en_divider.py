from amaranth import *

class ClockEnableDivider(Elaboratable):
    """A fixed-point clock enable divider.

    Parameters
    ----------
    integer_width : int
        The width, in bits, of the integer part of the number.
    fraction_width : int
        The width, in bits, of the fractional part of the number.

    Attributes
    ----------
    integer_width : int
    fraction_width : int
    integer : Signal(integer_width), in
        The integer part of the clock divisor.
    fraction : Signal(fraction_width), in
        The fraction part of the clock divisor.
    en : Signal(), in
        Enable/disable clock divider.
    clk_en: Signal(), out
        Asserted when the attached block should be enabled.
    """
    def __init__(self, integer_width: int, fraction_width: int):
        if integer_width <= 0 or integer_width <= 0:
            raise TypeError("Bit width of the integer and fractional parts of the fixed-point number must be positive integers")
        
        self.integer_width = integer_width
        self.fraction_width = fraction_width

        self.integer = Signal(integer_width, reset = 1)
        self.fraction = Signal(fraction_width)

        self.clk_en = Signal()
        self.en = Signal()

    def elaborate(self, platform):
        m = Module()

        # int_divisor = Signal(self.integer_width)
        # frac_divisor = Signal(self.fraction_width)
        counter = Signal(self.integer_width + self.fraction_width + 1)

        m.d.sync += [
            counter.eq(counter + (2**self.fraction_width) - Mux(counter >= Cat(self.fraction, self.integer), Cat(self.fraction, self.integer), 0)),
            self.clk_en.eq(counter >= Cat(self.fraction, self.integer)),
        ]

        return m


if __name__ == "__main__":
    from amaranth.sim import Simulator

    dut = ClockEnableDivider(16, 8)

    def bench():
        # Set divisor to 1.0
        yield dut.integer.eq(1)
        yield dut.fraction.eq(0)

        yield dut.en.eq(1)

        for _ in range(12):
            yield

        yield dut.en.eq(0)
        yield

        # Set divisor to 2.0
        yield dut.integer.eq(2)
        yield dut.fraction.eq(0)
        yield dut.en.eq(1)

        for _ in range(12):
            yield

        yield dut.en.eq(0)
        yield

        # Set divisor to 2.5
        yield dut.integer.eq(2)
        yield dut.fraction.eq(((2**8) - 1) // 2)
        yield dut.en.eq(1)

        for _ in range(12):
            yield

    sim = Simulator(dut)
    sim.add_clock(1e-6) # 1 Mhz
    sim.add_sync_process(bench)
    with sim.write_vcd("clock_en_divider.vcd"):
        sim.run()