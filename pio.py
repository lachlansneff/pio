from amaranth import *
from clock_en_divider import ClockEnableDivider

class Scratch:
    def __init__(self):
        self.x = Signal(32)
        self.y = Signal(32)

class Ctrl:
    def __init__(self, addr_width: int):
        self.clkdiv = Record([
            ("integer", 16),
            ("fraction", 8),
        ])

        self.exec = Record([
            ("wrap_top", addr_width),
            ("wrap_bottom", addr_width),
        ])


class PioStateMachine(Elaboratable):
    """
    
    Attributes
    ----------
    addr_width : int
        The width of addresses, in bits
    pc : Signal(addr_width), out
        The current program counter
    inst : Signal(16), in
        The instruction at `pc`
    wrap_top : Signal(addr_width, reset = 31), in
        When the `pc` hits this value, it is wrapped around to `wrap_bottom`.
    wrap_bottom : Signal(addr_width, reset = 0), in

    """
    def __init__(self, addr_width: int):
        self.addr_width = addr_width

        # The FIFO interfaces
        self.tx_fifo = Record([
            ("en", 1),
            ("data", 8),
        ])
        self.rx_fifi = Record([
            ("rdy", 1),
            ("data", 8),
        ])

        # Used 
        self.pc = Signal(addr_width)
        self.inst = Signal(16)

        self.wrap_top = Signal(addr_width, reset = 31)
        self.wrap_bottom = Signal(addr_width, reset = 0)
        

    def elaborate(self, platform):
        m = Module()

        # State
        self.scratch = Scratch()

        

        # Output Shift Register
        self.osr = Signal(32)
        # Input Shift Register
        self.isr = Signal(32)


