from amaranth import *
from enum import Enum

class Config:
    def __init__(self, *, fifo_depth: int = 4, state_machines: int = 4, inst_mem_init=None):
        self.fifo_depth: int = fifo_depth

        assert (state_machines <= 4 and state_machines > 0), f"There must be between 1 and 4 state machines in a block, not {state_machines}."
        self.state_machines: int = state_machines

        self.inst_mem_init = inst_mem_init

    @property
    def inst_mem_size(self) -> int:
        return 32

    @property
    def addr_width(self) -> int:
        return 5

class Ctrl:
    class ShiftDirection(Enum):
        LEFT = 0
        RIGHT = 1

    def __init__(self, cfg: Config):
        self.clkdiv = Record([
            ("integer", 16),
            ("fraction", 8),
        ])

        self.exec = Record([
            ("side_en", 1),
            ("sideset_pindir", 1), # if 1, side-set data changes pindirs instead of pin values
            ("wrap_top", cfg.addr_width),
            ("wrap_bottom", cfg.addr_width),
            ("jmp_pin", 5),
        ])
        self.exec.wrap_top.reset = cfg.inst_mem_size - 1

        self.shift = Record([
            ("pull_threshold", 5),
            ("push_threshold", 5),
            ("in_shiftdir", self.ShiftDirection),
            ("out_shiftdir", self.ShiftDirection),
        ])

        self.pin = Record([
            ("sideset_count", 3),
            ("sideset_base", 5),
            ("set_base", 5),
            ("set_count", 3),
            ("out_base", 5),
            ("out_count", 5),
            ("in_base", 5),
        ])

        self.en = Signal(1)
