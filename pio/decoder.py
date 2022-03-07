from amaranth import *
from enum import Enum

class Inst(Enum):
    JMP = 0b000
    WAIT = 0b001
    IN = 0b010
    OUT = 0b011
    PUSH_PULL = 0b100
    MOV = 0b101
    IRQ = 0b110
    SET = 0b111

class PushPull(Enum):
    PUSH = 0b0
    PULL = 0b1

class InstDecoder(Elaboratable):
    def __init__(self):
        # In
        self.inst = Signal(16)

        # Out
        self.op = Signal(Inst)
        self.delay = Signal(5)
        
        # Only relevent with `op == Inst.PUSH_PULL`
        self.push_pull = Signal(PushPull)

        self.invalid = Signal()

        self.jmp = Record([
            ("addr", 5),
            ("cond", 3),
        ])
        self.wait = Record([
            ("idx", 5),
            ("src", Wait.Source),
            ("polarity", Wait.Polarity),
        ])
        self.in_ = Record([
            ("bit_count", 5),
            ("src", 3),
        ])
        self.out = Record([
            ("bit_count", 5),
            ("dest", 3),
        ])
        self.push = Record([
            ("block", 1),
            ("if_full", 1),
        ])
        self.pull = Record([
            ("block", 1),
            ("if_empty", 1),
        ])
        self.mov = Record([
            ("src", 3),
            ("op", 2),
            ("dest", 3),
        ])
        self.irq = Record([
            ("idx", 3),
            ("_", 1),
            ("add_id", 1),
            ("wait", 1),
            ("clear", 1),
        ])
        self.set = Record([
            ("data", 5),
            ("dest", 3),
        ])

    
    def elaborate(self, platform):
        m = Module()

        rest = Signal(8)

        m.d.comb += [
            self.op.eq(self.inst.bit_select(13, 3)),
            self.delay.eq(self.inst.bit_select(8, 5)),
            self.push_pull.eq(self.inst[7]),
            rest.eq(self.inst.bit_select(0, 8)),

            self.jmp.eq(rest),
            self.wait.eq(rest),
            self.in_.eq(rest),
            self.out.eq(rest),
            self.push.eq(rest.bit_select(5, 2)),
            self.pull.eq(rest.bit_select(5, 2)),
            self.mov.eq(rest),
            self.irq.eq(rest),
            self.set.eq(rest),
        ]

        with m.If(self.op == Inst.PUSH_PULL):
            m.d.comb += self.invalid.eq(rest[:5] != 0)
        with m.Elif(self.op == Inst.IRQ):
            m.d.comb += self.invalid.eq((rest[7] != 0) | (rest[3] != 0))

        return m

class Wait:
    class Polarity(Enum):
        WAIT_FOR_0 = 0
        WAIT_FOR_1 = 1

    class Source(Enum):
        GPIO = 0b00
        PIN = 0b01
        IRQ = 0b10

    def __init__(self, stall_next):
        self.source = Signal(self.Source)
        self.polarity = Signal(self.Polarity)
        self.irq_idx = Signal(3)
        self.stall_next = stall_next

    def for_irq(self, m: Module, irq_idx, polarity):
        m.d.sync += [
            self.source.eq(Wait.Source.IRQ),
            self.polarity.eq(polarity),
            self.irq_idx.eq(irq_idx),
        ]
        m.d.comb += self.stall_next.eq(1)
        m.next = "WAIT"
