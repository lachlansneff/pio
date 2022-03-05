import math
from typing import Type
from amaranth import *
from amaranth.lib.fifo import SyncFIFO
from enum import Enum
from clock_en_divider import ClockEnableDivider
import piodisasm

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


class Scratch:
    def __init__(self):
        self.x = Signal(32)
        self.y = Signal(32)

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
            ("wrap_top", cfg.addr_width),
            ("wrap_bottom", cfg.addr_width),
        ])
        self.exec.wrap_top.reset = cfg.inst_mem_size - 1

        self.shift = Record([
            ("pull_threshold", 5),
            ("push_threshold", 5),
            ("in_shiftdir", self.ShiftDirection),
            ("out_shiftdir", self.ShiftDirection),
        ])

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
        self.inst = Signal(16, decoder=lambda inst: piodisasm.parse_to_str(inst, 0, 0))

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

class ErrorReason(Enum):
    NONE = 0
    RESERVED = 1
    INVALID_INST = 1
    BLOCKED_PUSH = 2

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

def saturating_add(x, y, limit):
    added = x + y
    return Mux(added[-1], limit, added[:-1])

class PioStateMachine(Elaboratable):
    """
    
    Attributes
    ----------
    en : Signal, in
        Enable the state machine
    addr_width : int
        The width of addresses, in bits
    pc : Signal(addr_width), out
        The current program counter
    wrap_top : Signal(addr_width, reset = 31), in
        When the `pc` hits this value, it is wrapped around to `wrap_bottom`.
    wrap_bottom : Signal(addr_width, reset = 0), in
    error_reason : Signal(ErrorReason), out

    """
    def __init__(self, *, id: int, cfg: Config):
        self.cfg = cfg
        self.id = id

        # The FIFO interfaces
        self.tx_fifo = Record([
            ("w_rdy", 1),
            ("w_en", 1),
            ("w_data", 32),
        ])
        self.rx_fifo = Record([
            ("r_rdy", 1),
            ("r_en", 1),
            ("r_data", 32),
        ])

        self.en = Signal()
        
        self.pc = Signal(cfg.addr_width)
        self.inst = Signal(16)

        self.ctrl = Ctrl(cfg)
        self.irq = Signal(8)

        self.error = Signal()
        self.error_reason = Signal(ErrorReason)

    def increment_pc(self, m: Module):
        with m.If(self.pc == self.ctrl.exec.wrap_top):
            m.d.sync += self.pc.eq(self.ctrl.exec.wrap_bottom)
        with m.Else():
            m.d.sync += self.pc.eq(self.pc + 1)

    def elaborate(self, platform):
        m = Module()

        m.submodules.clkdiv = clkdiv = ClockEnableDivider(16, 8)
        m.submodules.decoder = decoder = InstDecoder()

        # State
        scratch = Scratch()

        # Output Shift Register
        osr = Signal(32)
        osr_count = Signal(range(32))
        # Input Shift Register
        isr = Signal(32)
        isr_count = Signal(range(32))

        # FIFOs
        tx_fifo = SyncFIFO(width=32, depth=self.cfg.fifo_depth)
        rx_fifo = SyncFIFO(width=32, depth=self.cfg.fifo_depth)

        delay_counter = Signal(5)

        # Stall on the current instruction.
        # This can either run the current instruction again on the next clock cycle
        # or be used to tell the state machine not to advance the PC this cycle.
        stall_next = Signal()
        wait = Wait(stall_next)

        m.d.comb += [
            decoder.inst.eq(self.inst),

            clkdiv.en.eq(self.en),
            
            # Hook TX FIFO
            tx_fifo.w_rdy.eq(self.tx_fifo.w_rdy),
            tx_fifo.w_en.eq(self.tx_fifo.w_en),
            tx_fifo.w_data.eq(self.tx_fifo.w_data),

            # Hook up RX FIFO
            rx_fifo.r_rdy.eq(self.rx_fifo.r_rdy),
            rx_fifo.r_en.eq(self.rx_fifo.r_en),
            rx_fifo.r_data.eq(self.rx_fifo.r_data),
        ]
        
        with m.If(clkdiv.clk_en):
            with m.FSM() as fsm:
                # If the decoder says the instruction is invalid, pause the state-machine.                
                with m.State("EXEC"):
                    with m.If(decoder.invalid):
                        m.d.sync += self.error_reason.eq(ErrorReason.INVALID_INST)
                        m.next = "ERROR"

                    with m.If(~stall_next):
                        self.increment_pc(m)

                    with m.If(decoder.delay != 0):
                        m.d.sync += delay_counter.eq(decoder.delay)
                        m.d.comb += stall_next.eq(1)
                        m.next = "DELAY"

                    with m.If(decoder.op == Inst.JMP):
                        vars = decoder.jmp
                        do_jmp = Signal()

                        with m.Switch(vars.cond):
                            with m.Case("000"):
                                # We always jmp
                                m.d.comb += do_jmp.eq(1)
                            with m.Case("001"):
                                # Scratch X is zero
                                m.d.comb += do_jmp.eq(scratch.x == 0)
                            with m.Case("010"):
                                # Scratch X is non-zero and then scratch.x is decremented
                                m.d.comb += do_jmp.eq(scratch.x != 0)
                                m.d.sync += scratch.x.eq(scratch.x - 1)
                            with m.Case("011"):
                                # Scratch Y is zero
                                m.d.comb += do_jmp.eq(scratch.y == 0)
                            with m.Case("100"):
                                # Scratch Y is non-zero and then scratch.y is decremented
                                m.d.comb += do_jmp.eq(scratch.y != 0)
                                m.d.sync += scratch.y.eq(scratch.y - 1)
                            with m.Case("101"):
                                # X != Y
                                m.d.comb += do_jmp.eq(scratch.x != scratch.y)
                            with m.Case("110"):
                                # branch on input pin
                                m.next = "NOT_IMPLEMENTED"
                            with m.Case("111"):
                                # output shift register is not empty
                                m.d.comb += do_jmp.eq(osr_count != 0)

                        with m.If(do_jmp):
                            m.d.sync += self.pc.eq(vars.addr)
                        
                    with m.Elif(decoder.op == Inst.WAIT):
                        vars = decoder.wait
                        with m.Switch(vars.src):
                            with m.Case(Wait.Source.GPIO):
                                m.next = "NOT_IMPLEMENTED"
                            with m.Case(Wait.Source.PIN):
                                m.next = "NOT_IMPLEMENTED"
                            with m.Case(Wait.Source.IRQ):
                                # Decode the IRQ index
                                irq_idx = Mux(vars.idx[4], vars.idx[:3] + self.id, vars.idx[:3])
                                wait.for_irq(m, irq_idx, vars.polarity)
                            with m.Case():
                                m.next = "ERROR"
                                m.d.sync += self.error_reason.eq(ErrorReason.RESERVED)

                    with m.Elif(decoder.op == Inst.IN):
                        vars = decoder.in_
                        src_data = Signal(32)

                        with m.Switch(vars.src):
                            with m.Case("000"):
                                # From PINS
                                m.next = "NOT_IMPLEMENTED"
                            with m.Case("001"):
                                # From Scratch X
                                m.d.comb += src_data.eq(scratch.x)
                            with m.Case("010"):
                                # Scratch Y
                                m.d.comb += src_data.eq(scratch.y)
                            with m.Case("011"):
                                m.d.comb += src_data.eq(0)
                            with m.Case("10-"):
                                # Reserved
                                m.next = "ERROR"
                                m.d.sync += self.error_reason.eq(ErrorReason.RESERVED)
                            with m.Case("110"):
                                # ISR
                                m.d.comb += src_data.eq(isr)
                            with m.Case("111"):
                                # OSR
                                m.d.comb += src_data.eq(osr)
                        
                        bit_count = Mux(vars.bit_count == 0, 32, vars.bit_count)
                        m.d.sync += isr_count.eq(saturating_add(isr_count, bit_count, 32))

                        with m.If(self.ctrl.shift.in_shiftdir == Ctrl.ShiftDirection.LEFT):
                            shifted_src_data = src_data << (32 - bit_count)
                            m.d.sync += isr.eq((Cat(shifted_src_data, isr) << bit_count)[32:64])
                        with m.Else(): # Shift right
                            m.d.sync += isr.eq((Cat(isr, src_data) >> bit_count)[0:32])
                        
                    with m.Elif(decoder.op == Inst.OUT):
                        m.next = "NOT_IMPLEMENTED"
                        vars = decoder.out
                        shifted_data = Signal(32)
                        shifted_osr = Signal(32)

                        bit_count = Mux(vars.bit_count == 0, 32, vars.bit_count)
                        m.d.sync += osr_count.eq(saturating_add(osr_count, bit_count, 32))

                        with m.If(self.ctrl.shift.out_shiftdir == Ctrl.ShiftDirection.LEFT):
                            m.d.comb += Cat(shifted_osr, shifted_data).eq(osr << bit_count)
                        with m.Else(): # Shift Right
                            temp_shift = Signal(32)
                            m.d.comb += [
                                Cat(temp_shift, shifted_osr).eq(Cat(osr, C(0, 32)) >> bit_count),
                                shifted_data.eq(temp_shift >> (1 - bit_count)),
                            ]
                        
                        m.d.sync += osr.eq(shifted_osr)

                        with m.Switch(vars.dest):
                            with m.Case("000"):
                                # PINS
                                m.next = "NOT_IMPLEMENTED"
                            with m.Case("001"):
                                # Scratch X
                                m.d.sync += scratch.x.eq(shifted_data)
                            with m.Case("010"):
                                # Scratch Y
                                m.d.sync += scratch.y.eq(shifted_data)
                            with m.Case("011"):
                                # NULL (discard data)
                                pass
                            with m.Case("100"):
                                # PINDIRS
                                m.next = "NOT_IMPLEMENTED"
                            with m.Case("101"):
                                # PC
                                m.d.sync += self.pc.eq(shifted_data)
                            with m.Case("110"):
                                # ISR
                                m.d.sync += [
                                    isr.eq(shifted_data),
                                    isr_count.eq(bit_count),
                                ]
                            with m.Case("111"):
                                # EXEC
                                m.next = "NOT_IMPLEMENTED"
                    
                    # PUSH
                    with m.Elif((decoder.op == Inst.PUSH_PULL) & (decoder.push_pull == PushPull.PUSH)):
                        vars = decoder.push

                        # If the RX FIFO is full and `block` is set, block the state machine
                        with m.If(vars.block & ~rx_fifo.w_rdy):
                            # Stall the state machine until the next clock cycle
                            # This will get run again then, and will continue
                            # to stall until the RX FIFO is not full
                            m.d.comb += stall_next.eq(1)
                        with m.Else():
                            with m.If(vars.if_full & (isr_count < self.ctrl.shift.push_threshold)):
                                pass
                            # If there's space in the RX FIFO
                            with m.Elif(rx_fifo.w_rdy):
                                m.d.comb += [
                                    # Write the data
                                    rx_fifo.w_data.eq(isr),
                                    rx_fifo.w_en.eq(1),
                                ]
                                m.d.sync += [
                                    # Clear the ISR
                                    isr.eq(0),
                                    isr_count.eq(0),
                                ]

                    # PULL
                    with m.Elif((decoder.op == Inst.PUSH_PULL) & (decoder.push_pull == PushPull.PULL)): # PULL
                        vars = decoder.pull

                        with m.If(vars.block & ~tx_fifo.r_rdy):
                            # Stall the state machine until the next clock cycle
                            # This will get run again then, and will continue
                            # to stall until the TX FIFO is not full
                            m.d.comb += stall_next.eq(1)
                        with m.Else():
                            with m.If(vars.if_empty & (osr_count < self.ctrl.shift.pull_threshold)):
                                pass
                            with m.Else():
                                # Fill OSR
                                m.d.sync += osr_count.eq(32)

                                # If the TX FIFO is not empty, read it
                                with m.If(tx_fifo.r_rdy):
                                    m.d.sync += osr.eq(tx_fifo.r_data)
                                    m.d.comb += tx_fifo.r_en.eq(1)
                                # Otherwise, copy scratch X into the OSR
                                with m.Else():
                                    m.d.sync += osr.eq(scratch.x)

                    with m.Elif(decoder.op == Inst.MOV):
                        vars = decoder.mov

                        modified_src_data = Signal(32)
                        src_data = Signal(32)

                        with m.Switch(vars.src):
                            with m.Case("000"):
                                # PINS
                                m.next = "NOT_IMPLEMENTED"
                            with m.Case("001"):
                                # Load from Scratch X
                                m.d.comb += src_data.eq(scratch.x)
                            with m.Case("010"):
                                # Load from Scratch Y
                                m.d.comb += src_data.eq(scratch.y)
                            with m.Case("011"):
                                # Load all zeros
                                m.d.comb += src_data.eq(0)
                            with m.Case("100"):
                                # Reserved
                                m.d.sync += self.error_reason.eq(ErrorReason.RESERVED)
                                m.next = "ERROR"
                            with m.Case("101"):
                                # Load from STATUS
                                m.next = "NOT_IMPLEMENTED"
                            with m.Case("110"):
                                # Input shift register
                                m.d.comb += src_data.eq(isr)
                            with m.Case("111"):
                                # Output shift register
                                m.d.comb += src_data.eq(osr)
                        
                        with m.Switch(vars.op):
                            with m.Case("00"):
                                # None
                                m.d.comb += modified_src_data.eq(src_data)
                            with m.Case("01"):
                                # Invert
                                m.d.comb += modified_src_data.eq(~src_data)
                            with m.Case("10"):
                                # Bit-reversed
                                m.d.comb += modified_src_data.eq(src_data[::-1])
                            with m.Case("11"):
                                # Reserved
                                m.d.sync += self.error_reason.eq(ErrorReason.RESERVED)
                                m.next = "ERROR"
                        
                        with m.Switch(vars.dest):
                            with m.Case("000"):
                                # PINS
                                m.next = "NOT_IMPLEMENTED"
                            with m.Case("001"):
                                # Scratch X
                                m.d.sync += scratch.x.eq(modified_src_data)
                            with m.Case("010"):
                                # Scratch Y
                                m.d.sync += scratch.y.eq(modified_src_data)
                            with m.Case("011"):
                                # Reserved
                                m.d.sync += self.error_reason.eq(ErrorReason.RESERVED)
                                m.next = "ERROR"
                            with m.Case("100"):
                                # EXEC
                                m.next = "NOT_IMPLEMENTED"
                            with m.Case("101"):
                                # PC
                                m.d.sync += self.pc.eq(modified_src_data)
                            with m.Case("110"):
                                # Input shift register
                                m.d.sync += [
                                    isr.eq(modified_src_data),
                                    isr_count.eq(0), # Set to full
                                ]
                            with m.Case("111"):
                                # Output shift register
                                m.d.sync += [
                                    osr.eq(modified_src_data),
                                    osr_count.eq(0), # Set to full
                                ]
                    with m.Elif(decoder.op == Inst.IRQ):
                        m.next = "NOT_IMPLEMENTED"

                        vars = decoder.irq
                        irq_idx = Mux(vars.add_id, vars.idx + self.id, vars.idx)
                        
                        with m.If(vars.clear):
                            # Clear the selected IRQ bit
                            m.d.sync += self.irq.bit_select(irq_idx, 1).eq(0)
                        with m.Else():
                            # Set the selected IRQ bit
                            m.d.sync += self.irq.bit_select(irq_idx, 1).eq(1)
                            wait.for_irq(m, irq_idx, Wait.Polarity.WAIT_FOR_0)

                    with m.Elif(decoder.op == Inst.SET):
                        vars = decoder.set

                        with m.Switch(vars.dest):
                            with m.Case("000"):
                                # PINS
                                m.next = "NOT_IMPLEMENTED"
                            with m.Case("001"):
                                # Scratch X
                                m.d.sync += scratch.x.eq(vars.data)
                            with m.Case("010"):
                                # Scratch Y
                                m.d.sync += scratch.y.eq(vars.data)
                            with m.Case("100"):
                                # PINDIRS
                                m.next = "NOT_IMPLEMENTED"
                            with m.Case():
                                # Reserved
                                m.d.sync += self.error_reason.eq(ErrorReason.RESERVED)
                                m.next = "ERROR"

                with m.State("DELAY"):
                    m.d.sync += delay_counter.eq(delay_counter - 1)
                    with m.If(delay_counter == 1):
                        self.increment_pc(m)
                        m.next = "EXEC"
                
                # Wait for a selected IRQ bit to become zero
                with m.State("WAIT"):
                    wait_over = Signal()
                    with m.Switch(wait.source):
                        with m.Case(Wait.Source.IRQ):
                            with m.If(self.irq.bit_select(wait.irq_idx, 1) == wait.polarity):
                                # The bit has changed!
                                m.d.comb += wait_over.eq(1)

                                # Clear the IRQ bit if we're waiting for 1.
                                with m.If(wait.polarity == Wait.Polarity.WAIT_FOR_1):
                                    m.d.sync += self.irq.bit_select(wait.irq_idx, 1).eq(0)

                    with m.If(wait_over):
                        with m.If(decoder.delay != 0):
                            # Start the delay after the wait period has finished
                            m.d.sync += delay_counter.eq(decoder.delay)
                            m.next = "DELAY"
                        with m.Else():
                            self.increment_pc(m)
                            m.next = "EXEC"

                m.d.comb += self.error.eq(fsm.ongoing("ERROR") | fsm.ongoing("NOT_IMPLEMENTED"))
                with m.State("ERROR"):
                    pass
                with m.State("NOT_IMPLEMENTED"):
                    pass
    
        return m


class PioBlock(Elaboratable):
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.sm = [PioStateMachine(id=id, cfg=cfg) for id in range(cfg.state_machines)]

        self.inst_mem = Record([
            ("w_en", 1),
            ("w_addr", 5),
            ("w_data", 16),
        ])

        self.irq = Signal(4)

    def elaborate(self, platform):
        m = Module()

        inst_mem = Memory(width=16, depth=self.cfg.inst_mem_size, init=self.cfg.inst_mem_init)

        full_irq = Signal(8)

        m.d.comb += [
            self.irq.eq(full_irq[0:4]),
        ]

        m.submodules.inst_mem_wrport = inst_mem_wrport = inst_mem.write_port()
        m.d.comb += [
            inst_mem_wrport.en.eq(self.inst_mem.w_en),
            inst_mem_wrport.addr.eq(self.inst_mem.w_addr),
            inst_mem_wrport.data.eq(self.inst_mem.w_data),
        ]

        for i, sm in enumerate(self.sm):
            m.submodules[f"sm{i}_rdport"] = rdport = inst_mem.read_port(domain="comb")
            m.submodules["sm{}".format(i)] = sm
            m.d.comb += [
                full_irq.eq(sm.irq),
                rdport.addr.eq(sm.pc),
                sm.inst.eq(rdport.data),
            ]

        return m

if __name__ == "__main__":
    from amaranth.sim import Simulator

    dut = PioBlock(Config(
        state_machines=1,
        inst_mem_init = [
            0b101_00001_010_01_001, # mov !X -> Y [1]
            0b101_00001_001_00_010, # mov Y -> X [1]
        ],
    ))
    
    def bench():
        yield dut.sm[0].ctrl.exec.wrap_top.eq(1)
        yield dut.sm[0].en.eq(1)

        for _ in range(12):
            yield

    sim = Simulator(dut)
    sim.add_clock(1e-6) # 1 Mhz
    sim.add_sync_process(bench)

    with sim.write_vcd("pio.vcd"):
        sim.run()

