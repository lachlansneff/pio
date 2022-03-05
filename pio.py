from amaranth import *
from amaranth.lib.fifo import SyncFIFO
from enum import Enum, auto
from clock_en_divider import ClockEnableDivider
import piodisasm

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
        self.exec.wrap_top.reset = 2**addr_width - 1

        self.shift = Record([
            ("pull_threshold", 5),
            ("push_threshold", 5),
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
        self.delay_or_sideset = Signal(5)
        
        # Only relevent with `op == Inst.PUSH_PULL`
        self.push_pull = Signal(PushPull)
        
        self.rest = Signal(8)

        self.invalid = Signal()

    
    def elaborate(self, platform):
        m = Module()

        m.d.comb += [
            self.op.eq(self.inst.bit_select(13, 3)),
            self.delay_or_sideset.eq(self.inst.bit_select(8, 5)),
            self.push_pull.eq(self.inst[7]),
            self.rest.eq(self.inst.bit_select(0, 8)),
        ]

        with m.If(self.op == Inst.PUSH_PULL):
            m.d.comb += self.invalid.eq(self.rest[:5] == 0)
        with m.Elif(self.op == Inst.IRQ):
            m.d.comb += self.invalid.eq(self.rest[7] == 0)
        with m.Else():
            m.d.comb += self.invalid.eq(0)

        return m

class ErrorReason(Enum):
    NONE = auto()
    RESERVED = auto()
    INVALID_INST = auto()
    BLOCKED_PUSH = auto()

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
    def __init__(self, addr_width: int, inst_read_port):
        self.addr_width = addr_width

        # The FIFO interfaces
        self.tx_fifo = Record([
            ("w_rdy", 1),
            ("w_en", 1),
            ("w_data", 32),
        ])
        self.rx_fifi = Record([
            ("r_rdy", 1),
            ("r_en", 1),
            ("r_data", 32),
        ])

        self.en = Signal()
        self.pc = Signal(addr_width)

        assert (inst_read_port.domain == "comb")
        self._inst_read_port = inst_read_port

        self.ctl = Ctrl(addr_width)

        self.error = Signal()
        self.error_reason = Signal(ErrorReason)

    def increment_pc(self, m: Module):
        with m.If(self.pc == self.wrap_top):
            m.d.sync += self.pc.eq(self.wrap_bottom)
        with m.Else():
            m.d.sync += self.pc.eq(self.pc + 1)

    def elaborate(self, platform):
        m = Module()

        m.submodules.clkdiv = clkdiv = ClockEnableDivider(16, 8)
        m.submodules.decoder = decoder = InstDecoder()
        m.submodules.inst_rdport = inst_rdport = self._inst_read_port

        # State
        scratch = Scratch()

        # Output Shift Register
        osr = Signal(32)
        osr_count = Signal(range(32))
        # Input Shift Register
        isr = Signal(32)
        isr_count = Signal(range(32))

        # FIFOs
        tx_fifo = SyncFIFO(width=32, depth=4)
        rx_fifo = SyncFIFO(width=32, depth=4)

        delay_counter = Signal(5)

        m.d.comb += [
            inst_rdport.addr.eq(self.pc),
            decoder.inst.eq(inst_rdport.data),

            clkdiv.en.eq(self.en),
            
            # Hook TX FIFO
            tx_fifo.w_rdy.eq(self.tx_fifo.w_rdy),
            tx_fifo.w_en.eq(self.tx_fifo.w_en),
            tx_fifo.w_data.eq(self.tx_fifo.w_data),

            # Hook up RX FIFO
            rx_fifo.r_rdy.eq(self.rx_fifo.r_rdy),
            rx_fifo.r_en.eq(self.rx_fifo.r_en),
            rx_fifo.r_data.eq(self.rx_fifi.r_data),
        ]
        
        with m.If(clkdiv.clk_en):
            with m.FSM() as fsm:
                # If the decoder says the instruction is invalid, pause the state-machine.                
                with m.State("EXEC"):
                    with m.If(decoder.invalid):
                        m.d.sync += self.error_reason.eq(ErrorReason.INVALID_INST)
                        m.next = "ERROR"

                    with m.If(decoder.delay_or_sideset != 0):
                        m.d.sync += delay_counter.eq(decoder.delay_or_sideset)
                        m.next = "DELAY"
                    with m.Else():
                        self.increment_pc(m)

                    with m.If(decoder.op == Inst.JMP):
                        jmp_addr = decoder.rest.bit_select(0, 5)
                        condition = decoder.rest.bit_select(5, 3)
                        
                        do_jmp = Signal()

                        with m.Switch(condition):
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
                            m.d.sync += self.pc.eq(jmp_addr)
                        
                    with m.Elif(decoder.op == Inst.WAIT):
                        m.next = "NOT_IMPLEMENTED"
                    with m.Elif(decoder.op == Inst.IN):
                        # bit_count = decoder.rest.bit_select(0, 5)
                        # source = decoder.rest.bit_select(5, 3)

                        # with m.Switch(source):
                        #     with m.Case("000"):
                        #         # From PINS
                        #         m.next = "NOT_IMPLEMENTED"
                        #     with m.Case("001"):
                        #         # From scratch.x
                                
                        m.next = "NOT_IMPLEMENTED"
                    with m.Elif(decoder.op == Inst.OUT):
                        m.next = "NOT_IMPLEMENTED"
                    
                    # PUSH
                    with m.Elif((decoder.op == Inst.PUSH_PULL) & (decoder.push_pull == PushPull.PUSH)):
                        block = decoder.rest.bit_select(5, 1)
                        if_full = decoder.rest.bit_select(6, 1)

                        # If the RX FIFO is full and `block` is set, block the state machine
                        with m.If(block & ~rx_fifo.w_rdy):
                            pass
                        with m.Else():
                            with m.If(if_full & (isr_count < self.ctrl.shift.push_threshold)):
                                pass
                            # If there's space in the RX FIFO
                            with m.Elif(rx_fifo.w_rdy):
                                m.d.sync += [
                                    # Write the data
                                    rx_fifo.w_data.eq(isr),
                                    rx_fifo.w_en.eq(1),
                                    # Clear the ISR
                                    isr.eq(0),
                                    isr_count.eq(0),
                                ]

                    # PULL
                    with m.Elif((decoder.op == Inst.PUSH_PULL) & (decoder.push_pull == PushPull.PULL)): # PULL
                        m.next = "NOT_IMPLEMENTED"
                    with m.Elif(decoder.op == Inst.MOV):
                        src = decoder.rest.bit_select(0, 3)
                        op = decoder.rest.bit_select(3, 2)
                        dest = decoder.rest.bit_select(5, 3)

                        modified_src_data = Signal(32)
                        src_data = Signal(32)

                        with m.Switch(src):
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
                        
                        with m.Switch(op):
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
                        
                        with m.Switch(dest):
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
                    with m.Elif(decoder.op == Inst.SET):
                        dest = decoder.rest.bit_select(0, 3)
                        imm = decoder.rest.bit_select(3, 5)

                        with m.Switch(dest):
                            with m.Case("000"):
                                # PINS
                                m.next = "NOT_IMPLEMENTED"
                            with m.Case("001"):
                                # Scratch X
                                m.d.sync += scratch.x.eq(imm)
                            with m.Case("010"):
                                # Scratch Y
                                m.d.sync += scratch.y.eq(imm)
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

                m.d.comb += self.error.eq(fsm.ongoing("ERROR") | fsm.ongoing("NOT_IMPLEMENTED"))
                with m.State("ERROR"):
                    pass
                with m.State("NOT_IMPLEMENTED"):
                    pass
    
        return m


class PioBlock(Elaboratable):
    def __init__(self, state_machine_count: int, addr_width: int):
        self.state_machine_count = state_machine_count
        self.addr_width = addr_width
        self.inst_mem = Memory(width=16, depth=2**self.addr_width)

        self.sm = [PioStateMachine(self.addr_width, self.inst_mem.read_port(domain="comb")) for _ in range(state_machine_count)]

    def elaborate(self, platform):
        m = Module()

        for i in range(self.state_machine_count):
            m.submodules["state_machine{}".format(i)] = self.sm[i]

        return m

if __name__ == "__main__":
    from amaranth.sim import Simulator

    dut = PioBlock(1, 5)
    dut.inst_mem.init = [
        0b101_00001_010_01_001, # mov !X -> Y [1]
        0b101_00001_001_00_010, # mov Y -> X [1]
    ]
    
    def bench():
        yield dut.sm[0].ctrl.wrap_top.eq(1)
        yield dut.sm[0].en.eq(1)

        for _ in range(12):
            yield

    sim = Simulator(dut)
    sim.add_clock(1e-6) # 1 Mhz
    sim.add_sync_process(bench)

    with sim.write_vcd("pio.vcd"):
        sim.run()

