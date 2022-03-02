from amaranth import *
from enum import Enum
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
        self.delay_or_sideset = Signal(5)
        
        # Only relevent with `op == Inst.PUSH_PULL`
        self.push_pull = Signal(PushPull)
        
        self.rest = Signal(8)

        self.invalid = Signal()

    
    def elaborate(self, platform):
        m = Module()

        be_inst = Signal(16)

        m.d.comb += be_inst.eq(self.inst[::-1])

        m.d.comb += [
            self.op.eq(be_inst[:3]),
            self.delay_or_sideset.eq(be_inst[3:8]),
            self.push_pull.eq(be_inst[8]),
            self.rest.eq(be_inst[8:]),
        ]

        with m.If(self.op == Inst.PUSH_PULL):
            m.d.comb += self.invalid.eq(self.rest[3:] == 0)
        with m.Elif(self.op == Inst.IRQ):
            m.d.comb += self.invalid.eq(self.rest[0] == 0)
        with m.Else():
            m.d.comb += self.invalid.eq(0)

        return m

class PioStateMachine(Elaboratable):
    """
    
    Attributes
    ----------
    en : Signal
        Enable the state machine
    addr_width : int
        The width of addresses, in bits
    pc : Signal(addr_width), out
        The current program counter
    wrap_top : Signal(addr_width, reset = 31), in
        When the `pc` hits this value, it is wrapped around to `wrap_bottom`.
    wrap_bottom : Signal(addr_width, reset = 0), in

    """
    def __init__(self, addr_width: int, inst_read_port):
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

        self.en = Signal()
        self.pc = Signal(addr_width)
        self._inst_read_port = inst_read_port

        self.wrap_top = Signal(addr_width, reset = 31)
        self.wrap_bottom = Signal(addr_width, reset = 0)

        self.error = Signal()
        

    def elaborate(self, platform):
        m = Module()

        m.submodules.clkdiv = clkdiv = ClockEnableDivider(16, 8)
        m.submodules.decoder = decoder = InstDecoder()
        m.submodules.inst_rdport = inst_rdport = self._inst_read_port

        # State
        scratch = Scratch()

        # Output Shift Register
        osr = Signal(32)
        osr_counter = Signal(range(32))
        # Input Shift Register
        isr = Signal(32)
        isr_counter = Signal(range(32))

        delay_counter = Signal(5)

        new_pc = Signal(self.addr_width)

        m.d.comb += [
            inst_rdport.addr.eq(self.pc),
            decoder.inst.eq(inst_rdport.data),
        ]

        with m.If(new_pc == self.wrap_top):
            m.d.sync += self.pc.eq(self.wrap_bottom)
        with m.Else():
            m.d.sync += self.pc.eq(new_pc)
        
        with m.If(self.en):
            with m.FSM() as fsm:
                # If the decoder says the instruction is invalid, pause the state-machine.                
                with m.State("EXEC"):
                    with m.If(decoder.invalid):
                        m.next = "ERROR"
                    m.d.comb += new_pc.eq(self.pc + 1)

                    with m.If(decoder.delay_or_sideset != 0):
                        m.d.sync += delay_counter.eq(decoder.delay_or_sideset)
                        m.next = "DELAY"

                    with m.If(decoder.op == Inst.JMP):
                        condition = decoder.rest.bit_select(0, 3)
                        jmp_addr = decoder.rest.bit_select(3, 5)

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
                                m.d.comb += do_jmp.eq(osr_counter != 0)

                        with m.If(do_jmp):
                            m.d.comb += new_pc.eq(jmp_addr)
                        
                    with m.Elif(decoder.op == Inst.WAIT):
                        m.next = "NOT_IMPLEMENTED"
                    with m.Elif(decoder.op == Inst.IN):
                        # source = decoder.rest.bit_select(0, 3)
                        # bit_count = decoder.rest.bit_select(3, 5)

                        # with m.Switch(source):
                        #     with m.Case("000"):
                        #         # From PINS
                        #         m.next = "NOT_IMPLEMENTED"
                        #     with m.Case("001"):
                        #         # From scratch.x
                        m.next = "NOT_IMPLEMENTED"
                    with m.Elif(decoder.op == Inst.OUT):
                        m.next = "NOT_IMPLEMENTED"
                    with m.Elif((decoder.op == Inst.PUSH_PULL) & (decoder.push_pull == PushPull.PUSH)):
                        m.next = "NOT_IMPLEMENTED"
                    with m.Elif((decoder.op == Inst.PUSH_PULL) & (decoder.push_pull == PushPull.PULL)):
                        m.next = "NOT_IMPLEMENTED"
                    with m.Elif(decoder.op == Inst.MOV):
                        dest = decoder.rest.bit_select(0, 3)
                        op = decoder.rest.bit_select(3, 2)
                        src = decoder.rest.bit_select(5, 3)

                        dest_data = Signal(32)
                        modified_src_data = Signal(32)
                        src_data = Signal(32)

                        with m.Switch(src):
                            with m.Case("000"):
                                # PINS
                                m.next = "NOT_IMPLEMENTED"
                            with m.Case("001"):
                                # Load from Scratch X
                                m.d.comb += dest_data.eq(scratch.x)
                            with m.Case("010"):
                                # Load from Scratch Y
                                m.d.comb += dest_data.eq(scratch.y)
                            with m.Case("011"):
                                # Load all zeros
                                m.d.comb += dest_data.eq(0)
                            with m.Case("100"):
                                # Reserved
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
                                m.next = "ERROR"
                            with m.Case("100"):
                                # EXEC
                                m.next = "NOT_IMPLEMENTED"
                            with m.Case("101"):
                                # PC
                                m.d.comb += new_pc.eq(modified_src_data)
                            with m.Case("110"):
                                # Input shift register
                                m.d.sync += [
                                    isr.eq(modified_src_data),
                                    isr_counter.eq(0), # Set to full
                                ]
                            with m.Case("111"):
                                # Output shift register
                                m.d.sync += [
                                    osr.eq(modified_src_data),
                                    osr_counter.eq(0), # Set to full
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
                                m.next = "ERROR"

                with m.State("DELAY"):
                    m.d.sync += delay_counter.eq(delay_counter - 1)
                    with m.If(delay_counter == 1):
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

    def elaborate(self, platform):
        m = Module()

        inst_mem = Memory(width=16, depth=2**self.addr_width)

        state_machines = []
        for i in range(self.state_machine_count):
            state_machines.append(PioStateMachine(self.addr_width, inst_mem.read_port()))
            m.submodules["state_machine{}".format(i)] = state_machines[i]
        
        for sm in state_machines:
            m.d.comb += sm.en.eq(1)

        return m

if __name__ == "__main__":
    from amaranth.sim import Simulator

    dut = PioBlock(1, 5)
    def bench():
        for _ in range(12):
            yield

    sim = Simulator(dut)
    sim.add_clock(1e-6) # 1 Mhz
    sim.add_sync_process(bench)

    with sim.write_vcd("pio.vcd"):
        sim.run()

