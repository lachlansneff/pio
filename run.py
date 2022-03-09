import pio
from amaranth.sim import Simulator

dut = pio.Block(pio.Config(
    inst_mem_init = [
        # 0b101_00001_010_01_001, # mov !X -> Y [1]
        # 0b101_00001_001_00_010, # mov Y -> X [1]
        0b001_00000_1_10_00000, # wait 1 irq 0 [0]
        0b101_00001_010_01_001, # mov !X -> Y [1]
    ],
))

def bench():
    yield dut.sm[0].exec.wrap_top.eq(1)
    yield dut.sm[0].en.eq(1)

    yield
    yield
    
    yield dut.irq.idx.eq(0)
    yield dut.irq.w_data.eq(1)
    yield dut.irq.w_stb.eq(1)
    yield
    yield dut.irq.w_stb.eq(0)

    for _ in range(12):
        yield

sim = Simulator(dut)
sim.add_clock(1e-6) # 1 Mhz
sim.add_sync_process(bench)

with sim.write_vcd("pio.vcd"):
    sim.run()
