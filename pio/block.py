from amaranth import *
from .pio import Config, Ctrl
from .state_machine import StateMachine
from .irq import Irq

class Block(Elaboratable):
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.sm = [Ctrl(cfg) for _ in range(cfg.state_machines)]

        self.inst_mem = Record([
            ("w_en", 1),
            ("w_addr", 5),
            ("w_data", 16),
        ])
        self.irq = Record([
            ("idx", range(3)),
            ("r_data", 1),
            ("w_stb", 1),
            ("w_data", 1),
        ])

    def elaborate(self, platform):
        m = Module()

        inst_mem = Memory(width=16, depth=self.cfg.inst_mem_size, init=self.cfg.inst_mem_init)

        m.submodules.inst_mem_wrport = inst_mem_wrport = inst_mem.write_port()
        m.d.comb += [
            inst_mem_wrport.en.eq(self.inst_mem.w_en),
            inst_mem_wrport.addr.eq(self.inst_mem.w_addr),
            inst_mem_wrport.data.eq(self.inst_mem.w_data),
        ]

        m.submodules.irq = irq = Irq()
        irqport = irq.port()
        m.d.comb += [
            irqport.idx.eq(self.irq.idx),
            self.irq.r_data.eq(irqport.r_data),
            irqport.w_stb.eq(self.irq.w_stb),
            irqport.w_data.eq(self.irq.w_data),
        ]

        for id, ctrl in enumerate(self.sm):
            m.submodules[f"sm{id}_rdport"] = rdport = inst_mem.read_port(domain="comb")
            m.submodules[f"sm{id}"] = sm = StateMachine(id=id, cfg=self.cfg, irq_port=irq.port(), ctrl=ctrl)

            m.d.comb += [
                rdport.addr.eq(sm.pc),
                sm.inst.eq(rdport.data),
            ]

        return m
