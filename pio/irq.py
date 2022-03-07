from amaranth import *

class Irq(Elaboratable):
    def __init__(self):
        self._buses = []

    def port(self) -> Record:
        irq_bus = Record([
            ("idx", range(8)),
            ("r_data", 1),
            ("w_stb", 1),
            ("w_data", 1),
        ])

        self._buses.append(irq_bus)
        return irq_bus

    def elaborate(self, platform):
        m = Module()

        data = Signal(8)

        # Reading
        for idx in range(8):
            for bus in self._buses:
                with m.If(bus.idx == idx):
                    m.d.comb += bus.r_data.eq(data[idx])

        for idx in range(8):
            per_idx_w_stb = Signal()
            per_idx_data = Signal()
            first = True
            for bus in self._buses:
                if first:
                    first = False
                    with m.If(bus.w_stb & (idx == bus.idx)):
                        m.d.comb += [
                            per_idx_w_stb.eq(1),
                            per_idx_data.eq(bus.w_data),
                        ]
                else:
                    with m.Elif(bus.w_stb & (idx == bus.idx)):
                        m.d.comb += [
                            per_idx_w_stb.eq(1),
                            per_idx_data.eq(bus.w_data),
                        ]

            with m.If(per_idx_w_stb):
                m.d.sync += data[idx].eq(per_idx_data)
        
        return m
