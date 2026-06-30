"""HIL flight scenarios. Each scenario module exposes:

    SCENARIO_NAME : str
    def build(args) -> Iterable[(delay_ms, packet_bytes, kind_label)]:
        ...
    def expectations() -> list[Expectation]   # optional

The runner imports a scenario by name and feeds its packet stream to
the firmware over USB CDC.
"""
