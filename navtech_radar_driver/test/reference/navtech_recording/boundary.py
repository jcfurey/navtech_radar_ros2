"""Recognize a revolution boundary hidden by missing Colossus FFT packets."""


def counter_crossed_zero(previous, current, advance, count, encoder_size, rate, gap_ns):
    if advance == 0 or (current >= previous and advance < count):
        return False
    ticks = encoder_size + current - previous
    # Half a nominal cell is ambiguous. Elapsed time, when known, must also
    # fit the forward path through encoder zero (same rule as the C++ assembler).
    return (2 * abs(ticks * count - advance * encoder_size) < encoder_size
            and (not rate or gap_ns is None or abs(gap_ns * 1e-9 * rate - ticks / encoder_size) < 0.5))
