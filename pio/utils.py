from amaranth import *

def saturating_add(x, y, limit):
    added = x + y
    return Mux(added[-1], limit, added[:-1])

def bitmask(n):
    """
    Create a bitmask of `n` ones.
    """
    return (1 << n) - 1

def is_power_of_two(n):
    return (n & (n-1) == 0) and n != 0

def rotate_right(rotatee, amount):
    assert 2**len(amount) <= len(rotatee)

    ret = (Cat(rotatee, rotatee) >> amount)[:len(rotatee)]
    assert len(ret) == len(rotatee)
    return ret
