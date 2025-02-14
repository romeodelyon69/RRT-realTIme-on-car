import time
import math
import numpy as np

look_up_table = [math.sqrt(i/20) for i in range(0, 1000)]

def test_function():
    temps = time.time()
    for i in range(0, 10000000):
        math.sqrt(i/200000)
    print("Temps d'execution de math.sqrt : ", time.time() - temps)

    temps = time.time()
    for x in range(0, 10000000):
        fast_sqrt(x/200000)
    print("Temps d'execution de fast_sqrt : ", time.time() - temps)

    temps = time.time()
    for i in range(10000000):
        np.sqrt(i/200000)
    print("Temps d'execution de np.sqrt : ", time.time() - temps)

    temps = time.time()
    for i in range(10000000):
        (i/200000)**0.5
    print("Temps d'execution de i**0.5 : ", time.time() - temps)

def fast_sqrt(y):
    return look_up_table[int(y*20)]

print(fast_sqrt(7.6))
test_function()