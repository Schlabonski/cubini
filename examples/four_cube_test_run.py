import time
import numpy as np
from cubini.KPZ101 import KPZ101

# serial numbers of KPZ101 cubes
cuboids =  [29250279,
            29250280,
            29250236,
            29250274]

# connect to all modules
cubinis = []
for sn in cuboids:
    try:
        cubini = KPZ101(serial_number=sn)
        time.sleep(0.1)
        cubini.set_input_mode()
        cubinis.append(cubini)
        print('KPZ101 {} found.'.format(sn))
    except Exception as e:
        print(e)
        print('KPZ101 {} not found.'.format(sn))

# sequentially run a test sequence for each cube
for kpz in cubinis:
    kpz.set_max_voltage(150)
    kpz.enable_output()
    kpz.set_output_voltage(0)
    time.sleep(0.5)
    for v in np.linspace(0, 50, 50):
        kpz.set_output_voltage(v)
        time.sleep(0.1)
    kpz.set_output_voltage(0)
    kpz.disable_output()
