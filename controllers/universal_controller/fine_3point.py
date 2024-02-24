import math
from core import servo11, servo12, servo13
from core import servo21, servo22, servo23
from core import servo31, servo32, servo33
from core import servo41, servo42, servo43
from core import servo51, servo52, servo53
from core import servo61, servo62, servo63
from core import leg1, leg2, leg3, leg4, leg5, leg6


def init():
    servo11.setPosition(-math.pi/14)
    servo12.setPosition(math.pi/6)
    servo13.setPosition(-math.pi/2)

    servo21.setPosition(math.pi/14)
    servo22.setPosition(math.pi/6)
    servo23.setPosition(-math.pi/2)

    servo31.setPosition(-math.pi/14)
    servo32.setPosition(math.pi/6)
    servo33.setPosition(-math.pi/2)

    servo41.setPosition(math.pi*13/14)
    servo42.setPosition(math.pi/6)
    servo43.setPosition(-math.pi/2)

    servo51.setPosition(math.pi*15/14)
    servo52.setPosition(math.pi/6)
    servo53.setPosition(-math.pi/2)

    servo61.setPosition(math.pi*13/14)
    servo62.setPosition(math.pi/6)
    servo63.setPosition(-math.pi/2)


steps = [[leg1.fine_back, leg2.fine_forward, leg3.fine_back,
          leg4.fine_forward, leg5.fine_back, leg6.fine_forward],
         [leg1.fine_forward, leg2.fine_back, leg3.fine_forward,
          leg4.fine_back, leg5.fine_forward, leg6.fine_back]]
