import math
from core import servo12, servo13, servo22, servo23
from core import servo32, servo33, servo42, servo43
from core import servo52, servo53, servo62, servo63
from core import leg1, leg2, leg3, leg4, leg5, leg6


def init():
    servo12.setPosition(math.pi/6)
    servo13.setPosition(-math.pi/2)

    servo22.setPosition(math.pi/6)
    servo23.setPosition(-math.pi/2)

    servo32.setPosition(math.pi/6)
    servo33.setPosition(-math.pi/2)

    servo42.setPosition(math.pi/6)
    servo43.setPosition(-math.pi/2)

    servo52.setPosition(math.pi/6)
    servo53.setPosition(-math.pi/2)

    servo62.setPosition(math.pi/6)
    servo63.setPosition(-math.pi/2)


steps = [[leg1.complete_move],
         [leg4.complete_move],
         [leg2.complete_move],
         [leg5.complete_move],
         [leg3.complete_move],
         [leg6.complete_move]]
