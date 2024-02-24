"""
Universal Controoller

This controller supports many gaits
of hexapod. Gaits are specified throught
GAIT environment variable.
"""

import core
import dumb_mechatronic
import dumb_3point
import fine_3point
import os

if __name__ == "__main__":
    GAIT = os.environ.get('GAIT', None)
    if GAIT is None:
        print("Error: Gait unspecified.")
        quit()
    elif GAIT == 'DUMB_MECHATRONIC':
        """
        Dumb Mechatronic Gait features movement
        of only one leg at a time.
        """
        dumb_mechatronic.init()
        steps = dumb_mechatronic.steps
    elif GAIT == 'DUMB_3POINT':
        """
        Dumb Three-Point Gait features movement
        of three legs at a time.
        """
        dumb_3point.init()
        steps = dumb_3point.steps
    elif GAIT == 'FINE_3POINT':
        """
        Fine Three-Point Gait features sychronized
        movemement of all legs.
        """
        fine_3point.init()
        steps = fine_3point.steps
    else:
        print(f"Error: Unknown gait: \"{GAIT}\".")
        quit()

    core.main(steps)
