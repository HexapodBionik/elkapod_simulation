import yaml
import numpy as np

class SetupNotFoundError(Exception):
    pass


def load_setup(path: str):
    try:
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
            f.close()
        return data
    except FileNotFoundError:
        raise SetupNotFoundError