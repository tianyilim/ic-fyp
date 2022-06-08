'''
Configuration of a given test scenario.

Defines the name of the config variable under test, the range of values it can take, and the
number of times to run each test.
'''

import itertools
from typing import Dict, List

# Edit this for the number of repeated experiements per combination
test_repetitions = 2

# Edit this for the number of combinations
test_variables: Dict[str,List] = {
    'linear_speed_limit': [0.4,0.5,0.6,0.7],
    'angular_speed_limit': [1.0, 1.1],
}

def make_grid(pars_dict: Dict[str,List]):
    '''Takes in a dictionary of parameters and finds the Cartesian product of input iterables.
    This gives us a list of combinations to test for.
    
    It also multiplies each combination by `iterations`.
    '''
    keys=pars_dict.keys()
    combinations=itertools.product(*pars_dict.values())
    ds=[dict(zip(keys,cc)) for cc in combinations]
    return ds

# Extract this for test values
TEST_COMBINATIONS = make_grid(test_variables)
TEST_GRID = TEST_COMBINATIONS * test_repetitions