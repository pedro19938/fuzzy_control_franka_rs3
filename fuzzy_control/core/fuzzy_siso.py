"""
Mamdani fuzzy controller with 1 input and 1 output (SISO).

The 4 stages:
  1. Fuzzification   -> degree of the error in each set (NB, NS, ZE, PS, PB)
  2. Rules           -> strength of each rule = degree of its antecedent
  3. Aggregation     -> clips each output set at the rule strength (min)
                        and joins them all (max)
  4. Defuzzification -> centroid of the aggregated shape = final number
"""

import numpy as np

from fuzzy_control.configs import fuzzy_rules as cfg
from fuzzy_control.core.membership import make_function


class FuzzySISO:
    def __init__(self, input_sets=None, output_sets=None, rules=None,
                 v_max=None, n_points=None):
        # configs in configs/fuzzy_rules.py

        # input sets, output sets and rules
        input_sets = input_sets or cfg.INPUT_SETS
        output_sets = output_sets or cfg.OUTPUT_SETS
        self.rules = rules or cfg.RULES

        # allowed V_max and number of points to evaluate the output
        v_max = v_max or cfg.V_MAX
        n_points = n_points or cfg.N_OUTPUT_POINTS

        # creates membership functions for each input and output set
        self.inputs = {name: make_function(*mf) for name, mf in input_sets.items()}

        # The output is evaluated only once, on a discrete universe
        self.v = np.linspace(-v_max, v_max, n_points)
        self.outputs = {name: make_function(*mf)(self.v) for name, mf in output_sets.items()}

    def fuzzify(self, error):
        """Stage 1: {set: degree} for an error value."""
        return {name: float(mf(error)) for name, mf in self.inputs.items()}

    def compute(self, error, details=False):
        """Stages 1 to 4: error (m) -> velocity (m/s)."""
        degrees = self.fuzzify(error)

        aggregated = np.zeros_like(self.v)
        for if_set, then_set in self.rules:
            strength = degrees[if_set]                              # stage 2
            clipped = np.minimum(strength, self.outputs[then_set])  # implication (min)
            aggregated = np.maximum(aggregated, clipped)            # stage 3 (max)

        # defuzzification: centroid of the aggregated shape
        area = aggregated.sum()
        v = 0.0 if area == 0 else float((self.v * aggregated).sum() / area)  # stage 4

        if details:
            return v, degrees, aggregated
        return v
