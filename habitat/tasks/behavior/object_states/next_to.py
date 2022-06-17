import numpy as np


class NextTo:
    def __init__(self, sim):
        self.sim = sim

    def _aabb_check(self, sim, this, other):
        bb_1 = sim.object_manual_bb[this]
        bb_2 = sim.object_manual_bb[other]
        distance_vec = []
        for dim in range(3):
            glb = max(bb_1.min[dim], bb_2.min[dim])
            lub = min(bb_1.max[dim], bb_2.max[dim])
            distance_vec.append(max(0, glb - lub))
        distance = np.linalg.norm(np.array(distance_vec))
        obj_1_dims = bb_1.max - bb_1.min
        obj_2_dims = bb_2.max - bb_2.min
        avg_aabb_length = np.mean(obj_1_dims + obj_2_dims)
        # If the distance is longer than acceptable, return False.
        if distance > avg_aabb_length * (1.0 / 6.0):
            return False
        return True

    # pass in current object handle and other object (id)
    def _get_value(self, this, other):
        # sim must have an ObjectStates attribute
        assert hasattr(self.sim, "object_states")

        # AABB check
        if not self._aabb_check(self.sim, this, other):
            return False
        # if this shorter than other
        for axis in self.sim.object_states.get_state(this, "horizontal_adjacency_by_axis"):
            for direction in axis:
                if other in direction:
                    return True
        # if other shorter than this
        for axis in self.sim.object_states.get_state(other, "horizontal_adjacency_by_axis"):
            for direction in axis:
                if this in direction:
                    return True
        return False
