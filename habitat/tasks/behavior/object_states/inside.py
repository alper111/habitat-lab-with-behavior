import numpy as np


# check if obj1 inside obj2 aabb
def aabb_contains_point(sim, obj1, obj2):
    t_1 = sim.get_articulated_object_manager().get_object_by_id(obj1).translation
    lower = sim.object_manual_bb[obj2].min
    upper = sim.object_manual_bb[obj2].max
    return np.less_equal(lower, t_1).all() and \
        np.less_equal(t_1, upper).all()


class Inside:
    def __init__(self, sim):
        self.sim = sim

    # pass in current object handle and other object (id)
    def _get_value(self, this, other):
        # sim must have an ObjectStates attribute
        assert hasattr(self.sim, "object_states")

        if not aabb_contains_point(self.sim, this, other):
            # print("aabb doesn't contain point")
            return False

        # First check Z axis, whether this is on both directions of Z
        vertical_adjacency = self.sim.object_states.get_state(this, "vertical_adjacency_by_axis")
        horizontal_adjacency = self.sim.object_states.get_state(this, "horizontal_adjacency_by_axis")

        on_both_sides_z = (
            other in vertical_adjacency[0][0] and other in vertical_adjacency[0][1]
        )
        if on_both_sides_z:
            # If the object is on both sides of Z, we already found 1 axis, so just
            # find another axis where the object is on both sides.
            on_both_sides_in_any_axis = any(
                (other in bodies_by_axis[0] and other in bodies_by_axis[1] for bodies_by_axis in horizontal_adjacency)
            )
            return on_both_sides_in_any_axis

        # If the object was not on both sides of Z, then we need to look at each
        # plane and try to find one where the object is on both sides of both
        # axes in that plane.
        on_both_sides_of_both_axes_in_any_plane = any(
            other in first_axis[0] and other in first_axis[1]
            and other in second_axis[0] and other in second_axis[1]
            for first_axis, second_axis in zip(horizontal_adjacency[::2], horizontal_adjacency[1::2])
        )
        return on_both_sides_of_both_axes_in_any_plane
