
class OnTop:
    def __init__(self, sim):
        self.sim = sim

    # pass in current object handle and other object (id)
    def _get_value(self, this, other):
        # sim must have an ObjectStates attribute
        assert hasattr(self.sim, "object_states")

        # check vertical adjacency
        vertical_adjacency = self.sim.object_states.get_state(this, "vertical_adjacency_by_axis")
        touching_list = self.sim.object_states.get_state(this, "touching")
        vertical_adjacency_other = self.sim.object_states.get_state(other, "vertical_adjacency_by_axis")

        # check touching (probably doesn't need touching for on top
        # if other not in touching_list:
        #     return False
        this_ontop_other = (other in vertical_adjacency[0][1]) and (other not in vertical_adjacency[0][0])
        other_under_this = (this in vertical_adjacency_other[0][0]) and (this not in vertical_adjacency_other[0][1])

        return this_ontop_other or other_under_this
