
class Touching:
    def __init__(self, sim):
        self.sim = sim

    # pass in current object id and other object
    def _get_value(self, this, other):
        # sim must have an ObjectStates attribute
        assert hasattr(self.sim, "object_states")
        return other in self.sim.object_states.get_state(this, "touching")
