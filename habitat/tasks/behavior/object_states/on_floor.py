
class OnFloor:
    def __init__(self, sim):
        self.sim = sim

    # pass in current object handle and other object (id)
    def _get_value(self, this):
        # sim must have an ObjectStates attribute
        assert hasattr(self.sim, "object_states")

        touching_list = self.sim.object_states.get_state(this, "touching")

        for touching_id in touching_list:
            if touching_id in self.sim.objects_by_category["floors"]:
                return True

        return False
