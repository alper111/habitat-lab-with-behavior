
class FakeUnary:
    def __init__(self, sim):
        self.sim = sim

    # pass in current object handle and other object (category or handle)
    def _get_value(self, this):
        # sim must have an ObjectStates attribute
        return True
