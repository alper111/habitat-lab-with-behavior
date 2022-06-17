#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.


from habitat.core.embodied_task import Measure
from habitat.core.registry import registry
from habitat.tasks.rearrange.rearrange_sensors import (
    EndEffectorToObjectDistance,
    ObjectToGoalDistance,
    RearrangeReward,
)
from habitat.tasks.behavior.utils import rearrange_logger



@registry.register_measure
class DoesWantTerminate(Measure):
    cls_uuid: str = "does_want_terminate"

    @staticmethod
    def _get_uuid(*args, **kwargs):
        return DoesWantTerminate.cls_uuid

    def reset_metric(self, *args, **kwargs):
        self.update_metric(*args, **kwargs)

    def update_metric(self, *args, task, **kwargs):
        self._metric = task.actions["REARRANGE_STOP"].does_want_terminate


# ---------------------------------------------- BEHAVIOR Task Measures and Rewards ------------------------------------------
@registry.register_measure
class BehaviorReward(Measure):
    """
    The reward based on where the agent currently is in the hand defined solution list.
    """

    cls_uuid: str = "behavior_reward"

    @staticmethod
    def _get_uuid(*args, **kwargs):
        return BehaviorReward.cls_uuid

    def __init__(self, sim, config, *args, **kwargs):
        super().__init__(**kwargs)
        self._sim = sim
        self._config = config
        self.prev_task_completion_percentage = 0.0

    def reset_metric(self, *args, episode, task, observations, **kwargs):
        self.update_metric(
            *args,
            episode=episode,
            task=task,
            observations=observations,
            **kwargs,
        )
        self.prev_task_completion_percentage = 0.0

    def update_metric(self, *args, episode, task, observations, **kwargs):
        self._metric = self._stage_completion_reward()

    def _stage_completion_reward(self):
        if not self._sim.performed_kinematic_step:
            return 0.0
        bddl_result = self._sim.check_task_success()
        n_satisfied = len(bddl_result[1]["satisfied"])
        n_unsatisfied = len(bddl_result[1]["unsatisfied"])
        curr_completion_percentage = n_satisfied / (n_satisfied + n_unsatisfied)
        reward = self._config.STAGE_REWARD * (curr_completion_percentage - self.prev_task_completion_percentage)
        self.prev_task_completion_percentage = curr_completion_percentage
        return reward



@registry.register_measure
class BehaviorSuccess(Measure):
    """
    Did satisfy all the goal predicates?
    """

    cls_uuid: str = "behavior_success"

    def __init__(self, sim, config, *args, **kwargs):
        super().__init__(**kwargs)
        self._sim = sim
        self._config = config

    @staticmethod
    def _get_uuid(*args, **kwargs):
        return BehaviorSuccess.cls_uuid

    def reset_metric(self, *args, task, **kwargs):
        task.measurements.check_measure_dependencies(
            self.uuid,
            [DoesWantTerminate.cls_uuid],
        )
        self.update_metric(*args, task=task, **kwargs)

    def update_metric(self, *args, episode, task, observations, **kwargs):
        does_action_want_stop = task.measurements.measures[
            DoesWantTerminate.cls_uuid
        ].get_metric()
        task_success = False
        if self._sim.performed_kinematic_step:
            task_success = self._sim.check_task_success()[0]
        self._metric = task_success and does_action_want_stop
        # TODO: should the agent decide to stop by itself?
        if does_action_want_stop or task_success:
            task.should_end = True
