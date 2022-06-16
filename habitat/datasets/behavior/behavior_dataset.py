#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import json
from typing import Dict, List, Optional, Tuple

import attr
import numpy as np

import habitat_sim.utils.datasets_download as data_downloader
from habitat.config import Config
from habitat.core.dataset import Episode
from habitat.core.logging import logger
from habitat.core.registry import registry
from habitat.core.utils import DatasetFloatJSONEncoder, not_none_validator
from habitat.datasets.pointnav.pointnav_dataset import PointNavDatasetV1
from habitat.datasets.utils import check_and_gen_physics_config


@attr.s(auto_attribs=True, kw_only=True)
class BehaviorObjectSpec:
    r"""Object specifications that capture position of each object in the scene,
    the associated object template.
    """
    object_id: str = attr.ib(default=None, validator=not_none_validator)
    object_name: str = attr.ib(default=None, validator=not_none_validator)
    category: str = attr.ib(default=None, validator=not_none_validator)
    room: str = attr.ib(default=None, validator=not_none_validator)
    states: List[float] = attr.ib(default=None, validator=not_none_validator)
    fixed: bool = attr.ib(default=False, validator=not_none_validator)
    bddl_scope: str = attr.ib(default=None, validator=not_none_validator)
    object_template: str = attr.ib(default=None, validator=not_none_validator)
    start_position: List[float] = attr.ib(default=None, validator=not_none_validator)
    start_rotation: List[float] = attr.ib(default=None, validator=not_none_validator)

    info: Optional[Dict[str, str]] = attr.ib(default=None)


@attr.s(auto_attribs=True, kw_only=True)
class BehaviorEpisode(Episode):
    r"""Specifies additional objects, targets, markers, and ArticulatedObject states for a particular instance of an object rearrangement task.

    :property ao_states: Lists modified ArticulatedObject states for the scene: {instance_handle -> {link, state}}
    :property rigid_objs: A list of objects to add to the scene, each with: (handle, transform)
    :property targets: Maps an object instance to a new target location for placement in the task. {instance_name -> target_transform}
    :property markers: Indicate points of interest in the scene such as grasp points like handles. {marker name -> (type, (params))}
    :property target_receptacles: The names and link indices of the receptacles containing the target objects.
    :property goal_receptacles: The names and link indices of the receptacles containing the goals.
    """
    objects: List[BehaviorObjectSpec] = attr.ib(
        default=None, validator=not_none_validator
    )
    bddl_goals: List[List[str]] = attr.ib(default=None, validator=not_none_validator)
    activity: str = attr.ib(default=None, validator=not_none_validator)


@registry.register_dataset(name="BehaviorDataset-v0")
class BehaviorDatasetV0(PointNavDatasetV1):
    r"""Class inherited from PointNavDataset that loads Rearrangement dataset."""
    episodes: List[BehaviorEpisode] = []  # type: ignore
    content_scenes_path: str = "{data_path}/content/{scene}.json.gz"

    def to_json(self) -> str:
        result = DatasetFloatJSONEncoder().encode(self)
        return result

    def __init__(self, config: Optional[Config] = None) -> None:
        self.config = config

        if config and not self.check_config_paths_exist(config):
            logger.info(
                "Invalid path in dataset config. Exiting..."
            )
            exit(-1)

        check_and_gen_physics_config()

        super().__init__(config)

    def from_json(
        self, json_str: str, scenes_dir: Optional[str] = None
    ) -> None:
        deserialized = json.loads(json_str)

        for i, episode in enumerate(deserialized["episodes"]):
            rearrangement_episode = BehaviorEpisode(**episode)
            rearrangement_episode.episode_id = str(i)

            self.episodes.append(rearrangement_episode)
