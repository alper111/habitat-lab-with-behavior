"""
    Adapted from: https://github.com/StanfordVL/iGibson/blob/ig-develop/igibson/object_states/adjacency.py
"""

import numpy as np
import magnum as mn

import habitat
import habitat_sim
from habitat_sim.geo import Ray
# from src.utils import axes_raycast, get_object_id_by_handle, get_object_handle_by_id


_MAX_ITERATIONS = 10
_MAX_DISTANCE_VERTICAL = 5.0
_MAX_DISTANCE_HORIZONTAL = 1.0

# How many 2-D bases to try during horizontal adjacency check. When 1, only the standard axes will be considered.
# When 2, standard axes + 45 degree rotated will be considered. The tried axes will be equally spaced. The higher
# this number, the lower the possibility of false negatives in Inside.
# basically how many rays to shoot in the horizontal plane,  each axis corresponds to a cross
_HORIZONTAL_AXIS_COUNT = 3


class ObjectStates(object):
    def __init__(self, object_ids=[]):
        self.object_states = {}
        for object_id in object_ids:
            self.object_states[object_id] = {
                "horizontal_adjacency_list": [],
                "vertical_adjacency_list": [],
                "horizontal_adjacency_by_axis": [],
                "vertical_adjacency_by_axis": [],
                "touching": [],
                "aabb": {"lower": [], "upper": []},
                "prev_translation": None,
                "prev_rotation": None
            }

    def update_state(self, object_id, state_key, state_value):
        self.object_states[object_id][state_key] = state_value

    def get_state(self, object_id, state_key):
        return self.object_states[object_id][state_key]


def axes_raycast(sim, origin, direction, max_distance=2.0):
    r"""Cast a ray in the direction of crosshair and check if it collides
    with another object within a certain distance threshold
    :param sim: Simulator object
    :param sensor_name: name of the visual sensor to be used for raycasting
    :param crosshair_pos: 2D coordiante in the viewport towards which the
        ray will be cast
    :param max_distance: distance threshold beyond which objects won't
        be considered
    """
    # render_camera = sim._sensors[sensor_name]._sensor_object.render_camera
    # center_ray = render_camera.unproject(mn.Vector2i(crosshair_pos))
    center_ray = Ray(origin, direction)

    raycast_results = sim.cast_ray(center_ray, max_distance=max_distance)

    assert hasattr(sim, "link_id_to_object_id")

    hit_objects = []
    if raycast_results.has_hits():
        for hit in raycast_results.hits:
            # seems that somehow the object_id here is 1-based...
            # Nope! It's because hit.object_id is the link id, an object may have many ids
            # print("hit: ", hit.object_id)
            # if hasattr(sim, "link_id_to_object_id"):
            base_link_id = sim.link_id_to_object_id[hit.object_id]
            # else:
            #     base_link_id = link_id_to_object_id[hit.object_id]
            # print("base_link_id:", base_link_id )
            hit_objects.append(base_link_id)

    return hit_objects


def get_equidistant_coordinate_planes(n_planes=_HORIZONTAL_AXIS_COUNT):
    """Given a number, sample that many equally spaced coordinate planes.
    The samples will cover all 360 degrees (although rotational symmetry
    is assumed, e.g. if you take into account the axis index and the
    positive/negative directions, only 1/4 of the possible coordinate
    planes will be sampled: the ones where the first axis' positive direction
    is in the first quadrant).
    :param n_planes: number of planes to sample
    :return np.array of shape (n_planes, 2, 3) where the first dimension
        is the sampled plane index, the second dimension is the axis index
        (0/1), and the third dimension is the 3-D world-coordinate vector
        corresponding to the axis.

    Modifications:
        1. Changed from Z-up to Y-up
    """
    # Compute the positive directions of the 1st axis of each plane.
    first_axis_angles = np.linspace(0, np.pi / 2, n_planes)
    first_axes = np.stack(
        [np.sin(first_axis_angles), np.zeros_like(first_axis_angles), np.cos(first_axis_angles)], axis=1
    )

    # Compute the positive directions of the 2nd axes. These axes are
    # orthogonal to both their corresponding first axes and to the Z axis.
    second_axes = np.cross([0, 1, 0], first_axes)

    # Return the axes in the shape (n_planes, 2, 3)
    return np.stack([first_axes[:, None, :], second_axes[:, None, :]], axis=1)


def compute_adjacencies(sim, obj_id, axes, max_distance):
    """
    Given an object and a list of axes, find the adjacent objects in the axes'
    positive and negative directions.
    :param obj: The object to check adjacencies of.
    :param axes: The axes to check in. Note that each axis will be checked in
        both its positive and negative direction.
    # :return: List[AxisAdjacencyList] of length len(axes) containing the adjacencies.
    :return: List of object_id that were hit by casted rays, within max_distance
    """
    # Get vectors for each of the axes' directions.
    # The ordering is axes1+, axis1-, axis2+, axis2- etc.
    directions = np.empty((len(axes) * 2, 3))
    directions[0::2] = axes
    directions[1::2] = -axes

    # For now, we keep our result in the dimensionality of (direction, hit_object_order).
    # finalized = np.zeros(directions.shape[0], dtype=np.bool)
    # bodies_by_direction = [[] for _ in directions]

    aom = sim.get_articulated_object_manager()
    rom = sim.get_rigid_object_manager()

    # Prepare this object's info for ray casting.
    # Use AABB center instead of position because we cannot get valid position
    # for fixed objects if fixed links are merged.
    # if aom.get_library_has_id(obj_id):
    #     obj_pos = aom.get_object_by_id(obj_id).translation
    # elif rom.get_library_has_id(obj_id):
    #     obj_pos = rom.get_object_by_id(obj_id).translation
    # else:
    #     raise RuntimeError("Object not found.")

    # TODO: check this. using center of aabb to start raycast
    # aabbA = aom.get_object_by_handle(obj_handle).get_link_scene_node(0).cumulative_bb
    bb = sim.object_manual_bb[obj_id]
    obj_pos = mn.Vector3(np.array([bb.max.x + bb.min.x, bb.max.y + bb.min.y, bb.max.z + bb.min.z]) / 2.0)

    # obj_pos[1] += aabbA.max[2]

    bodies_by_direction = []

    for i in range(directions.shape[0]):
        direction_hits = axes_raycast(sim, mn.Vector3(obj_pos), mn.Vector3(directions[i]), max_distance)
        bodies_by_direction.append(list(set(direction_hits)))

    bodies_by_axis = [
        [positive_neighbors, negative_neighbors]
        for positive_neighbors, negative_neighbors in zip(bodies_by_direction[::2], bodies_by_direction[1::2])
    ]
    return bodies_by_axis
