from typing import List

import numpy as np
#from semantic_digital_twin.world_description import *
from krrood.entity_query_language.entity import variable, entity, contains
from krrood.entity_query_language.entity_result_processors import an
from semantic_digital_twin import world
from semantic_digital_twin.reasoning.predicates import Above
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import Region, Body, SemanticAnnotation
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
#from semantic_digital_twin.semantic_annotations.semantic_annotations import root_C_milk
from xacro import Table

from suturo_resources.suturo_map import load_environment


def query_kitchen_area(world):
    """
    Queries the kitchen area from the environment.
    Returns the center of mass and global pose of the kitchen region.
    """
    body = variable(type_=Region, domain=world.regions)
    query = an(entity(body).where(contains(body.name.name, "kitchen")))
    kitchen_room_area = list(query.evaluate())[0]
    return kitchen_room_area

def query_living_room_area(world):
    """
    Queries the living room area.
    Returns the center of mass and global pose of the living room region.
    """
    body = variable(type_=Region, domain=world.regions)
    query = an(entity(body).where(contains(body.name.name, "living_room")))
    living_room_area = list(query.evaluate())[0]
    return living_room_area

def query_bed_room_area(world):
    """
    Queries the bedroom area.
    Returns the center of mass and global pose of the bedroom region.
    """
    body = variable(type_=Region, domain=world.regions)
    query = an(entity(body).where(contains(body.name.name, "bed_room")))
    bed_room_area = list(query.evaluate())[0]
    return bed_room_area

def query_office_area(world):
    """
    Queries the office area.
    Returns the center of mass and global pose of the office region.
    """
    body = variable(type_=Region, domain=world.regions)
    query = an(entity(body).where(contains(body.name.name, "office")))
    office_area = list(query.evaluate())[0]
    return office_area

def query_trash(world):
    """
    Queries the location of the trash can in the environment.
    Returns the x, y, z coordinates of the trash can's global pose.
    """
    body = variable(type_=Body, domain=world.bodies)
    query = an(entity(body).where(contains(body.name.name, "trash_can_body")))
    trash_can = list(query.evaluate())[0]
    return trash_can


def query_table(world):

    body = variable(type_=Body, domain=world.bodies)
    query = an(entity(body).where(contains(body.name.name, "table_body")))
    table = list(query.evaluate())[0]
    return table

##print(query_table(load_environment()).collision)
##print("hi")

def on_top_of_table(world: World):
    table = world.get_body_by_name("table_body")
    trash_can = world.get_body_by_name("trash_can_body")
    pov = HomogeneousTransformationMatrix.from_xyz_rpy(x=0.2, y=0, z=0.6)

    bodies_above_target = []

    above_relation = Above(table, trash_can, pov)

    return above_relation

#print(on_top_of_table(load_environment()))



# def query_pickup_closest(SemanticAnnotation, SemanticAnnotationType, world):
#     """
#     Queries the location of the pickup closet in the environment.
#     Returns the x, y, z coordinates of the pickup closet's global pose.
#     """
#     body = variable(type_=Body, domain=world.bodies)
#     query = an(entity(body).where(contains(body.name.name, "pickup_closet_body")))
#     pickup_closet = list(query.evaluate())[0]
#     return pickup_closet


# def euclidean_distance(p1, p2):
#     return np.linalg.norm(
#         np.array([p1.x, p1.y, p1.z]) -
#         np.array([p2.x, p2.y, p2.z])
#     )
#
#
# def closest_object_above_table_to_trash_can(world: World):
#     # Reference bodies
#     trash_can = world.get_body_by_name("trash_can_body")
#     table = world.get_body_by_name("table_body")
#
#     trash_pose = world.get_global_pose(trash_can).translation
#
#     candidates = []
#
#     for body in world.get_bodies():
#         # Skip trivial cases
#         if body in (trash_can, table):
#             continue
#
#         # Spatial relation: body ABOVE table
#         above_relation = Above(
#             reference_body=table,
#             target_body=body,
#             world=world
#         )
#
#         if above_relation():
#             body_pose = world.get_global_pose(body).translation
#             dist = euclidean_distance(trash_pose, body_pose)
#             candidates.append((body, dist))
#
#     if not candidates:
#         return None
#
#     # Return closest body (and its global pose)
#     closest_body, _ = min(candidates, key=lambda x: x[1])
#     closest_pose = world.get_global_pose(closest_body)
#
#     return closest_body, closest_pose
#
#


# getting every body above??
# def get_bodies_above(reference_body: Body, world: World) -> List[Body]:
#     """
#     Return all bodies in `world` that are above `reference_body`.
#     Tries multiple `Above` constructor signatures and falls back safely.
#     """
#     # get bodies collection (supports either get_bodies() or .bodies)
#     if hasattr(world, "get_bodies") and callable(world.get_bodies):
#         bodies = list(world.get_bodies())
#     else:
#         bodies = list(getattr(world, "bodies", []))
#
#     results: List[Body] = []
#     # small POV used if Above requires a point_of_semantic_annotation
#     pov = HomogeneousTransformationMatrix.from_xyz_rpy(x=0.0, y=0.0, z=0.0)
#
#     for b in bodies:
#         if b is reference_body:
#             continue
#
#         # Try common constructor signatures for Above
#         matched = False
#         try:
#             # signature: Above(reference_body=..., target_body=..., world=...)
#             pred = Above(reference_body=reference_body, target_body=b, world=world)
#             if callable(pred) and pred():
#                 results.append(b)
#                 matched = True
#         except TypeError:
#             pass
#         except Exception:
#             pass
#
#         if matched:
#             continue
#
#         try:
#             # signature: Above(body=body_a, other=body_b, point_of_semantic_annotation=...)
#             pred = Above(body=b, other=reference_body, point_of_semantic_annotation=pov)
#             if callable(pred) and pred():
#                 results.append(b)
#                 matched = True
#         except Exception:
#             pass
#
#     return results
#
# print(get_bodies_above(query_trash(load_environment()), load_environment()))

#################################################

# getting an empty list??
# def is_in_domain(candidate, reference_body, world) -> bool:
#     # 1) `domain` as a collection or a type
#     if hasattr(reference_body, "domain"):
#         domain_attr = reference_body.domain
#         if isinstance(domain_attr, (list, tuple, set)):
#             return candidate in domain_attr
#         if isinstance(domain_attr, type):
#             return isinstance(candidate, domain_attr)
#
#     # 2) region / parent / kinematic structure equality checks
#     if hasattr(reference_body, "region"):
#         if getattr(candidate, "region", None) == reference_body.region:
#             return True
#     if hasattr(reference_body, "parent"):
#         if getattr(candidate, "parent", None) == reference_body.parent:
#             return True
#     if hasattr(reference_body, "kinematic_structure"):
#         if getattr(candidate, "kinematic_structure", None) == reference_body.kinematic_structure:
#             return True
#
#     # 3) world helper (if available)
#     if hasattr(world, "get_bodies_in_region") and hasattr(reference_body, "region"):
#         try:
#             return candidate in world.get_bodies_in_region(reference_body.region)
#         except Exception:
#             pass
#
#     # Unknown domain shape -> exclude to respect "in the domain of the main body"
#     return False
#
#
# def get_bodies_above(reference_body: Body, world: World) -> List[Body]:
#     """
#     Return all bodies in `world` that are above `reference_body` AND are in the
#     `domain` of `reference_body`.
#     """
#     if hasattr(world, "get_bodies") and callable(world.get_bodies):
#         bodies = list(world.get_bodies())
#     else:
#         bodies = list(getattr(world, "bodies", []))
#
#     results: List[Body] = []
#     pov = HomogeneousTransformationMatrix.from_xyz_rpy(x=0.0, y=0.0, z=0.0)
#
#     for b in bodies:
#         if b is reference_body:
#             continue
#
#         # enforce domain membership first
#         if not is_in_domain(b, reference_body, world):
#             continue
#
#         matched = False
#         try:
#             pred = Above(reference_body=reference_body, target_body=b, world=world)
#             if callable(pred) and pred():
#                 results.append(b)
#                 matched = True
#         except TypeError:
#             pass
#         except Exception:
#             pass
#
#         if matched:
#             continue
#
#         try:
#             pred = Above(body=b, other=reference_body, point_of_semantic_annotation=pov)
#             if callable(pred) and pred():
#                 results.append(b)
#                 matched = True
#         except Exception:
#             pass
#
#     return results
#print(get_bodies_above(query_table(load_environment()), load_environment()))


def get_xy_bounds(body: Body, world: World):
    """
    Returns (x_min, x_max, y_min, y_max) in world coordinates.
    """
    pose = world.get_global_pose(body)
    center = pose.translation  # assumes .x .y .z or array-like

    # Adjust this if your body uses a different size attribute
    sx, sy, _ = body.scale  # full size, not half-size

    half_x = sx / 2.0
    half_y = sy / 2.0

    x_min = center.x - half_x
    x_max = center.x + half_x
    y_min = center.y - half_y
    y_max = center.y + half_y

    return x_min, x_max, y_min, y_max

def get_bodies_above(reference_body: Body, world: World) -> List[Body]:
    """
    Returns all bodies that:
    1) are above reference_body (z-axis)
    2) whose global (x, y) lies within the XY footprint of reference_body
    """
    if hasattr(world, "get_bodies"):
        bodies = world.get_bodies()
    else:
        bodies = world.bodies

    ref_pose = world.get_global_pose.to_list()[0](reference_body)
    ref_z = ref_pose.translation.z

    x_min, x_max, y_min, y_max = get_xy_bounds(reference_body, world)

    results = []

    for b in bodies:
        if b is reference_body:
            continue

        pose = world.get_global_pose(b)
        p = pose.translation

        # Above condition
        if p.z <= ref_z:
            continue

        # XY domain condition
        if not (x_min <= p.x <= x_max and y_min <= p.y <= y_max):
            continue

        results.append(b)

    return results
print(get_bodies_above(query_table(load_environment()), load_environment()))