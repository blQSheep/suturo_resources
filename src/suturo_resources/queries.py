from typing import List

import numpy as np
#from semantic_digital_twin.world_description import *
from krrood.entity_query_language.entity import variable, entity, contains
from krrood.entity_query_language.entity_result_processors import an
from semantic_digital_twin import world
from semantic_digital_twin.collision_checking.trimesh_collision_detector import TrimeshCollisionDetector
from semantic_digital_twin.reasoning.predicates import Above, is_supported_by
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

#print(query_table(load_environment()))

##print(query_table(load_environment()).collision)


def query_milk(world):
    """
    Queries the location of the milk in the environment.
    Returns the x, y, z coordinates of the milk's global pose.
    """
    body = variable(type_=Body, domain=world.bodies)
    query = an(entity(body).where(contains(body.name.name, "milk_body")))
    milk = list(query.evaluate())[0]
    return milk

def query_refrigerator(world):
    """
    Queries the location of the refrigerator in the environment.
    Returns the x, y, z coordinates of the refrigerator's global pose.
    """
    body = variable(type_=Body, domain=world.bodies)
    query = an(entity(body).where(contains(body.name.name, "refrigerator_body")))
    refrigerator = list(query.evaluate())[0]
    return refrigerator



def bodies_above_body(main_body: Body, world: World) -> List[Body]:
    bodies = []
    result = []
    for connection in world.connections:
        if str(connection.parent.name) == "root":
            bodies.append(connection.child)

    pov = HomogeneousTransformationMatrix.from_xyz_rpy(x=0.0, y=0.0, z=0.0)
    for body in bodies:
        if body.combined_mesh == None:
            continue
        if Above(body, main_body, pov)():
            result.append(body)

    tcd = TrimeshCollisionDetector(world)
    for r in result:
        body1 = main_body
        body2 = r
        if not tcd.check_collision_between_bodies(body1, body2):
            result.remove(r)

    return result
#print(bodies_above_body(load_environment().get_body_by_name("apple_body"), load_environment()))

# def test_collision_with_table():
#     world = load_environment()
#     table = world.get_body_by_name("table_body")
#     is_supported_by(world.get_body_by_name("milk_body"), table, world)
#     tcd = TrimeshCollisionDetector(world)
#     for b in world.bodies:
#         if b.combined_mesh == None:
#             continue
#         if tcd.check_collision_between_bodies(b, table):
#             print(f"Collision between {b.name} and table: {tcd.check_collision_between_bodies(b, table)}")


# def test_above():
#     world = load_environment()
#     table = world.get_body_by_name("table_body")
#     counter_top = world.get_body_by_name("counterTop_body")
#     print(is_supported_by(world.get_body_by_name("milk_body"), table, max_intersection_height=0.1))
#     #print(bodies_above_body(table, world))


def bodies_above_body1(main_body: Body, world:World) -> List[Body]:
    result= []
    bodies= []
    for connection in world.connections:
        if str(connection.parent.name) == "root":
            bodies.append(connection.child)
    #print(bodies)
    #print("-------------------------------------------------------------------------------------")
    for body in bodies:
        if body.combined_mesh == None:
            continue
        #print(body.name)
        if is_supported_by(body, main_body, max_intersection_height=0.1):
            result.append(body)
    return result
#print(bodies_above_body1(query_table(load_environment()), load_environment()))




def closest_body_to_me(toyas_body: Body, bodies: [Body], world: World) -> Body:
    toyas_pose_x, toyas_pose_y = toyas_body.global_pose.x, toyas_body.global_pose.y

    # Load environment's bodies
    bodies = []
    for body in load_environment().bodies:
        if body.name.name == "banana_body":
            continue
        bodies.append(body)

    # Initialize with the first body's position
    smallest_x = abs(toyas_pose_x - bodies[0].global_pose.x)
    smallest_y = abs(toyas_pose_y - bodies[0].global_pose.y)
    closest_body = bodies[0]

    for body in bodies:
        diff_x = abs(toyas_pose_x - body.global_pose.x)
        diff_y = abs(toyas_pose_y - body.global_pose.y)

        if (diff_x + diff_y) < (smallest_x + smallest_y):
            smallest_x = diff_x
            smallest_y = diff_y
            closest_body = body

    return closest_body

print(closest_body_to_me(load_environment().get_body_by_name("banana_body"), load_environment().bodies, load_environment()))
