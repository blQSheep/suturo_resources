import math
from typing import List

from krrood.entity_query_language.entity import variable, entity, contains
from krrood.entity_query_language.entity_result_processors import an
from krrood.utils import inheritance_path_length, _inheritance_path_length
from semantic_digital_twin.reasoning.predicates import is_supported_by
from semantic_digital_twin.semantic_annotations.semantic_annotations import Milk, Apple, Fruit, Produce, Vegetable, \
    Carrot, Food, Orange, Table, Tomato, Fridge, Banana

from semantic_digital_twin.world_description.world_entity import Region, Body, SemanticAnnotation

from suturo_resources.suturo_map import load_environment


def query_region_area(world, region: str):
    """
    Queries an area from the environment.
    Returns the center of mass and global pose of a given region.
    """
    body = variable(type_=Region, domain=world.regions)
    query = an(entity(body).where(contains(body.name.name, region)))
    kitchen_room_area = list(query.evaluate())[0]
    return kitchen_room_area




def bodies_above_body(main_body: Body) -> List[Body]:
    result= []
    bodies= []
    for connection in main_body._world.connections:
        if str(connection.parent.name) == "root":
            bodies.append(connection.child)
    for body in bodies:
        if body.combined_mesh == None:
            continue
        if is_supported_by(body, main_body, max_intersection_height=0.1):
            result.append(body)
    return result
#print(bodies_above_body(load_environment().get_body_by_name("diningTable_body")))

def get_next_object(supporting_surface):
    obj_distance = {}
    toya_x = load_environment().get_body_by_name("trash_can_body").global_pose.x.to_list()[0]
    toya_y = load_environment().get_body_by_name("trash_can_body").global_pose.y.to_list()[0]
    for obj in bodies_above_body(supporting_surface):
        dx = abs(obj.global_pose.x - toya_x)
        dy = abs(obj.global_pose.y - toya_y)
        dist_sq = dx + dy
        obj_distance[obj] = dist_sq
    sorted_objects = sorted(obj_distance.items(), key=lambda item: item[1])
    return sorted_objects

#print(get_next_object(load_environment().get_body_by_name("diningTable_body")))


def bodies_in_regions(areas : [Body]) -> List[List[Body]]:
    return [bodies_above_body(body) for body in areas]

#print(bodies_in_regions([load_environment().get_body_by_name("lowerTable_body"), load_environment().get_body_by_name("diningTable_body")]))

def query_which_region_to_place_object(handBody: Body, allBodies : List[List[Body]]) -> Body:
    """
    Queries which region to place an object in the environment.
    Returns the region where the object should be placed.
    """
    ...

def bodies_in_regions1(areas : [Body]) -> [SemanticAnnotation]:
    return [bodies_above_body(body) for body in areas]

#print(bodies_in_regions1([load_environment().get_body_by_name("lowerTable_body"), load_environment().get_body_by_name("diningTable_body")]))


def bodies_above_bodies(supporters: [Body]) -> [SemanticAnnotation]:
    result= []
    bodies= []
    for connection in supporters[0]._world.connections:
        if str(connection.parent.name) == "root":
            bodies.append(connection.child)
    for supporter in supporters:
        for body in bodies:
            if body.combined_mesh == None:
                continue
            if is_supported_by(body, supporter, max_intersection_height=0.1):
                result.append(body)
    return result
print (bodies_above_bodies([load_environment().get_body_by_name("lowerTable_body"), load_environment().get_body_by_name("diningTable_body")]))

def query_most_similar_obj(sem : SemanticAnnotation, objekts : [SemanticAnnotation]) -> SemanticAnnotation:
    """
    Queries which region to place an object in the environment.
    Returns the region where the object should be placed.
    """
    path_length = math.inf
    for obj in objekts:
        long_path = obj.__mro__
        for path in long_path:
            if inheritance_path_length(sem, path) == None:
                continue
            if inheritance_path_length(sem, path) < path_length:
                path_length = inheritance_path_length(sem, path)
                most_similar = obj
    return most_similar


#print (query_most_similar_obj(Carrot, [Tomato, Orange, Banana, Table, Fridge]))

