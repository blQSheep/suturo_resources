import math
from typing import List

from krrood.entity_query_language.entity import variable, entity, contains
from krrood.entity_query_language.entity_result_processors import an
from krrood.utils import inheritance_path_length, _inheritance_path_length
from semantic_digital_twin.reasoning.predicates import is_supported_by
from semantic_digital_twin.semantic_annotations.semantic_annotations import Milk, Apple, Fruit, Produce, Vegetable, \
    Carrot, Food, Orange, Table, Tomato, Fridge, Banana, Bread

from semantic_digital_twin.world_description.world_entity import Region, Body, SemanticAnnotation

from suturo_resources.suturo_map import load_environment


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


def query_semantic_annotations_on_surfaces(supporting_surfaces : List[SemanticAnnotation]) -> List[SemanticAnnotation]:
    """
    Queries a lis of Semantic annotations on top of a given list of other annotations.
    """
    surfaces2 = []
    for s in supporting_surfaces:
        surfaces2.append(s.bodies[0])
    body = variable(Body, domain=surfaces2[0]._world.bodies_with_enabled_collision)
    results = []
    result2= []
    for s in surfaces2:
        results.append(list(
            an(entity(body).where(is_supported_by(supported_body=body, supporting_body=s)))
            .evaluate()
        ))
    for r in results:
        for i in r:
            result2.append(i._semantic_annotations)
    return [item for s in result2 for item in s]


def query_get_next_object(supporting_surface):
    #supporting_surface = supporting_surface.bodies[0]
    obj_distance = {}
    toya_x = load_environment().get_body_by_name("trash_can_body").global_pose.x.to_list()[0]
    toya_y = load_environment().get_body_by_name("trash_can_body").global_pose.y.to_list()[0]
    bodies = []
    for obj in query_semantic_annotations_on_surfaces([supporting_surface]):
        bodies.append(obj.bodies)
    for obj in bodies:
        dx = abs(obj[0].global_pose.x - toya_x)
        dy = abs(obj[0].global_pose.y - toya_y)
        dist_sq = dx + dy
        obj_distance[obj[0]] = dist_sq
    sorted_objects = sorted(obj_distance.items(), key=lambda item: item[1])
    result = []
    for body in sorted_objects:
        result.append(body[0]._semantic_annotations)
    return result


def query_most_similar_obj(sem: SemanticAnnotation,objects: list[SemanticAnnotation]) -> SemanticAnnotation:
    """
    Returns the most similar object based on inheritance distance.
    If the minimal inheritance distance is greater than `threshold`,
    returns `sem`.
    """
    if not objects:
        return sem

    best_distance = math.inf
    most_similar = None

    for obj in objects:
        for cls in type(obj).__mro__:
            dist = inheritance_path_length(type(sem), cls)
            if dist is None:
                continue

            if dist < best_distance:
                best_distance = dist
                most_similar = obj

    # Apply threshold
    if best_distance > 1 or most_similar is None:
        return sem

    return most_similar