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


def query_semantic_annotations_on_surfaces(supporting_surfaces : List[SemanticAnnotation]) -> List[SemanticAnnotation]:
    """
    Queries a list of Semantic annotations that are on top of a given list of other annotations (ex. Tables).
    """
    surfaces_bodies = []
    for surface in supporting_surfaces:
        surfaces_bodies.append(surface.bodies[0])
    body = variable(Body, domain=surfaces_bodies[0]._world.bodies_with_enabled_collision)
    results_bodies = []
    results_annotations= []
    for surface in surfaces_bodies:
        results_bodies.append(list(
            an(entity(body).where(is_supported_by(supported_body=body, supporting_body=surface)))
            .evaluate()
        ))
    for result in results_bodies:
        for i in result:
            results_annotations.append(i._semantic_annotations)
    return [item for s in results_annotations for item in s]


def query_get_next_object(supporting_surface):
    """
    This function queries and retrieves semantic annotations from objects located on a given
    surface. It calculates the proximity of each object's associated body to a predefined
    reference object (future Toya, now trash can). The objects are then sorted by this proximity in ascending
    order, based on the distance from the reference. Semantic annotations associated with
    the objects are extracted and returned.
    """
    object_distance = {}
    toya_x = load_environment().get_body_by_name("trash_can_body").global_pose.x.to_list()[0]
    toya_y = load_environment().get_body_by_name("trash_can_body").global_pose.y.to_list()[0]
    bodies = []
    for object in query_semantic_annotations_on_surfaces([supporting_surface]):
        bodies.append(object.bodies)
    for object in bodies:
        dx = abs(object[0].global_pose.x - toya_x)
        dy = abs(object[0].global_pose.y - toya_y)
        dist_sq = dx + dy
        object_distance[object[0]] = dist_sq
    sorted_objects = sorted(object_distance.items(), key=lambda item: item[1])
    result = []
    for body in sorted_objects:
        result.append(body[0]._semantic_annotations)
    return result

def query_get_next_object_euclidean(supporting_surface):
    toya_pos = load_environment().get_semantic_annotation_by_name("trash_can_annotation").body.global_pose.to_list()[0]
    bodies = query_semantic_annotations_on_surfaces([supporting_surface])
    bodies.sort(key=lambda obj: math.dist(obj.body.global_pose.to_list()[0], toya_pos))

    return bodies

print(query_get_next_object_euclidean(load_environment().get_semantic_annotation_by_name("cookingTable_annotation")))

#print(query_semantic_annotations_on_surfaces([load_environment().get_semantic_annotation_by_name("cookingTable_annotation")]))
#print(query_get_next_object_euclidean(load_environment().get_semantic_annotation_by_name("cookingTable_annotation")))
#print(load_environment().get_semantic_annotation_by_name("chips_red_annotation").body.global_pose.to_list()[0])
#print (math.dist(load_environment().get_body_by_name("chips_red_body").global_pose.to_list()[0],load_environment().get_body_by_name("chips_blue_body").global_pose.to_list()[0]))

def query_most_similar_obj(hand_annotation: SemanticAnnotation,objects: list[SemanticAnnotation]) -> SemanticAnnotation:
    """
    Returns the most similar object based on inheritance distance.
    If the minimal inheritance distance is greater than `threshold`,
    returns `hand_annotation`.
    """
    if not objects:
        return hand_annotation

    best_distance = math.inf
    most_similar = None
    counter = 0
    for object in objects:

        for cls in type(object).__mro__:
            dist = inheritance_path_length(type(hand_annotation), cls)
            if dist is None:
                counter = counter + 1
                continue

            if counter < best_distance:
                best_distance = counter
                most_similar = object

        counter = 0
    # Apply threshold
    if best_distance > 1 or most_similar is None:
        return hand_annotation
    return most_similar