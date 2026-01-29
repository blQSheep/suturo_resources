import math
from typing import List

from krrood.entity_query_language.entity import variable, entity, contains
from krrood.entity_query_language.entity_result_processors import an
from krrood.utils import inheritance_path_length, _inheritance_path_length
from semantic_digital_twin.reasoning.predicates import is_supported_by
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Milk,
    Apple,
    Fruit,
    Produce,
    Vegetable,
    Carrot,
    Food,
    Orange,
    Table,
    Tomato,
    Fridge,
    Banana,
    Bread,
)

from semantic_digital_twin.world_description.world_entity import (
    Region,
    Body,
    SemanticAnnotation,
)

from conftest import test_load_world
from suturo_resources.suturo_map import load_environment


def query_semantic_annotations_on_surfaces(
    supporting_surfaces: List[SemanticAnnotation],
) -> List[SemanticAnnotation]:
    """
    Queries a list of Semantic annotations that are on top of a given list of other annotations (ex. Tables).
    """
    surfaces_bodies = []
    for surface in supporting_surfaces:
        surfaces_bodies.append(surface.bodies[0])
    body = variable(
        Body, domain=surfaces_bodies[0]._world.bodies_with_enabled_collision
    )
    results_bodies = []
    results_annotations = []
    for surface in surfaces_bodies:
        results_bodies.append(
            list(
                an(
                    entity(body).where(
                        is_supported_by(supported_body=body, supporting_body=surface)
                    )
                ).evaluate()
            )
        )
    for result in results_bodies:
        for i in result:
            results_annotations.append(i._semantic_annotations)
    return [item for s in results_annotations for item in s]


def query_get_next_object_euclidean(mainBody: Body, supporting_surface):
    toya_pos = mainBody.global_pose.to_position().to_list()[:2]
    bodies = query_semantic_annotations_on_surfaces([supporting_surface])
    bodies.sort(
        key=lambda obj: math.dist(
            obj.body.global_pose.to_position().to_list()[:2], toya_pos
        )
    )
    return bodies


def query_most_similar_obj(
    hand_annotation: SemanticAnnotation,
    objects: List[SemanticAnnotation],
    threshhold: int = 1,
) -> SemanticAnnotation:
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
    if best_distance > threshhold or most_similar is None:
        return hand_annotation
    return most_similar


world1 = test_load_world()
banana = world1.get_semantic_annotation_by_name("banana_annotation")
table1 = world1.get_semantic_annotation_by_name("fruit_table_annotation")
table2 = world1.get_semantic_annotation_by_name("vegetable_table_annotation")

print(query_most_similar_obj(banana, [table1]))

# print(query_semantic_annotations_on_surfaces([table1]))
