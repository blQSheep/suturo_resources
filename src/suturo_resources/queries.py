import math
from typing import List, Union
from krrood.entity_query_language.entity import (
    entity,
    variable_from,
)
from krrood.entity_query_language.symbolic import QueryObjectDescriptor, Entity
from krrood.utils import inheritance_path_length
from semantic_digital_twin.reasoning.predicates import (
    is_supported_by,
    compute_euclidean_distance_2d,
)
from semantic_digital_twin.world import World

from semantic_digital_twin.world_description.world_entity import (
    Body,
    SemanticAnnotation,
)


def query_semantic_annotations_on_surfaces(
    supporting_surfaces: List[SemanticAnnotation], world: World
) -> Union[Entity[SemanticAnnotation], SemanticAnnotation]:
    """
    Queries a list of Semantic annotations that are on top of a given list of other annotations (ex. Tables).
    param: supporting_surfaces: List of SemanticAnnotations that are supporting other annotations.
    :param world: World object that contains the supporting_surfaces.
    return: List of SemanticAnnotations that are supported by the given supporting_surfaces.
    """
    supporting_surfaces_var = variable_from(supporting_surfaces)
    body_with_enabled_collision = variable_from(world.bodies_with_enabled_collision)
    semantic_annotations = variable_from(
        body_with_enabled_collision._semantic_annotations
    )
    semantic_annotations_that_are_supported = entity(semantic_annotations).where(
        is_supported_by(
            supported_body=body_with_enabled_collision,
            supporting_body=supporting_surfaces_var.bodies[0],
        )
    )
    return semantic_annotations_that_are_supported


def query_get_next_object_euclidean_x_y(
    main_body: Body,
    supporting_surface,
) -> QueryObjectDescriptor[SemanticAnnotation]:
    """
    Queries the next object based on Euclidean distance in x and y coordinates
    relative to the given main body and supporting surface. This function utilizes
    semantic annotations of objects and orders them by their Euclidean distances
    to the main body.

    :param main_body: The main body to which the Euclidean distance is computed.
    :param supporting_surface: The surface on which the semantic annotations
        of interest are queried.
    :return: A `QueryObjectDescriptor` containing semantic annotations ordered
        by Euclidean distance to the main body.
    """
    supported_semantic_annotations = query_semantic_annotations_on_surfaces(
        [supporting_surface], main_body._world
    )
    return supported_semantic_annotations.order_by(
        compute_euclidean_distance_2d(
            body1=supported_semantic_annotations.selected_variable.bodies[0],
            body2=main_body,
        )
    )


def query_most_similar_obj(
    hand_annotation: SemanticAnnotation,
    objects: List[SemanticAnnotation],
    threshold: int = 1,
) -> SemanticAnnotation:
    """
    Finds the most similar object from a list of provided objects to a given
    hand-annotated semantic annotation, based on their inheritance
    distance within a class hierarchy.

    :param hand_annotation: The semantic annotation that serves as a reference
        for similarity comparison.
    :param objects: A list of semantic annotations to compare against the
        provided hand annotation.
    :param threshold: The maximum allowable distance for similarity. If the
        closest object's distance exceeds this threshold, the function
        defaults to returning the hand annotation. Defaults to 1.
    :return: A `SemanticAnnotation` object that is the most similar to
        the given hand annotation, or the original hand annotation if no
        suitable match is found within the threshold.
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
                break
        counter = 0
    # Apply threshold
    if best_distance > threshold or most_similar is None:
        return hand_annotation
    return most_similar
