from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.world import World

from conftest import test_load_world
from suturo_resources.queries import (
    query_most_similar_obj,
    query_semantic_annotations_on_surfaces,
    query_get_next_object_euclidean_x_y,
)
from suturo_resources.suturo_map import load_environment


def test_load_environment_returns_world():
    """
    Tests that loading the environment returns a World object with the correct root name.
    """
    world = load_environment()
    assert isinstance(world, World)
    assert world.root.name == PrefixedName("root_slam")


def test_query_semantic_annotations_on_surfaces():
    """
    Tests that giving Table annotations gives a list of the correct annotation on top.
    """
    world = test_load_world()
    table1 = world.get_semantic_annotation_by_name("fruit_table_annotation")
    table2 = world.get_semantic_annotation_by_name("vegetable_table_annotation")
    table3 = world.get_semantic_annotation_by_name("empty_table_annotation")
    apple = world.get_semantic_annotation_by_name("apple_annotation")
    carrot = world.get_semantic_annotation_by_name("carrot_annotation")
    orange = world.get_semantic_annotation_by_name("orange_annotation")
    lettuce = world.get_semantic_annotation_by_name("lettuce_annotation")
    assert query_semantic_annotations_on_surfaces([table1, table2], world).tolist() == [
        apple,
        orange,
        carrot,
        lettuce,
    ]
    assert query_semantic_annotations_on_surfaces([table3], world).tolist() == []
    assert query_semantic_annotations_on_surfaces([], world).tolist() == []


def test_query_get_next_object_euclidean_x_y():
    """
    Tests the functionality of the `query_get_next_object_euclidean_x_y` function to verify that it accurately identifies
    the next objects based on their Euclidean proximity within a simulation world. The test involves setting up a virtual
    world, retrieving specific objects and annotations, and validating the results returned by the function against
    predetermined expectations.

    :raises AssertionError: If any of the function assertions fail during testing.
    """
    world = test_load_world()
    toya = world.get_body_by_name("base_link_body")
    table1 = world.get_semantic_annotation_by_name("fruit_table_annotation")
    table2 = world.get_semantic_annotation_by_name("vegetable_table_annotation")
    table3 = world.get_semantic_annotation_by_name("empty_table_annotation")
    apple = world.get_semantic_annotation_by_name("apple_annotation")
    carrot = world.get_semantic_annotation_by_name("carrot_annotation")
    orange = world.get_semantic_annotation_by_name("orange_annotation")
    lettuce = world.get_semantic_annotation_by_name("lettuce_annotation")

    assert query_get_next_object_euclidean_x_y(toya, table1).tolist() == [orange, apple]
    assert query_get_next_object_euclidean_x_y(toya, table2).tolist() == [
        carrot,
        lettuce,
    ]
    assert query_get_next_object_euclidean_x_y(toya, table3).tolist() == []


def test_query_most_similar_obj():
    """
    Test function for validating the behavior of querying the most similar object from
    a list of semantic annotations.

    The test involves loading a virtual environment, fetching semantic annotations
    of specific objects, creating lists of semantic annotations on specific surfaces,
    and verifying the functionality of the `query_most_similar_obj` function.

    This test ensures that the function correctly identifies and returns the most
    similar object from a given list. It includes assertions for both matching and
    non-matching cases, along with scenarios where the input list is empty.
    """
    world = test_load_world()
    table1 = world.get_semantic_annotation_by_name("fruit_table_annotation")
    table2 = world.get_semantic_annotation_by_name("vegetable_table_annotation")
    table3 = world.get_semantic_annotation_by_name("empty_table_annotation")
    list_of_products_1_2 = query_semantic_annotations_on_surfaces(
        [table1, table2], world
    ).tolist()  # has apple, orange, carrot and lettuce
    list_of_products_1 = query_semantic_annotations_on_surfaces(
        [table1], world
    ).tolist()  # has apple and orange
    list_of_products_2 = query_semantic_annotations_on_surfaces(
        [table2], world
    ).tolist()  # has carrot and lettuce
    list_of_products_3 = query_semantic_annotations_on_surfaces(
        [table3], world
    ).tolist()  # empty

    banana = world.get_semantic_annotation_by_name("banana_annotation")
    apple = world.get_semantic_annotation_by_name("apple_annotation")
    carrot = world.get_semantic_annotation_by_name("carrot_annotation")
    orange = world.get_semantic_annotation_by_name("orange_annotation")
    lettuce = world.get_semantic_annotation_by_name("lettuce_annotation")

    assert query_most_similar_obj(orange, list_of_products_1_2) == orange
    assert query_most_similar_obj(banana, list_of_products_1_2) == apple
    assert query_most_similar_obj(lettuce, list_of_products_1_2) == lettuce
    assert query_most_similar_obj(carrot, list_of_products_3) == carrot
    assert query_most_similar_obj(apple, list_of_products_2) == apple
    assert query_most_similar_obj(carrot, list_of_products_1) == carrot
    assert query_most_similar_obj(table1, list_of_products_1_2) == table1
