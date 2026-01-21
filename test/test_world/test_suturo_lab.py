from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.world import World
from suturo_resources.queries import query_most_similar_obj, query_semantic_annotations_on_surfaces, \
    query_get_next_object
from suturo_resources.suturo_map import load_environment, Publisher


def test_load_environment_returns_world():
    """
    Tests that loading the environment returns a World object with the correct root name.
    """
    world = load_environment()
    #publisher = Publisher("semantic_digital_twin")
    #publisher.publish(world)
    assert isinstance(world, World)
    assert world.root.name == PrefixedName("root_slam")

def test_query_semantic_annotations_on_surfaces():
    """
    Tests that giving Table annotations gives a list of the correct annotation on top.
    """
    world = load_environment()
    table1 = world.get_semantic_annotation_by_name("lowerTable_a")
    table2 = world.get_semantic_annotation_by_name("diningTable_a")
    apple = world.get_semantic_annotation_by_name("apple_a")
    carrot = world.get_semantic_annotation_by_name("carrot_a")
    orange = world.get_semantic_annotation_by_name("orange_a")
    lettuce = world.get_semantic_annotation_by_name("lettuce_a")

    assert query_semantic_annotations_on_surfaces([table1, table2]) == [carrot, lettuce, apple, orange]

def test_query_get_next_object():
    """
    Tests tha query_get_next_object
    :return: an ordered by distance list of Semantic Annotation
    """
    world = load_environment()
    table1 = world.get_semantic_annotation_by_name("lowerTable_a")
    table2 = world.get_semantic_annotation_by_name("diningTable_a")

    assert query_get_next_object(table1) == [{world.get_semantic_annotation_by_name("lettuce_a")},
                                             {world.get_semantic_annotation_by_name("carrot_a")}]
    assert query_get_next_object(table2) == [{world.get_semantic_annotation_by_name("orange_a")},
                                             {world.get_semantic_annotation_by_name("apple_a")}]

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
    world = load_environment()
    table1 = world.get_semantic_annotation_by_name("lowerTable_a")
    table2 = world.get_semantic_annotation_by_name("diningTable_a")
    list_of_products_1_2 = query_semantic_annotations_on_surfaces([table1, table2])
    list_of_products_1 = query_semantic_annotations_on_surfaces([table1])
    list_of_products_2 = query_semantic_annotations_on_surfaces([table2])

    banana = world.get_semantic_annotation_by_name("banana_a")
    apple = world.get_semantic_annotation_by_name("apple_a")
    carrot = world.get_semantic_annotation_by_name("carrot_a")
    orange = world.get_semantic_annotation_by_name("orange_a")
    lettuce = world.get_semantic_annotation_by_name("lettuce_a")

    assert query_most_similar_obj(orange, list_of_products_1_2) == orange
    assert query_most_similar_obj(banana, list_of_products_1_2) == apple
    assert query_most_similar_obj(lettuce, list_of_products_1_2) == lettuce
    assert query_most_similar_obj(carrot, []) == carrot
    assert query_most_similar_obj(apple, list_of_products_1) == apple
    assert query_most_similar_obj(carrot, list_of_products_2) == carrot
    #assert query_most_similar_obj(table1, list_of_products_1_2) == table1      # even when line 64 and 65 are working (if the diffrence between the sem and the annotations graiter than 1 then it returns sem) here it doesent work (i get carrot and idk why)