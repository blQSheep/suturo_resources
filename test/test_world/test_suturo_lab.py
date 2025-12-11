import pytest
from krrood.entity_query_language.entity import let, contains, entity
from krrood.entity_query_language.quantify_entity import an
from semantic_digital_twin.semantic_annotations.semantic_annotations import Floor, Room
from semantic_digital_twin.world import World
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.world_description.world_entity import Body, Region

from suturo_resources.queries import query_kitchen_area, query_living_room_area, query_bed_room_area, query_office_area
from suturo_resources.suturo_map import load_environment, build_environment_walls, build_environment_furniture

def test_load_environment_returns_world():
    world = load_environment()
    assert isinstance(world, World)
    assert world.root.name == PrefixedName("root")


def test_areas():
    """
    Checks that key room areas can be queried and have valid center and pose.
    """
    world = load_environment()

    # List of areas and their query functions
    area_queries = [
        ("kitchen", query_kitchen_area),
        ("living room", query_living_room_area),
        ("bedroom", query_bed_room_area),
        ("office", query_office_area),
    ]

    for area_name, query_func in area_queries:
        center, pose = query_func(world)
        assert center is not None, f"{area_name} center should not be None"
        assert pose is not None, f"{area_name} pose should not be None"