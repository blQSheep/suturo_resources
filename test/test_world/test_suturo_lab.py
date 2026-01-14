from semantic_digital_twin.world import World
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName

from suturo_resources.queries import query_kitchen_area, query_living_room_area, query_bed_room_area, query_office_area
from suturo_resources.suturo_map import load_environment

def test_load_environment_returns_world():
    """
    Tests that loading the environment returns a World object with the correct root name.
    """
    world = load_environment()
    assert isinstance(world, World)
    assert world.root.name == PrefixedName("root_slam")


def test_areas():
    """
    Checks that room areas gives x, y, z coordinate each.
    """
    world = load_environment()

    # List of areas and their query functions
    area_queries = [
        ("kitchen", query_kitchen_area),
        ("living_room", query_living_room_area),
        ("bedroom", query_bed_room_area),
        ("office", query_office_area),
    ]

    for place, query in area_queries:
        print(query(world))
        assert len([query(world).global_pose.x.to_list()[0], query(world).global_pose.y.to_list()[0], query(world).global_pose.z.to_list()[0]]) == 3