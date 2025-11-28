import pytest
from semantic_digital_twin.world import World
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from suturo_resources.suturo_map import load_environment, build_environment_walls, build_environment_furniture

def test_load_environment_returns_world():
    world = load_environment()
    assert isinstance(world, World)
    assert world.root.name == PrefixedName("root")