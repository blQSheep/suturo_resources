import logging
import os
from copy import deepcopy
from dataclasses import dataclass
from typing import Tuple

from pycram.datastructures.dataclasses import Context
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.semantic_annotations import Banana, Apple, Orange, Carrot, Lettuce, \
    Table
from semantic_digital_twin.world import World

from suturo_resources.suturo_map import load_environment

from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.spatial_types.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world_description.world_entity import Body
from semantic_digital_twin.world_description.connections import FixedConnection, OmniDrive
from semantic_digital_twin.world_description.geometry import Box, Scale, Sphere, Cylinder
from semantic_digital_twin.world_description.shape_collection import ShapeCollection

# logger = logging.getLogger(__name__)
#
#
# @dataclass(frozen=True)
# class WorldSetupPaths:
#     hsrb_urdf: str
#     milk_stl: str
#     cereal_stl: str
#
# def _here(*parts: str) -> str:
#     return os.path.abspath(os.path.join(os.path.dirname(__file__), *parts))
#
# def default_paths() -> WorldSetupPaths:
#     return WorldSetupPaths(
#         hsrb_urdf=_here("..", "..", "resources", "robots", "hsrb.urdf"),
#         milk_stl=_here("..", "..", "resources", "objects", "milk.stl"),
#         cereal_stl=_here("..", "..", "resources", "objects", "breakfast_cereal.stl"),
#     )
#
# def build_hsrb_world(hsrb_urdf: str):
#     world = URDFParser.from_file(file_path=hsrb_urdf).parse()
#     with world.modify_world():
#         odom = Body(name=PrefixedName("odom_combined"))
#         world.add_kinematic_structure_entity(odom)
#         world.add_connection(
#             OmniDrive.create_with_dofs(parent=odom, child=world.root, world=world)
#         )
#     return world
#
#
#
#
# def try_make_viz(world):
#     try:
#         import rclpy
#         from semantic_digital_twin.adapters.viz_marker import VizMarkerPublisher
#
#         node = rclpy.create_node("viz_marker")
#         return VizMarkerPublisher(world, node)
#     except Exception:
#         logger.info(
#             "VizMarkerPublisher is unavailable (ROS not running or deps missing)."
#         )
#         return None
#
#
# def hsr_world_setup():
#     hsr = ("/home/rody/SUTURO/Rody_cognitive_robot_abstract_machine/pycram/resources/robots/hsrb.urdf")
#     hsr_parser = URDFParser.from_file(file_path=hsr)
#     world_with_hsr = hsr_parser.parse()
#     HSRB.from_world(world_with_hsr)
#     with world_with_hsr.modify_world():
#         hsr_root = world_with_hsr.root
#         localization_body = Body(name=PrefixedName("odom_combined"))
#         world_with_hsr.add_kinematic_structure_entity(localization_body)
#         c_root_bf = OmniDrive.create_with_dofs(
#             parent=localization_body, child=hsr_root, world=world_with_hsr
#         )
#         world_with_hsr.add_connection(c_root_bf)
#
#     return world_with_hsr
#
# def test_hsr_world_setup():
#     world = hsr_world_setup()

def test_load_world():
    world = World()
    all_elements_connections = []
    all_elements_annotations = []
    root = Body(name=PrefixedName("root"))


    fruitTable = Box(scale=Scale(2, 2, 1))
    shape_geometry = ShapeCollection([fruitTable])
    fruitTable_body = Body(name=PrefixedName("fruitTable_body"), collision=shape_geometry, visual=shape_geometry)
    fruitTable_annotation = Table(body=fruitTable_body, name=PrefixedName("fruitTable_annotation"))
    all_elements_annotations.append(fruitTable_annotation)

    root_C_fruitTable = FixedConnection(parent=root, child=fruitTable_body,
                                   parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(x=1,
                                                                                                               y=1,
                                                                                                               z=0))
    all_elements_connections.append(root_C_fruitTable)

    vegetableTable = Box(scale=Scale(2,2,1))
    shape_geometry = ShapeCollection([vegetableTable])
    vegetableTable_body = Body(name=PrefixedName("vegetableTable_body"), collision=shape_geometry, visual=shape_geometry)
    vegetableTable_annotation = Table(body=vegetableTable_body, name=PrefixedName("vegetableTable_annotation"))
    all_elements_annotations.append(vegetableTable_annotation)

    root_C_vegetableTable = FixedConnection(parent=root, child=vegetableTable_body,
                                   parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(x=1,
                                                                                                               y=1,
                                                                                                               z=2))
    all_elements_connections.append(root_C_vegetableTable)

    emptyTable = Box(scale=Scale(2,2,1))
    shape_geometry = ShapeCollection([emptyTable])
    emptyTable_body = Body(name=PrefixedName("emptyTable_body"), collision=shape_geometry, visual=shape_geometry)
    emptyTable_annotation = Table(body=emptyTable_body, name=PrefixedName("emptyTable_annotation"))
    all_elements_annotations.append(emptyTable_annotation)

    root_C_emptyTable = FixedConnection(parent=root, child=emptyTable_body,
                                   parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(x=1,
                                                                                                               y=1,
                                                                                                               z=4))
    all_elements_connections.append(root_C_emptyTable)

    apple = Sphere(radius=0.10)
    shape_geometry = ShapeCollection([apple])
    apple_body = Body(name=PrefixedName("apple_body"), collision=shape_geometry, visual=shape_geometry)
    apple_annotation = Apple(body=apple_body, name=PrefixedName("apple_annotation"))
    all_elements_annotations.append(apple_annotation)
    root_C_apple = FixedConnection(parent=root, child=apple_body,
                                   parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(x=1,
                                                                                                               y=1,
                                                                                                               z=0.6))
    all_elements_connections.append(root_C_apple)

    orange = Sphere(radius=0.10)
    shape_geometry = ShapeCollection([orange])
    orange_body = Body(name=PrefixedName("orange_body"), collision=shape_geometry, visual=shape_geometry)
    orange_annotation = Orange(body=orange_body, name=PrefixedName("orange_annotation"))
    all_elements_annotations.append(orange_annotation)
    root_C_orange = FixedConnection(parent=root, child=orange_body,
                                    parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(x=1,
                                                                                                                y=1.5,
                                                                                                                z=0.6))
    all_elements_connections.append(root_C_orange)
    all_elements_connections.append(root_C_orange)

    carrot = Cylinder(width=0.05, height=0.20)
    shape_geometry = ShapeCollection([carrot])
    carrot_body = Body(name=PrefixedName("carrot_body"), collision=shape_geometry, visual=shape_geometry)
    carrot_annotation = Carrot(body=carrot_body, name=PrefixedName("carrot_annotation"))
    all_elements_annotations.append(carrot_annotation)
    root_C_carrot = FixedConnection(parent=root, child=carrot_body,
                                    parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(x=1,
                                                                                                                y=1,
                                                                                                                z=2.6))
    all_elements_connections.append(root_C_carrot)

    lettuce = Box(scale=Scale(0.15, 0.15, 0.10))
    shape_geometry = ShapeCollection([lettuce])
    lettuce_body = Body(name=PrefixedName("lettuce_body"), collision=shape_geometry, visual=shape_geometry)
    lettuce_annotation = Lettuce(body=lettuce_body, name=PrefixedName("lettuce_annotation"))
    all_elements_annotations.append(lettuce_annotation)
    root_C_lettuce = FixedConnection(parent=root, child=lettuce_body,
                                     parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(x=1,
                                                                                                                y=1.5,
                                                                                                                z=2.55))
    all_elements_connections.append(root_C_lettuce)

    banana = Box(scale=Scale(0.20, 0.05, 0.05))
    shape_geometry = ShapeCollection([banana])
    banana_body = Body(name=PrefixedName("banana_body"), collision=shape_geometry, visual=shape_geometry)
    banana_annotation = Banana(body=banana_body, name=PrefixedName("banana_annotation"))
    all_elements_annotations.append(banana_annotation)
    root_C_banana = FixedConnection(parent=root, child=banana_body,
                                    parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(x=10,
                                                                                                                y=10,
                                                                                                                z=10))
    all_elements_connections.append(root_C_banana)

    with world.modify_world():
        for annotation in all_elements_annotations:
            world.add_semantic_annotation(annotation)
        for conn in all_elements_connections:
            world.add_connection(conn)
        return world