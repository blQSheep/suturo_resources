from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Banana,
    Apple,
    Orange,
    Carrot,
    Lettuce,
    Table,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.spatial_types.spatial_types import (
    HomogeneousTransformationMatrix,
)
from semantic_digital_twin.world_description.world_entity import Body
from semantic_digital_twin.world_description.connections import (
    FixedConnection,
)
from semantic_digital_twin.world_description.geometry import (
    Box,
    Scale,
    Sphere,
    Cylinder,
    Color,
)
from semantic_digital_twin.world_description.shape_collection import ShapeCollection


def test_load_world():
    world = World()
    red = Color(255, 0, 0)
    orangeC = Color(1, 0.647, 0)
    yellow = Color(255, 255, 0)
    green = Color(0, 1, 0)
    all_elements_connections = []
    all_elements_annotations = []
    root = Body(name=PrefixedName("root"))

    fruit_table = Box(scale=Scale(2, 2, 1))
    shape_geometry = ShapeCollection([fruit_table])
    fruit_table_body = Body(
        name=PrefixedName("fruit_table_body"),
        collision=shape_geometry,
        visual=shape_geometry,
    )
    fruit_table_annotation = Table(
        root=fruit_table_body, name=PrefixedName("fruit_table_annotation")
    )
    all_elements_annotations.append(fruit_table_annotation)

    root_C_fruit_table = FixedConnection(
        parent=root,
        child=fruit_table_body,
        parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
            x=1, y=1, z=0
        ),
    )
    all_elements_connections.append(root_C_fruit_table)

    vegetable_table = Box(scale=Scale(2, 2, 1))
    shape_geometry = ShapeCollection([vegetable_table])
    vegetable_table_body = Body(
        name=PrefixedName("vegetable_table_body"),
        collision=shape_geometry,
        visual=shape_geometry,
    )
    vegetable_table_annotation = Table(
        root=vegetable_table_body, name=PrefixedName("vegetable_table_annotation")
    )
    all_elements_annotations.append(vegetable_table_annotation)

    root_C_vegetable_table = FixedConnection(
        parent=root,
        child=vegetable_table_body,
        parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
            x=1, y=1, z=2
        ),
    )
    all_elements_connections.append(root_C_vegetable_table)

    empty_table = Box(scale=Scale(2, 2, 1))
    shape_geometry = ShapeCollection([empty_table])
    empty_table_body = Body(
        name=PrefixedName("empty_table_body"),
        collision=shape_geometry,
        visual=shape_geometry,
    )
    empty_table_annotation = Table(
        root=empty_table_body, name=PrefixedName("empty_table_annotation")
    )
    all_elements_annotations.append(empty_table_annotation)

    root_C_empty_table = FixedConnection(
        parent=root,
        child=empty_table_body,
        parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
            x=1, y=1, z=4
        ),
    )
    all_elements_connections.append(root_C_empty_table)

    apple = Sphere(radius=0.10, color=red)
    shape_geometry = ShapeCollection([apple])
    apple_body = Body(
        name=PrefixedName("apple_body"), collision=shape_geometry, visual=shape_geometry
    )
    apple_annotation = Apple(root=apple_body, name=PrefixedName("apple_annotation"))
    all_elements_annotations.append(apple_annotation)
    root_C_apple = FixedConnection(
        parent=root,
        child=apple_body,
        parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
            x=1, y=1, z=0.6
        ),
    )
    all_elements_connections.append(root_C_apple)

    orange = Sphere(radius=0.10, color=orangeC)
    shape_geometry = ShapeCollection([orange])
    orange_body = Body(
        name=PrefixedName("orange_body"),
        collision=shape_geometry,
        visual=shape_geometry,
    )
    orange_annotation = Orange(root=orange_body, name=PrefixedName("orange_annotation"))
    all_elements_annotations.append(orange_annotation)
    root_C_orange = FixedConnection(
        parent=root,
        child=orange_body,
        parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
            x=1, y=0.5, z=0.6
        ),
    )
    all_elements_connections.append(root_C_orange)

    carrot = Cylinder(width=0.05, height=0.20, color=orangeC)
    shape_geometry = ShapeCollection([carrot])
    carrot_body = Body(
        name=PrefixedName("carrot_body"),
        collision=shape_geometry,
        visual=shape_geometry,
    )
    carrot_annotation = Carrot(root=carrot_body, name=PrefixedName("carrot_annotation"))
    all_elements_annotations.append(carrot_annotation)
    root_C_carrot = FixedConnection(
        parent=root,
        child=carrot_body,
        parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
            x=1, y=1, z=2.6
        ),
    )
    all_elements_connections.append(root_C_carrot)

    lettuce = Box(scale=Scale(0.15, 0.15, 0.10), color=green)
    shape_geometry = ShapeCollection([lettuce])
    lettuce_body = Body(
        name=PrefixedName("lettuce_body"),
        collision=shape_geometry,
        visual=shape_geometry,
    )
    lettuce_annotation = Lettuce(
        root=lettuce_body, name=PrefixedName("lettuce_annotation")
    )
    all_elements_annotations.append(lettuce_annotation)
    root_C_lettuce = FixedConnection(
        parent=root,
        child=lettuce_body,
        parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
            x=1, y=1.5, z=2.55
        ),
    )
    all_elements_connections.append(root_C_lettuce)

    banana = Box(scale=Scale(0.20, 0.05, 0.05), color=yellow)
    shape_geometry = ShapeCollection([banana])
    banana_body = Body(
        name=PrefixedName("banana_body"),
        collision=shape_geometry,
        visual=shape_geometry,
    )
    banana_annotation = Banana(root=banana_body, name=PrefixedName("banana_annotation"))
    all_elements_annotations.append(banana_annotation)
    root_C_banana = FixedConnection(
        parent=root,
        child=banana_body,
        parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
            x=10, y=10, z=10
        ),
    )
    all_elements_connections.append(root_C_banana)

    toya = Cylinder(width=0.45, height=1.5)
    shape_geometry = ShapeCollection([toya])
    toya_body = Body(
        name=PrefixedName("base_link_body"),
        collision=shape_geometry,
        visual=shape_geometry,
    )

    root_C_toya = FixedConnection(
        parent=root,
        child=toya_body,
        parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(),
    )
    all_elements_connections.append(root_C_toya)

    with world.modify_world():
        for annotation in all_elements_annotations:
            world.add_semantic_annotation(annotation)
        for conn in all_elements_connections:
            world.add_connection(conn)
        return world
