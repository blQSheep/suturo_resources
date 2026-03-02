import numpy as np
from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
    VizMarkerPublisher,
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Table,
    Sofa,
    TrashCan,
    Fridge, Counter_Top, Wall, Cabinet,
)
from semantic_digital_twin.world import World
import threading
import rclpy
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.semantic_annotations.semantic_annotations import Room, Floor
from semantic_digital_twin.spatial_types.spatial_types import (
    HomogeneousTransformationMatrix,
    Point3,
)
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Box, Scale, Color
from semantic_digital_twin.world_description.geometry import Cylinder
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

def load_environment():
    """
    Initializes the world with a hierarchical scene graph containing walls, furniture, and room layouts.
    Returns the constructed World object representing the environment.
    """
    world = World()
    root_slam = Body(name=PrefixedName("root_slam"))
    root = Body(name=PrefixedName("root"))

    root_slam_C_root = FixedConnection(
        parent=root_slam,
        child=root,
        parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
            x=0.33, y=0.28, yaw=0.10707963267
        ),
    )
    with world.modify_world():
        world.add_connection(root_slam_C_root) # ToDelete
        # world.add_body(root)

    build_environment_walls(world)
    build_environment_furniture(world)
    build_environment_rooms(world)

    return world


def build_environment_walls(world: World):
    """
    Creates and connects all walls of the environment to the scene graph.
    The walls are represented as Body objects connected via FixedConnections.
    Returns the updated World object with walls integrated.
    """
    all_wall_connections = []
    root = world.get_body_by_name("root")

    south_wall1 = Box(scale=Scale(0.05, 1.00, 3.00))
    shape_geometry = ShapeCollection([south_wall1])
    south_wall1_body = Body(
        name=PrefixedName("south_wall1_body"),
        collision=shape_geometry,
        visual=shape_geometry,
    )

    root_C_south_wall1 = FixedConnection(
        parent=root,
        child=south_wall1_body,
        parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
            y=-2.01, z=1.50
        ),
    )
    all_wall_connections.append(root_C_south_wall1)

    south_wall2 = Box(scale=Scale(0.29, 0.05, 3.00))
    shape_geometry = ShapeCollection([south_wall2])
    south_wall2_body = Body(
        name=PrefixedName("south_wall2_body"),
        collision=shape_geometry,
        visual=shape_geometry,
    )

    root_C_south_wall2 = FixedConnection(
        parent=root,
        child=south_wall2_body,
        parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
            x=-0.145, y=-1.45, z=1.50
        ),
    )
    all_wall_connections.append(root_C_south_wall2)

    south_wall3 = Box(scale=Scale(0.05, 1.085, 1.00))
    shape_geometry = ShapeCollection([south_wall3])
    south_wall3_body = Body(
        name=PrefixedName("south_wall3_body"),
        collision=shape_geometry,
        visual=shape_geometry,
    )

    root_C_south_wall3 = FixedConnection(
        parent=root,
        child=south_wall3_body,
        parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
            x=-0.29, y=-0.9925, z=0.5
        ),
    )
    all_wall_connections.append(root_C_south_wall3)

    south_wall4 = Box(scale=Scale(0.29, 0.05, 1.00))
    shape_geometry = ShapeCollection([south_wall4])
    south_wall4_body = Body(
        name=PrefixedName("south_wall4_body"),
        collision=shape_geometry,
        visual=shape_geometry,
    )

    root_C_south_wall4 = FixedConnection(
        parent=root,
        child=south_wall4_body,
        parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
            x=-0.145, y=-0.45, z=0.5
        ),
    )
    all_wall_connections.append(root_C_south_wall4)

    south_wall5 = Box(scale=Scale(0.29, 0.05, 1.00))
    shape_geometry = ShapeCollection([south_wall5])
    south_wall5_body = Body(
        name=PrefixedName("south_wall5_body"),
        collision=shape_geometry,
        visual=shape_geometry,
    )

    root_C_south_wall5 = FixedConnection(
        parent=root,
        child=south_wall5_body,
        parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
            x=-0.145, y=0.45, z=0.5
        ),
    )
    all_wall_connections.append(root_C_south_wall5)

    south_wall6 = Box(scale=Scale(0.05, 2.75, 1.00))
    shape_geometry = ShapeCollection([south_wall6])
    south_wall6_body = Body(
        name=PrefixedName("south_wall6_body"),
        collision=shape_geometry,
        visual=shape_geometry,
    )

    root_C_south_wall6 = FixedConnection(
        parent=root,
        child=south_wall6_body,
        parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
            x=-0.29025, y=1.80, z=0.5
        ),
    )
    all_wall_connections.append(root_C_south_wall6)

    south_wall7 = Box(scale=Scale(0.05, 2.27, 1.00))
    shape_geometry = ShapeCollection([south_wall7])
    south_wall7_body = Body(
        name=PrefixedName("south_wall7_body"),
        collision=shape_geometry,
        visual=shape_geometry,
    )

    root_C_south_wall7 = FixedConnection(
        parent=root,
        child=south_wall7_body,
        parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
            x=-0.29025, y=5.16, z=0.5
        ),
    )
    all_wall_connections.append(root_C_south_wall7)

    east_wall = Box(scale=Scale(4.924, 0.05, 3.00))
    shape_geometry = ShapeCollection([east_wall])
    east_wall_body = Body(
        name=PrefixedName("east_wall_body"),
        collision=shape_geometry,
        visual=shape_geometry,
    )

    root_C_east_wall = FixedConnection(
        parent=root,
        child=east_wall_body,
        parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
            x=2.462, y=-2.535, z=1.50
        ),
    )
    all_wall_connections.append(root_C_east_wall)

    middle_wall = Box(scale=Scale(0.05, 2.67, 1.00))
    shape_geometry = ShapeCollection([middle_wall])
    middle_wall_body = Body(
        name=PrefixedName("middle_wall_body"),
        collision=shape_geometry,
        visual=shape_geometry,
    )

    root_C_middle_wall = FixedConnection(
        parent=root,
        child=middle_wall_body,
        parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
            x=2.20975, y=5.00, z=0.50
        ),
    )
    all_wall_connections.append(root_C_middle_wall)

    west_wall = Box(scale=Scale(4.449, 0.05, 3.00))
    shape_geometry = ShapeCollection([west_wall])
    west_wall_body = Body(
        name=PrefixedName("west_wall_body"),
        collision=shape_geometry,
        visual=shape_geometry,
    )

    root_C_west_wall = FixedConnection(
        parent=root,
        child=west_wall_body,
        parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
            x=1.9345, y=6.32, z=1.50
        ),
    )
    all_wall_connections.append(root_C_west_wall)

    north_wall = Box(scale=Scale(0.05, 8.04, 3.00))
    shape_geometry = ShapeCollection([north_wall])
    north_wall_body = Body(
        name=PrefixedName("north_wall_body"),
        collision=shape_geometry,
        visual=shape_geometry,
    )

    root_C_north_wall = FixedConnection(
        parent=root,
        child=north_wall_body,
        parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
            x=4.949, y=1.51, z=1.50
        ),
    )
    all_wall_connections.append(root_C_north_wall)

    north_west_wall = Cylinder(width=1.53, height=3.00)
    shape_geometry = ShapeCollection([north_west_wall])
    north_west_wall_body = Body(
        name=PrefixedName("north_west_wall_body"),
        collision=shape_geometry,
        visual=shape_geometry,
    )

    root_C_north_west_wall = FixedConnection(
        parent=root,
        child=north_west_wall_body,
        parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
            x=4.924, y=6.295, z=1.50
        ),
    )
    all_wall_connections.append(root_C_north_west_wall)

    with world.modify_world():
        for conn in all_wall_connections:
            world.add_connection(conn)
        return world


def build_environment_furniture(world: World):
    """
    Adds furniture items and room layouts (kitchen, living room, bedroom, office) to the scene graph.
    Connects furniture bodies and room structures hierarchically under the main root.
    Returns the updated World object with furniture integrated.
    """
    all_elements_connections = []
    all_elements_annotations = []
    root = world.get_body_by_name("root")

    root_transformation = HomogeneousTransformationMatrix.from_xyz_rpy(
        x=0.33, y=0.28, yaw=0.10707963267
    )

    trash_can = Cylinder(width=0.30, height=0.40, color=Color.BLACK())
    shape_geometry = ShapeCollection([trash_can])
    trash_can_body = Body(
        name=PrefixedName("trash_can_body"),
        collision=shape_geometry,
        visual=shape_geometry,
    )
    trash_can_annotation = TrashCan(
        root=trash_can_body, name=PrefixedName("trash_can_annotation")
    )
    all_elements_annotations.append(trash_can_annotation)

    root_C_trash_can = FixedConnection(
        parent=root,
        child=trash_can_body,
        parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
            x=0.416, y=5.5, z=0.20
        ),
    )
    all_elements_connections.append(root_C_trash_can)

    # refrigerator = Box(scale=Scale(0.60, 0.658, 1.49), color=Color.ORANGE())
    # shape_geometry = ShapeCollection([refrigerator])
    # refrigerator_body = Body(
    #     name=PrefixedName("refrigerator_body"),
    #     collision=shape_geometry,
    #     visual=shape_geometry,
    # )
    # refrigerator_annotation = Fridge(root=refrigerator_body, name=PrefixedName("refrigerator_annotation"))
    # all_elements_annotations.append(refrigerator_annotation)
    #
    # root_C_fridge = FixedConnection(
    #     parent=world.root,
    #     child=refrigerator_body,
    #     parent_T_connection_expression= root_transformation @ HomogeneousTransformationMatrix.from_xyz_rpy(
    #         x=0.537, y=-2.181, z=0.745
    #     ),
    # )
    # all_elements_connections.append(root_C_fridge)
    with world.modify_world():
        refrigerator = Fridge.create_with_new_body_in_world(
            world=world,
            name=PrefixedName("refrigerator"),
            world_root_T_self=root_transformation @ HomogeneousTransformationMatrix.from_xyz_rpy(
                 x=0.537, y=-2.181, z=0.745, yaw=np.pi*3/2),
            scale=Scale(x=0.60, y=0.658, z=1.49),
        )

    # counterTop = Box(scale=Scale(2.044, 0.658, 0.545), color=Color.BEIGE())
    # shape_geometry = ShapeCollection([counterTop])
    # counterTop_body = Body(
    #     name=PrefixedName("counterTop_body"),
    #     collision=shape_geometry,
    #     visual=shape_geometry,
    # )
    # counterTop_annotation = Counter_Top(root=counterTop_body, name=PrefixedName("counterTop_annotation"))
    # all_elements_annotations.append(counterTop_annotation)
    #
    # root_C_counterTop = FixedConnection(
    #     parent=root,
    #     child=counterTop_body,
    #     parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
    #         x=1.859, y=-2.181, z=0.2725
    #     ),
    # )
    # all_elements_connections.append(root_C_counterTop)
        counterTop = Counter_Top.create_with_new_body_in_world(
            world=world,
            name=PrefixedName("counterTop"),
            world_root_T_self= root_transformation @ HomogeneousTransformationMatrix.from_xyz_rpy(x=1.859, y=-2.181, z=0.2725),
            scale=Scale(x=2.044, y=0.658, z=0.545),
        )
        for color in counterTop.bodies[0].visual.shapes:
            color.color = Color.BEIGE()

    ovenArea = Box(scale=Scale(1.20, 0.658, 1.49))
    shape_geometry = ShapeCollection([ovenArea])
    ovenArea_body = Body(
        name=PrefixedName("ovenArea_body"),
        collision=shape_geometry,
        visual=shape_geometry,
    )

    root_C_ovenArea = FixedConnection(
        parent=root,
        child=ovenArea_body,
        parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
            x=3.481, y=-2.181, z=0.745
        ),
    )
    all_elements_connections.append(root_C_ovenArea)

    with world.modify_world():
        table = Table.create_with_new_body_in_world(
            world=world,
            name=PrefixedName("table"),
            world_root_T_self=root_transformation @ HomogeneousTransformationMatrix.from_xyz_rpy(x=3.545, y=0.426, z=0.4225),
            scale=Scale(x=2.45, y=0.796, z=0.845),
        )

        sofa = Sofa.create_with_new_body_in_world(
            world=world,
            name=PrefixedName("sofa"),
            world_root_T_self=root_transformation @ HomogeneousTransformationMatrix.from_xyz_rpy(x=3.60, y=1.20, z=0.34),
            scale=Scale(x=1.68, y=0.94, z=0.68),
        )
        for color in sofa.bodies[0].visual.shapes:
            color.color = Color.BEIGE()

        lowerTable = Table.create_with_new_body_in_world(
            world=world,
            name=PrefixedName("lowerTable"),
            world_root_T_self=root_transformation @ HomogeneousTransformationMatrix.from_xyz_rpy(x=4.22, y=2.22, z=0.22),
            scale=Scale(x=0.37, y=0.91, z=0.44),
        )

        cabinet = Cabinet.create_with_new_body_in_world(
            world=world,
            name=PrefixedName("cabinet"),
            world_root_T_self=root_transformation @ HomogeneousTransformationMatrix.from_xyz_rpy(x=4.8, y=4.72, z=1.01),
            scale=Scale(x=0.43, y=0.80, z=2.02),
        )

        desk = Table.create_with_new_body_in_world(
            world=world,
            name=PrefixedName("desk"),
            world_root_T_self=root_transformation @ HomogeneousTransformationMatrix.from_xyz_rpy(x=0.05, y=1.28, z=0.375),
            scale=Scale(x=0.60, y=1.20, z=0.75),
        )

        cooking_table = Table.create_with_new_body_in_world(
            world=world,
            name=PrefixedName("cooking_table"),
            world_root_T_self=root_transformation @ HomogeneousTransformationMatrix.from_xyz_rpy(x=1.325, y=5.99, z=0.355),
            scale=Scale(1.75, 0.64, 0.71),
        )
        for color in cooking_table.bodies[0].visual.shapes:
            color.color = Color.BEIGE()

        dinning_table = Table.create_with_new_body_in_world(
            world=world,
            name=PrefixedName("dining_table"),
            world_root_T_self=root_transformation @ HomogeneousTransformationMatrix.from_xyz_rpy(x=2.59975, y=5.705, z=0.365),
            scale=Scale(0.73, 1.18, 0.73),
        )
        for color in dinning_table.bodies[0].visual.shapes:
            color.color = Color.BEIGE()



        for annotation in all_elements_annotations:
            world.add_semantic_annotation(annotation)
        for conn in all_elements_connections:
            world.add_connection(conn)
    return world


def build_environment_rooms(world: World):

    room_annotations = []

    root_slam_T_root = world.get_body_by_name("root").parent_connection.origin

    with world.modify_world():
        kitchen_floor_polytope = [
            Point3(0, 0, 0),
            Point3(0, 3.334, 0),
            Point3(5.214, 3.334, 0),
            Point3(5.214, 0, 0),
        ]

        living_room_floor_polytope = [
            Point3(0, 0, 0),
            Point3(0, 2.971, 0),
            Point3(5.214, 2.971, 0),
            Point3(5.214, 0, 0),
        ]

        bed_room_floor_polytope = [
            Point3(0, 0, 0),
            Point3(0, 2.67, 0.0),
            Point3(2.50, 2.67, 0.0),
            Point3(2.50, 0, 0.0),
        ]

        office_floor_polytope = [
            Point3(0, 0, 0),
            Point3(0, 2.67, 0),
            Point3(2.71, 2.67, 0),
            Point3(2.71, 0, 0),
        ]

        kitchen_floor = Floor.create_with_new_body_from_polytope_in_world(
            name=PrefixedName("kitchen_floor"),
            world=world,
            floor_polytope=kitchen_floor_polytope,
            world_root_T_self=root_slam_T_root
            @ HomogeneousTransformationMatrix.from_xyz_rpy(x=2.317, y=-0.843),
        )
        kitchen = Room(floor=kitchen_floor, name=PrefixedName("kitchen"))
        room_annotations.append(kitchen)

        living_room_floor = Floor.create_with_new_body_from_polytope_in_world(
            name=PrefixedName("living_room_floor"),
            world=world,
            floor_polytope=living_room_floor_polytope,
            world_root_T_self=root_slam_T_root
            @ HomogeneousTransformationMatrix.from_xyz_rpy(x=2.317, y=2.3095),
        )
        living_room = Room(floor=living_room_floor, name=PrefixedName("living_room"))
        room_annotations.append(living_room)

        bed_room_floor = Floor.create_with_new_body_from_polytope_in_world(
            name=PrefixedName("bed_room_floor"),
            world=world,
            floor_polytope=bed_room_floor_polytope,
            world_root_T_self=root_slam_T_root
            @ HomogeneousTransformationMatrix.from_xyz_rpy(x=0.96, y=4.96),
        )
        bed_room = Room(floor=bed_room_floor, name=PrefixedName("bed_room"))
        room_annotations.append(bed_room)

        office_floor = Floor.create_with_new_body_from_polytope_in_world(
            name=PrefixedName("office_floor"),
            world=world,
            floor_polytope=office_floor_polytope,
            world_root_T_self=root_slam_T_root
            @ HomogeneousTransformationMatrix.from_xyz_rpy(x=3.56, y=4.96),
        )
        office = Room(floor=office_floor, name=PrefixedName("office"))
        room_annotations.append(office)

        world.add_semantic_annotations(room_annotations)

    return world


class Publisher:
    def __init__(self, name):
        self.context = rclpy.init()
        self.node = rclpy.create_node(name)
        self.thread = threading.Thread(
            target=rclpy.spin, args=(self.node,), daemon=True
        )
        self.thread.start()

    def publish(self, world):
        viz = VizMarkerPublisher(world=world, node=self.node)
        viz.with_tf_publisher()
