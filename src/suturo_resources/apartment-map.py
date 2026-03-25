import threading

import numpy as np
import rclpy
from semantic_digital_twin.adapters.ros.visualization.viz_marker import VizMarkerPublisher
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName


import threading

import numpy as np
import rclpy
from semantic_digital_twin.adapters.ros.visualization.viz_marker import VizMarkerPublisher
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.semantic_annotations.semantic_annotations import Wall, Table, Cupboard, ShelfLayer
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Scale, Box, Color
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body


def build_apartment_map():
    world = World()
    root = Body(name=PrefixedName("root"))

    with world.modify_world():
        world.add_body(root)

    build_apartment_walls(world)
    build_apartment_furniture(world)

    return world


def build_apartment_walls(world: World):
    root = world.root

    slam_map_transformation = HomogeneousTransformationMatrix.from_xyz_rpy(
        x=0, y=0, yaw=0
    )

    with world.modify_world():
        world.add_body(root)
        south_wall1 = Wall.create_with_new_body_in_world(
            world=world,
            name=PrefixedName("south_wall1"),
            world_root_T_self=slam_map_transformation @ HomogeneousTransformationMatrix.from_xyz_rpy(
                x=-0.025, y=-1.375, yaw=0
            ),
            scale=Scale(0.05, 1.85, 3.00),
        )
        south_wall2 = Wall.create_with_new_body_in_world(
            world=world,
            name=PrefixedName("south_wall2"),
            world_root_T_self=slam_map_transformation @ HomogeneousTransformationMatrix.from_xyz_rpy(
                x=-0.025, y=4.365, yaw=0
            ),
            scale=Scale(0.05, 7.83, 3.00),
        )

        east_wall = Wall.create_with_new_body_in_world(
            world=world,
            name=PrefixedName("east_wall"),
            world_root_T_self=slam_map_transformation @ HomogeneousTransformationMatrix.from_xyz_rpy(
                x=2.485, y=-2.325, yaw=np.pi / 2
            ),
            scale=Scale(0.05, 4.97, 3.00),
        )

        north_wall = Wall.create_with_new_body_in_world(
            world=world,
            name=PrefixedName("north_wall"),
            world_root_T_self=slam_map_transformation @ HomogeneousTransformationMatrix.from_xyz_rpy(
                x=4.995, y=2.99
            ),
            scale=Scale(0.05, 10.58, 3.00),
        )

        west_wall = Wall.create_with_new_body_in_world(
            world=world,
            name=PrefixedName("west_wall"),
            world_root_T_self=slam_map_transformation @ HomogeneousTransformationMatrix.from_xyz_rpy(
                x=2.485, y=8.305, yaw=np.pi / 2
            ),
            scale=Scale(0.05, 4.97, 3.00),
        )

    return world


def build_apartment_furniture(world: World):
    root = world.root

    with world.modify_world():
        # 1. Create a cabinet (carcass).
        cabinet_scale = Scale(0.35, 0.85, 1.903)
        cabinet = Cupboard.create_with_new_body_in_world(
            name=PrefixedName("my_new_cabinet"),
            world=world,
            world_root_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
                x=0.15, y=2.37, z=cabinet_scale.z / 2, yaw=np.pi  # pose
            ),
            scale=cabinet_scale,
            wall_thickness=0.02,
        )

        # attach the cabinet to the room root so that the coordinates are relative to the room.
        cabinet_connection = cabinet.root.parent_connection
        world.remove_connection(cabinet_connection)
        cabinet_connection.parent = root
        world.add_connection(cabinet_connection)

        # 2. Create the physical bodies for the shelves
        shelf_thickness = 0.018  # 1.8 cm
        shelf_scale = Scale(0.33, 0.76, shelf_thickness)
        
        # Target surface heights from the floor in meters
        surface_heights = [0.291, 0.639, 0.956]
        cabinet_center_z = cabinet_scale.z / 2
        
        for i, target_surface_height in enumerate(surface_heights, start=1):
            shelf_geom = ShapeCollection([Box(scale=shelf_scale, color=Color.BEIGE())])
            shelf_body = Body(
                name=PrefixedName(f"shelf_body_{i}"),
                collision=shelf_geom,
                visual=shelf_geom,
            )
            shelf = ShelfLayer(root=shelf_body, name=PrefixedName(f"shelf_{i}"))

            # Calculate local z: (surface_height - half_thickness) - cabinet_center
            local_z = (target_surface_height - (shelf_thickness / 2)) - cabinet_center_z

            cabinet_C_shelf = FixedConnection(
                parent=cabinet.root,
                child=shelf_body,
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=0, y=0, z=local_z
                ),
            )
            world.add_connection(cabinet_C_shelf)
            world.add_semantic_annotation(shelf)
            cabinet.add_shelf_layer(shelf)
            
        # 3. Create two open-top storage boxes on the cabinet floor
        box_depth = 0.30
        box_width = 0.34
        box_height = 0.15
        box_wt = 0.01  # 1 cm thickness of the boxes
        box_color = Color.BEIGE()  # color for the boxes
        
        # Y-offsets, so that the boxes stand nicely next to each other.
        y_offsets = [-0.19, 0.19]
        
        for box_idx, y_off in enumerate(y_offsets, start=1):
            # Box base plate (root of the box)
            bottom_scale = Scale(box_depth, box_width, box_wt)
            bottom_geom = ShapeCollection([Box(scale=bottom_scale, color=box_color)])
            box_root = Body(name=PrefixedName(f"storage_box_{box_idx}_bottom"), collision=bottom_geom, visual=bottom_geom)

            # Place precisely on the cabinet floor
            # The inner floor area of the cabinet is -Z/2 plus the wall thickness (0.02)
            cabinet_inner_bottom_z = -(cabinet_scale.z / 2) + 0.02
            z_off = cabinet_inner_bottom_z + (box_wt / 2)
            
            world.add_connection(FixedConnection(
                parent=cabinet.root,
                child=box_root,
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(x=0, y=y_off, z=z_off)
            ))
            
            # box wall (placed on the base plate)
            wall_height = box_height - box_wt
            wall_z_off = (box_wt / 2) + (wall_height / 2)
            
            # front wall (-X)
            front_scale = Scale(box_wt, box_width, wall_height)
            front_geom = ShapeCollection([Box(scale=front_scale, color=box_color)])
            front_body = Body(name=PrefixedName(f"storage_box_{box_idx}_front"), collision=front_geom, visual=front_geom)
            world.add_connection(FixedConnection(
                parent=box_root, child=front_body,
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(x=-(box_depth / 2) + (box_wt / 2), y=0, z=wall_z_off)
            ))
            
            # back wall (+X)
            back_geom = ShapeCollection([Box(scale=front_scale, color=box_color)])
            back_body = Body(name=PrefixedName(f"storage_box_{box_idx}_back"), collision=back_geom, visual=back_geom)
            world.add_connection(FixedConnection(
                parent=box_root, child=back_body,
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(x=(box_depth / 2) - (box_wt / 2), y=0, z=wall_z_off)
            ))
            
            # left wall (-Y)
            side_scale = Scale(box_depth - 2*box_wt, box_wt, wall_height)
            left_geom = ShapeCollection([Box(scale=side_scale, color=box_color)])
            left_body = Body(name=PrefixedName(f"storage_box_{box_idx}_left"), collision=left_geom, visual=left_geom)
            world.add_connection(FixedConnection(
                parent=box_root, child=left_body,
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(x=0, y=-(box_width / 2) + (box_wt / 2), z=wall_z_off)
            ))
            
            # right wall (+Y)
            right_geom = ShapeCollection([Box(scale=side_scale, color=box_color)])
            right_body = Body(name=PrefixedName(f"storage_box_{box_idx}_right"), collision=right_geom, visual=right_geom)
            world.add_connection(FixedConnection(
                parent=box_root, child=right_body,
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(x=0, y=(box_width / 2) - (box_wt / 2), z=wall_z_off)
            ))

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
        viz = VizMarkerPublisher(_world=world, node=self.node)
        viz.with_tf_publisher()


publisher = Publisher("semantic_digital_twin")
publisher.publish(build_apartment_map())

# print(build_apartment_walls(world=World()).root.name)