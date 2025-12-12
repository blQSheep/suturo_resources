# myfactories.py

from dataclasses import dataclass, field
from semantic_digital_twin.world import World
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.world_description.geometry import Box, Scale, Color
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body
from semantic_digital_twin.world_description.connections import Connection6DoF, FixedConnection, RevoluteConnection
from semantic_digital_twin.spatial_types.spatial_types import TransformationMatrix

@dataclass
class TableFactory:
    name: PrefixedName
    top_scale: Scale
    leg_scale: Scale = field(default_factory=lambda: Scale(0.02, 0.02, 0.73))
    color: Color = field(default_factory=lambda: Color(1, 0.827, 0.6078))  # wood
    parent_T_top: TransformationMatrix = field(
        default_factory=lambda: TransformationMatrix.from_xyz_rpy(
            x=2.59975, y=5.705, z=0.72
        )
    )
    leg_positions: tuple = (
        (2.25, 5.125, 0.35),
        (2.25, 6.28, 0.35),
        (2.95, 5.125, 0.35),
        (2.95, 6.28, 0.35),
    )

    def create(self) -> World:
        world = World(name=self.name.name)

        with world.modify_world():
            root = Body(name=self.name)
            world.add_body(root)

            # Tischplatte
            tabletop = Box(scale=self.top_scale, color=self.color)
            tabletop_geom = ShapeCollection([tabletop])
            tabletop_body = Body(
                name=PrefixedName(f"{self.name.name}_top", self.name.prefix),
                collision=tabletop_geom,
                visual=tabletop_geom,
            )
            world.add_body(tabletop_body)
            conn_top = FixedConnection(
                parent=root,
                child=tabletop_body,
                parent_T_connection_expression=self.parent_T_top,
            )
            world.add_connection(conn_top)

            # Beine
            for i, (x, y, z) in enumerate(self.leg_positions, start=1):
                leg = Box(scale=self.leg_scale, color=self.color)
                leg_geom = ShapeCollection([leg])
                leg_body = Body(
                    name=PrefixedName(f"{self.name.name}_leg_{i}", self.name.prefix),
                    collision=leg_geom,
                    visual=leg_geom,
                )
                world.add_body(leg_body)
                conn_leg = FixedConnection(
                    parent=root,
                    child=leg_body,
                    parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(
                        x=x, y=y, z=z
                    ),
                )
                world.add_connection(conn_leg)

        return world