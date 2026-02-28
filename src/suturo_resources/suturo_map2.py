from typing import Optional, List, Literal
import rclpy
import threading
from dataclasses import dataclass

from semantic_digital_twin.world import World
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.spatial_types.spatial_types import (
    HomogeneousTransformationMatrix,
)
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Scale, Color
from semantic_digital_twin.world_description.world_entity import Body
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Cupboard,
    Door,
    Handle,
    Shelf,
    Floor,
    Room
)
from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
    VizMarkerPublisher,
)

# Farben
WHITE = Color(1, 1, 1)
WOOD = Color(1, 0.827, 0.6078)
GRAY = Color(0.74, 0.74, 0.74)


class CupboardFactory:
    """
    Eine Factory-Klasse, um komplexe Schränke mit Türen und Regalböden
    parametrisch zu erstellen.
    """

    @staticmethod
    def create_cupboard(
        world: World,
        name_prefix: str,
        position: HomogeneousTransformationMatrix,
        dimensions: Scale,
        wall_thickness: float = 0.02,
        num_shelves: int = 0,
        door_mode: Literal["none", "full", "bottom_half", "top_half"] = "none",
        color: Color = WHITE
    ) -> Cupboard:
        """
        Erstellt einen Cupboard mit optionalen Einbauten.

        :param world: Die Welt, in die der Schrank eingefügt wird.
        :param name_prefix: Prefix für die Namen der generierten Bodies.
        :param position: Die Pose des Schranks im World-Frame.
        :param dimensions: Scale(width, depth, height) des gesamten Schranks.
        :param wall_thickness: Dicke der Wände.
        :param num_shelves: Anzahl der Regalböden (werden gleichmäßig verteilt).
        :param door_mode: Konfiguration der Türen.
        :param color: Farbe des Schranks.
        """
        
        # 1. Korpus erstellen
        # Der Cupboard (als Cabinet) hat seine Öffnung standardmäßig in NEGATIVE_X Richtung.
        cupboard = Cupboard.create_with_new_body_in_world(
            name=PrefixedName(f"{name_prefix}_annotation"),
            world=world,
            world_root_T_self=position,
            scale=dimensions,
            wall_thickness=wall_thickness,
        )
        
        # Innenmaße berechnen
        inner_w = dimensions.y - (2 * wall_thickness)
        inner_h = dimensions.z - (2 * wall_thickness)
        inner_d = dimensions.x - wall_thickness # Rückwand abziehen

        # 2. Regalböden hinzufügen
        if num_shelves > 0:
            CupboardFactory._add_shelves(
                world, cupboard, name_prefix, num_shelves, 
                inner_w, inner_d, inner_h, wall_thickness
            )

        # 3. Türen hinzufügen
        if door_mode != "none":
            CupboardFactory._add_doors(
                world, cupboard, name_prefix, door_mode,
                dimensions, wall_thickness
            )

        return cupboard

    @staticmethod
    def _add_shelves(world, cupboard, prefix, count, width, depth, height, thickness):
        """Verteilt Regalböden gleichmäßig im Innenraum."""
        # Wir lassen oben und unten etwas Luft
        spacing = height / (count + 1)
        
        # Startpunkt (Z) ist der Boden des Innenraums. 
        # Der Ursprung des Schranks ist in der Mitte.
        # Boden ist bei -height/2 + thickness
        z_start = -(height + 2*thickness) / 2 + thickness

        shelf_scale = Scale(depth - 0.01, width - 0.005, thickness)

        for i in range(1, count + 1):
            z_pos = z_start + (i * spacing)
            
            # Position relativ zum Schrank-Ursprung
            # X-Pos: Die Rückwand ist bei +X/2. Die Öffnung bei -X/2.
            # Der Boden muss mittig in der Tiefe des Hohlraums sitzen.
            # Hohlraum geht von (X/2 - thickness) bis (-X/2).
            # Mitte Hohlraum = -thickness / 2
            
            shelf = Shelf.create_with_new_body_in_world(
                name=PrefixedName(f"{prefix}_shelf_{i}"),
                world=world,
                world_root_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=-thickness / 2, 
                    y=0, 
                    z=z_pos
                ),
                scale=shelf_scale
            )
            cupboard.add_shelf(shelf)

    @staticmethod
    def _add_doors(world, cupboard, prefix, mode, dims, thickness):
        """Fügt Türen basierend auf dem Modus hinzu."""
        
        door_h = dims.z
        z_offset = 0.0

        if mode == "bottom_half":
            door_h = dims.z / 2
            z_offset = -dims.z / 4
        elif mode == "top_half":
            door_h = dims.z / 2
            z_offset = dims.z / 4
        
        # Türen sind etwas kleiner als die Hälfte der Breite, damit sie nicht klemmen
        door_w = (dims.y / 2) - 0.002
        door_d = thickness
        
        # Position der Türen (Frontseite ist bei -dims.x / 2)
        # Wir setzen sie davor.
        door_x_pos = -(dims.x / 2) - (door_d / 2)

        # Linke Tür (bei +Y)
        door_l = Door.create_with_new_body_in_world(
            name=PrefixedName(f"{prefix}_door_left"),
            world=world,
            world_root_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
                x=door_x_pos,
                y=dims.y/4,
                z=z_offset
            ),
            scale=Scale(door_d, door_w, door_h)
        )
        
        # Rechte Tür (bei -Y)
        door_r = Door.create_with_new_body_in_world(
            name=PrefixedName(f"{prefix}_door_right"),
            world=world,
            world_root_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
                x=door_x_pos,
                y=-dims.y/4,
                z=z_offset
            ),
            scale=Scale(door_d, door_w, door_h)
        )

        # Griffe hinzufügen
        handle_scale = Scale(0.04, 0.02, 0.15)
        # Griff Position relativ zur Tür
        # Linke Tür: Griff rechts (negatives Y lokal zur Tür, da Tür Y-Achse global ist)
        # Moment, wir addieren Handles im World Frame relativ zur Türposition beim Erstellen.
        
        # Griff Links (sollte innen liegen, also bei y = +etwas relativ zur Tür-Mitte ist falsch, 
        # Tür Mitte ist bei +Y/4. Griff muss Richtung Mitte (0) zeigen -> also -Y relativ zur Tür)
        handle_l = Handle.create_with_new_body_in_world(
            name=PrefixedName(f"{prefix}_handle_l"),
            world=world,
            world_root_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
                x=door_x_pos - 0.02, # Etwas vor der Tür
                y=(dims.y/4) - (door_w/2) + 0.05, # Nahe der Kante zur Mitte hin
                z=z_offset,
                yaw=3.14159 # Umdrehen
            ),
            scale=handle_scale
        )
        door_l.add_handle(handle_l)
        
        handle_r = Handle.create_with_new_body_in_world(
            name=PrefixedName(f"{prefix}_handle_r"),
            world=world,
            world_root_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
                x=door_x_pos - 0.02,
                y=-(dims.y/4) + (door_w/2) - 0.05,
                z=z_offset,
                yaw=3.14159
            ),
            scale=handle_scale
        )
        door_r.add_handle(handle_r)

        cupboard.add_door(door_l)
        cupboard.add_door(door_r)


def load_environment():
    world = World()
    root = Body(name=PrefixedName("root"))
    
    # Ein Boden zur Orientierung
    floor_body = Body(name=PrefixedName("floor"), visual=ShapeCollection([
        # Box(Scale(10, 10, 0.1), color=GRAY)
    ]))
    with world.modify_world():
        world.add_connection(FixedConnection(world.root, root))
        
        # 1. Ein hoher Schrank mit Türen oben
        CupboardFactory.create_cupboard(
            world=world,
            name_prefix="tall_cupboard",
            position=HomogeneousTransformationMatrix.from_xyz_rpy(x=2.0, y=0.0, z=1.0),
            dimensions=Scale(0.5, 1.0, 2.0),
            num_shelves=3,
            door_mode="top_half"
        )

        # 2. Ein Sideboard mit vollen Türen
        CupboardFactory.create_cupboard(
            world=world,
            name_prefix="sideboard",
            position=HomogeneousTransformationMatrix.from_xyz_rpy(x=2.0, y=2.0, z=0.5),
            dimensions=Scale(0.6, 1.5, 1.0),
            num_shelves=1,
            door_mode="full"
        )

    return world

class Publisher:
    def __init__(self, name):
        self.context = rclpy.init()
        self.node = rclpy.create_node(name)
        self.thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.thread.start()

    def publish(self, world):
        viz = VizMarkerPublisher(world=world, node=self.node)
        viz.with_tf_publisher()