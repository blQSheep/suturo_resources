"""
Beispiele für die Verwendung der PerceivedObjectFactory API.
Zeigt wie Perception und Planning Teams damit arbeiten.
"""

from semantic_digital_twin.semantic_annotations.perceived_object_factory import (
    PerceivedObjectFactory,
    query_object_by_class,
    query_objects_by_class,
    get_all_perceived_objects,
    query_object_by_name,
)
from semantic_digital_twin.world_description.geometry import Scale
from semantic_digital_twin.world import World
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName


# =============================================================
# EXAMPLE 1: Einfache Objekterstellung
# =============================================================

def example_create_single_object():
    """
    Erstelle ein einzelnes wahrgenommenes Objekt (z.B. einen Apfel).
    Typical Perception Team Output.
    """
    # Perception Team gibt diese Daten zurück
    apple_factory = PerceivedObjectFactory(
        perceived_object_class="apple",
        object_dimensions=Scale(x=0.08, y=0.08, z=0.08)
    )
    
    # Erstelle die World mit dem Apfel
    apple_world = apple_factory.create()
    
    print(f"Created object world: {apple_world.name}")
    print(f"Bodies in world: {len(apple_world.bodies)}")
    print(f"Semantic annotations: {len(apple_world.semantic_annotations)}")
    
    return apple_world


# =============================================================
# EXAMPLE 2: Mehrere Objekte in einer World
# =============================================================

def example_create_multiple_objects():
    """
    Erstelle eine Scene mit mehreren wahrgenommenen Objekten.
    Planning Team kann damit arbeiten.
    """
    # Erstelle eine zentrale Scene World
    scene_world = World(name="kitchen_scene")
    
    # Objekt-Liste die vom Perception System kommt
    perceived_objects = [
        ("apple", Scale(x=0.08, y=0.08, z=0.08)),
        ("banana", Scale(x=0.02, y=0.02, z=0.12)),
        ("cup_small", Scale(x=0.08, y=0.08, z=0.10)),
        ("spoon_dinner_blackgrip", Scale(x=0.02, y=0.02, z=0.18)),
    ]
    
    # Erstelle Factory für jedes Objekt und füge zur Scene hinzu
    for object_class, dimensions in perceived_objects:
        factory = PerceivedObjectFactory(
            perceived_object_class=object_class,
            object_dimensions=dimensions,
            name=PrefixedName(f"{object_class}_instance_1")
        )
        
        object_world = factory.create()
        
        # Merge in die zentrale Scene
        with scene_world.modify_world():
            scene_world.merge_world(object_world)
    
    print(f"\nCreated scene with {len(scene_world.bodies)} bodies")
    print(f"Objects: {len(get_all_perceived_objects(scene_world))}")
    
    return scene_world


# =============================================================
# EXAMPLE 3: Objekt-Abfragen (für Planning Team)
# =============================================================

def example_query_objects(scene_world: World):
    """
    Zeige verschiedene Query-Möglichkeiten für das Planning System.
    """
    print("\n=== QUERY EXAMPLES ===")
    
    # Query 1: Nach Objektklasse suchen
    apple = query_object_by_class(scene_world, "apple")
    if apple:
        print(f"Found apple: {apple.name.name}")
        print(f"  Body: {apple.body.name}")
        print(f"  Collision type: {type(apple.body.collision)}")
    
    # Query 2: Alle Objekte einer Klasse
    all_objects = query_objects_by_class(scene_world, "apple")
    print(f"\nFound {len(all_objects)} apples")
    
    # Query 3: Alle wahrgenommenen Objekte
    all_perceived = get_all_perceived_objects(scene_world)
    print(f"\nTotal perceived objects in scene: {len(all_perceived)}")
    for obj in all_perceived:
        print(f"  - {obj.object_class}: {obj.name.name}")
    
    # Query 4: Nach Namen suchen
    cup = query_object_by_name(scene_world, "cup_small_instance_1")
    if cup:
        print(f"\nFound object by name: {cup.object_class}")


# =============================================================
# EXAMPLE 4: Integration mit anderen Semantic Annotations
# =============================================================

def example_mixed_scene():
    """
    Erstelle eine Scene mit Rooms, Containers UND Objekten.
    Zeigt wie die neue Factory sich in den bestehenden System integriert.
    """
    from semantic_digital_twin.semantic_annotations.factories import (
        RoomFactory,
        ContainerFactory,
    )
    from semantic_digital_twin.spatial_types.spatial_types import Point3, HomogeneousTransformationMatrix
    from semantic_digital_twin.world_description.connections import FixedConnection
    
    # Erstelle eine Room
    kitchen_factory = RoomFactory(
        name=PrefixedName("kitchen"),
        floor_polytope=[
            Point3(0, 0, 0),
            Point3(5, 0, 0),
            Point3(5, 3, 0),
            Point3(0, 3, 0),
        ]
    )
    kitchen_world = kitchen_factory.create()
    
    # Erstelle einen Container (z.B. eine Schüssel)
    bowl_factory = ContainerFactory(
        name=PrefixedName("bowl"),
        scale=Scale(x=0.25, y=0.20, z=0.10)
    )
    bowl_world = bowl_factory.create()
    
    # Erstelle wahrgenommene Objekte IN der Schüssel
    apple_factory = PerceivedObjectFactory(
        perceived_object_class="apple",
        object_dimensions=Scale(x=0.08, y=0.08, z=0.08),
        name=PrefixedName("apple_in_bowl")
    )
    apple_world = apple_factory.create()
    
    # Merge alles zusammen
    main_world = kitchen_world
    
    # Füge Schüssel zur Kitchen hinzu
    with main_world.modify_world():
        main_world.merge_world(bowl_world)
    
    # Füge Apfel zur Schüssel hinzu (mit Transformation)
    # Der Apfel sollte oben auf der Schüssel sein
    bowl_root = bowl_world.root
    apple_root = apple_world.root
    
    apple_T = HomogeneousTransformationMatrix.from_xyz_rpy(
        x=0.0, y=0.0, z=0.15,  # 15cm über der Schüssel
        roll=0, pitch=0, yaw=0
    )
    
    connection = FixedConnection(
        parent=bowl_root,
        child=apple_root,
        parent_T_connection_expression=apple_T
    )
    
    with main_world.modify_world():
        main_world.merge_world(apple_world, connection)
    
    print(f"\n=== MIXED SCENE ===")
    print(f"Room: {len(main_world.get_semantic_annotations_by_type(Room))}")
    print(f"Containers: {len(main_world.get_semantic_annotations_by_type(Container))}")
    print(f"Perceived Objects: {len(get_all_perceived_objects(main_world))}")
    
    return main_world


# =============================================================
# MAIN: Führe alle Beispiele aus
# =============================================================

if __name__ == "__main__":
    print("=== PERCEIVED OBJECT FACTORY EXAMPLES ===\n")
    
    # Example 1
    apple_world = example_create_single_object()
    
    # Example 2
    scene_world = example_create_multiple_objects()
    
    # Example 3
    example_query_objects(scene_world)
    
    # Example 4
    # mixed_world = example_mixed_scene()
    # (Nur auskommentiert, da es weitere Abhängigkeiten braucht)
    
    print("\n✅ All examples completed!")
