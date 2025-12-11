from krrood.entity_query_language.entity import let, entity, contains
from krrood.entity_query_language.quantify_entity import an
from semantic_digital_twin.world_description.world_entity import Region


def query_kitchen_area(world):
    """
    Queries the kitchen area from the environment.
    Returns the center of mass and global pose of the kitchen region.
    """
    body = let(type_=Region, domain=world.regions)
    query = an(entity(body, contains(body.name.name, "kitchen")))
    kitchen_room_area = list(query.evaluate())[0]
    return [float(kitchen_room_area.global_pose.x.to_np()[0]), float(kitchen_room_area.global_pose.y.to_np()[0]), float(kitchen_room_area.global_pose.z.to_np()[0])]

def query_living_room_area(world):
    """
    Queries the living room area.
    Returns the center of mass and global pose of the living room region.
    """
    body = let(type_=Region, domain=world.regions)
    query = an(entity(body, contains(body.name.name, "living_room")))
    living_room_area = list(query.evaluate())[0]
    return [float(living_room_area.global_pose.x.to_np()[0]), float(living_room_area.global_pose.y.to_np()[0]), float(living_room_area.global_pose.z.to_np()[0])]

def query_bed_room_area(world):
    """
    Queries the bedroom area.
    Returns the center of mass and global pose of the bedroom region.
    """
    body = let(type_=Region, domain=world.regions)
    query = an(entity(body, contains(body.name.name, "bed_room")))
    bed_room_area = list(query.evaluate())[0]
    return [float(bed_room_area.global_pose.x.to_np()[0]), float(bed_room_area.global_pose.y.to_np()[0]), float(bed_room_area.global_pose.z.to_np()[0])]

def query_office_area(world):
    """
    Queries the office area.
    Returns the center of mass and global pose of the office region.
    """
    body = let(type_=Region, domain=world.regions)
    query = an(entity(body, contains(body.name.name, "office")))
    office_area = list(query.evaluate())[0]
    return [float(office_area.global_pose.x.to_np()[0]), float(office_area.global_pose.y.to_np()[0]), float(office_area.global_pose.z.to_np()[0])]