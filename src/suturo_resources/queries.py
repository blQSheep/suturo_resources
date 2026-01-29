from krrood.entity_query_language.entity import variable, entity, contains
from krrood.entity_query_language.entity_result_processors import an
from semantic_digital_twin.world_description.world_entity import Region, Body


def query_kitchen_area(world):
    """
    Queries the kitchen area from the environment.
    Returns the center of mass and global pose of the kitchen region.
    """
    body = variable(type_=Body, domain=world.bodies)
    query = an(entity(body).where(contains(body.name.name, "kitchen")))
    kitchen_room_area = list(query.evaluate())[0]
    return kitchen_room_area

def query_living_room_area(world):
    """
    Queries the living room area.
    Returns the center of mass and global pose of the living room region.
    """
    body = variable(type_=Body, domain=world.bodies)
    query = an(entity(body).where(contains(body.name.name, "living_room")))
    living_room_area = list(query.evaluate())[0]
    return living_room_area

def query_bed_room_area(world):
    """
    Queries the bedroom area.
    Returns the center of mass and global pose of the bedroom region.
    """
    body = variable(type_=Body, domain=world.bodies)
    query = an(entity(body).where(contains(body.name.name, "bed_room")))
    bed_room_area = list(query.evaluate())[0]
    return bed_room_area

def query_office_area(world):
    """
    Queries the office area.
    Returns the center of mass and global pose of the office region.
    """
    body = variable(type_=Body, domain=world.bodies)
    query = an(entity(body).where(contains(body.name.name, "office")))
    office_area = list(query.evaluate())[0]
    return office_area

def query_trash(world):
    """
    Queries the location of the trash can in the environment.
    Returns the x, y, z coordinates of the trash can's global pose.
    """
    body = variable(type_=Body, domain=world.bodies)
    query = an(entity(body).where(contains(body.name.name, "trash_can_body")))
    trash_can = list(query.evaluate())[0]
    return trash_can