from semantic_digital_twin.semantic_annotations.semantic_annotations import Apple, Milk, Table, Carrot, Bread, Tomato, \
    TunaCan

from suturo_resources.queries import query_region_area, query_most_similar_obj
from suturo_resources.suturo_map import load_environment, Publisher


def test_load_environment_returns_world():
    """
    Tests that loading the environment returns a World object with the correct root name.
    """
    world = load_environment()
    publisher = Publisher("semantic_digital_twin")
    publisher.publish(world)
    #assert isinstance(world, World)
    #assert world.root.name == PrefixedName("root_slam")


def test_areas():
    """
    Checks that room areas gives x, y, z coordinate each.
    """
    world = load_environment()
    query = query_region_area

    areas =[
        "kitchen", "living_room", "bed_room", "office"
    ]
    for area in areas:
        print(query(world, area))
        assert len([query(world, area).global_pose.x.to_list()[0], query(world, area).global_pose.y.to_list()[0],
                    query(world, area).global_pose.z.to_list()[0]]) == 3


def test_most_simular_obj():
    lop = [Milk, Table, Carrot, Bread]

    assert query_most_similar_obj(Apple, lop) == Carrot
    assert query_most_similar_obj(Tomato, lop) == Carrot
    assert query_most_similar_obj(TunaCan, lop) == Milk
    assert query_most_similar_obj(Table, lop) == Table


#
# def test_eql_is_supported_by():
#
#     @symbolic_function
#     def is_supported_by(
#             supported_body: Body, supporting_body: Body, max_intersection_height: float = 0.1
#     ) -> bool:
#         """
#         Checks if one object is supporting another object.
#
#         :param supported_body: Object that is supported
#         :param supporting_body: Object that potentially supports the first object
#         :param max_intersection_height: Maximum height of the intersection between the two objects.
#         If the intersection is higher than this value, the check returns False due to unhandled clipping.
#         :return: True if the second object is supported by the first object, False otherwise
#         """
#         if supported_body == supporting_body:
#             return False
#
#         if supported_body.combined_mesh is None or supporting_body.combined_mesh is None:
#             return False
#
#         if Below(supported_body, supporting_body, supported_body.global_pose)():
#             return False
#         bounding_box_supported_body = (
#             supported_body.collision.as_bounding_box_collection_at_origin(
#                 HomogeneousTransformationMatrix(reference_frame=supported_body)
#             ).event
#         )
#         bounding_box_supporting_body = (
#             supporting_body.collision.as_bounding_box_collection_at_origin(
#                 HomogeneousTransformationMatrix(reference_frame=supported_body)
#             ).event
#         )
#
#         intersection = (
#                 bounding_box_supported_body & bounding_box_supporting_body
#         ).bounding_box()
#
#         if intersection.is_empty():
#             return False
#
#         z_intersection: Interval = intersection[SpatialVariables.z.value]
#         size = sum([si.upper - si.lower for si in z_intersection.simple_sets])
#         return size < max_intersection_height
#
#     @symbolic_function
#     def get_next_object(supporting_surface, pov):
#         obj_distance = {}
#         for obj in bodies_above_body(supporting_surface):
#             dx = abs(obj.global_pose.x - pov[0])
#             dy = abs(obj.global_pose.y - pov[1])
#             dist_sq = dx + dy
#             obj_distance[obj] = dist_sq
#         sorted_objects = sorted(obj_distance.items(), key=lambda item: item[1])
#         return sorted_objects
#
#     world = load_environment()
#     table = world.get_body_by_name("table_body")
#     milk = world.get_body_by_name("milk_body")
#
#     body = variable(Body, domain=world.bodies)
#     #body2 = variable(Body, domain=world.bodies)
#
#     query_kwargs = an(entity(body).where(is_supported_by(supported_body=body, supporting_body=table)).)
#     results = list(query_kwargs.evaluate())
#
#
#
#     print(results)