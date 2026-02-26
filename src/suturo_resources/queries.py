import math
from typing import List, Union, Optional
from krrood.entity_query_language.entity import (
    entity,
    variable_from,
)
from krrood.entity_query_language.symbolic import QueryObjectDescriptor, Entity
from krrood.utils import inheritance_path_length
from semantic_digital_twin.reasoning.predicates import (
    is_supported_by,
    compute_euclidean_distance_2d,
    is_supporting,
)
from semantic_digital_twin.semantic_annotations.mixins import HasSupportingSurface
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Color

from semantic_digital_twin.world_description.world_entity import (
    Body,
    SemanticAnnotation,
)

from suturo_resources.suturo_map import load_environment
from enum import Enum


class ColorEnum(Enum):
    RED = Color(255, 0, 0)
    YELLOW = Color(255, 255, 0)
    GREEN = Color(0, 255, 0)
    CYAN = Color(0, 255, 255)
    BLUE = Color(0, 0, 255)
    MAGENTA = Color(255, 0, 255)
    WHITE = Color(255, 255, 255)
    BLACK = Color(0, 0, 0)
    GREY = Color(127, 127, 127)


def query_semantic_annotations_on_surfaces(
    supporting_surfaces: List[SemanticAnnotation], world: World
) -> Union[Entity[SemanticAnnotation], SemanticAnnotation]:
    """
    Queries a list of Semantic annotations that are on top of a given list of other annotations (ex. Tables).
    param: supporting_surfaces: List of SemanticAnnotations that are supporting other annotations.
    :param world: World object that contains the supporting_surfaces.
    return: List of SemanticAnnotations that are supported by the given supporting_surfaces.
    """
    supporting_surfaces_var = variable_from(supporting_surfaces)
    body_with_enabled_collision = variable_from(world.bodies_with_enabled_collision)
    semantic_annotations = variable_from(
        body_with_enabled_collision._semantic_annotations
    )
    semantic_annotations_that_are_supported = entity(semantic_annotations).where(
        is_supported_by(
            supported_body=body_with_enabled_collision,
            supporting_body=supporting_surfaces_var.bodies[0],
        )
    )
    return semantic_annotations_that_are_supported


def query_get_next_object_euclidean_x_y(
    main_body: Body,
    supporting_surface,
) -> QueryObjectDescriptor[SemanticAnnotation]:
    """
    Queries the next object based on Euclidean distance in x and y coordinates
    relative to the given main body and supporting surface. This function utilizes
    semantic annotations of objects and orders them by their Euclidean distances
    to the main body.

    :param main_body: The main body to which the Euclidean distance is computed.
    :param supporting_surface: The surface on which the semantic annotations
        of interest are queried.
    :return: A `QueryObjectDescriptor` containing semantic annotations ordered
        by Euclidean distance to the main body.
    """
    supported_semantic_annotations = query_semantic_annotations_on_surfaces(
        [supporting_surface], main_body._world
    )
    return supported_semantic_annotations.order_by(
        compute_euclidean_distance_2d(
            body1=supported_semantic_annotations.selected_variable.bodies[0],
            body2=main_body,
        )
    )


def query_surface_of_most_similar_obj(
    object_of_interest: SemanticAnnotation,
    supporting_surfaces: List[HasSupportingSurface],
    threshold: int = 1,
) -> Optional[HasSupportingSurface]:
    """
    Finds the most similar object to a given semantic annotation among a list of tables
    based on the inheritance path length. If the similarity does not meet the provided
    threshold, the method attempts to return the table that is not supporting any object.
    The similarity metric leverages the class hierarchy to compute distances.

    :param object_of_interest: The semantic annotation to compare.
    :param supporting_surfaces: A list of supporting surfaces semantic annotations to search on top of them for similar objects to the object_of_interest.
    :param threshold: The maximum acceptable inheritance path length to classify objects
                      as similar. Defaults to 1.
    :return: The semantic annotation of the most appropriate surface based on similarity
             metrics or the non-supporting table when no viable candidate is found, or None if there are no supporting surfaces.
    """
    if not supporting_surfaces:
        return None

    # Find the surface that is not supporting anything
    non_supporting_table = None
    for supporting_surface in supporting_surfaces:
        if not is_supporting(supporting_surface.bodies[0]):
            non_supporting_table = supporting_surface
            break

    # Query annotations on the surfaces of the tables
    objects = query_semantic_annotations_on_surfaces(
        supporting_surfaces, object_of_interest._world
    ).tolist()

    best_distance = math.inf
    most_similar = None

    # Iterate over each object to find the most similar based on inheritance path length
    for obj in objects:
        for cls in type(obj).__mro__:
            dist = inheritance_path_length(type(object_of_interest), cls)
            if dist is None:
                continue
            if dist < best_distance:
                best_distance = dist
                most_similar = obj
            break  # Once a match is found, no need to check further classes for this object

    # Apply threshold to determine if the match is acceptable
    if best_distance > threshold or most_similar is None:
        return non_supporting_table

    # Find the table supporting the most similar object
    for supporting_surface in supporting_surfaces:
        if is_supported_by(most_similar.bodies[0], supporting_surface.bodies[0]):
            return supporting_surface


# def query_body_by_color(color: str, world: World) -> [Body]:
#     """
#     Queries bodies in the world that have a specific color. This function assumes that the color information is stored as a semantic annotation on the bodies.
#
#     :param color: The color to query for (e.g., "red", "blue").
#     :param world: The world object containing the bodies and their annotations.
#     :return: A containing bodies that match the specified color.
#     """
#     bodies = variable_from(world.bodies)
#     colored_bodies = entity(bodies).where(
#         lambda body: any(
#             annotation.color == color
#             for annotation in body._semantic_annotations
#             if hasattr(annotation, "color")
#         )
#     )
#     return colored_bodies
#
#
# print(query_body_by_color("red", load_environment()))


# def query_bodies_by_color(color: str, world: World) -> List[Body]:
#     """
#     Retrieves all bodies in the world with a specific color.
#
#     :param color: The color to filter bodies by (e.g., "red").
#     :param world: The World object containing bodies.
#     :return: List of bodies matching the specified color.
#     """
#     bodies = variable_from(world.bodies)
#     colored_bodies = entity(bodies).where(
#         lambda body: any(
#             getattr(annotation, "color", None) == color
#             for annotation in body._semantic_annotations
#         )
#     )
#     return colored_bodies


def query_bodies_by_color(color: Color, world: World) -> List[Body]:
    all_bodies = world.bodies
    filtered_bodies = []

    for body in all_bodies:
        if not body.visual.shapes:
            continue
        if body.visual.shapes[0].color == color:
            filtered_bodies.append(body)
    return filtered_bodies


print(query_bodies_by_color(ColorEnum.WHITE.value, load_environment()))
# print(load_environment().get_body_by_name("sofa_body").visual.shapes[0].color)
# print(load_environment().get_body_by_name("cup").visual.shapes[0].color)

# print(load_environment().get_body_by_name("sofa_body").visual.shapes[0].color)
print(ColorEnum.WHITE.value)


"""
    
    {
      "0": "apple",
      "1": "banana",
      "2": "baseball",
      "3": "bleachcleanser_softscrub_bottle",
      "4": "bowl_collapsable_greengrey",
      "5": "bowl_collapsable_redgrey",
      "6": "bowl_collapsable_yellowgrey",
      "7": "bowl_cone_red_plastic",
      "8": "bowl_red_metal",
      "9": "bowl_redwhite_plastic",
      "10": "buttermilk_mueller_bottle_original",
      "11": "buttermilk_mueller_bottle_raspberry",
      "12": "candle_tealight",
      "13": "candy_autodrop_box_cadillacs",
      "14": "candy_autodrop_box_totalloss",
      "15": "candy_tictac_box",
      "16": "cappuccinopowder_combo_can",
      "17": "cappuccinopowder_magico_can",
      "18": "cereal_jumbo_box_special",
      "19": "cereal_kellogs_box_original",
      "20": "cereal_nesquick_box_chocoballs",
      "21": "chips_feurich_can_paprika",
      "22": "chips_jumbo_bag_blue_paprika",
      "23": "chips_jumbo_bag_red_naturel",
      "24": "chips_pringles_can_original",
      "25": "chips_pringles_can_sourcreamonion",
      "26": "chips_pringles_can_saltvinegar",
      "27": "chips_stacked_can_original",
      "28": "clamp_black",
      "29": "soda_jumbo_bottle_cola",
      "30": "corn_bonduelle_can",
      "31": "cracker_cheezit_box_original",
      "32": "crispbread_wasa_pack_rosemaryseasalt",
      "33": "cup_small",
      "34": "cup_tigerpattern",
      "35": "dishwashertab",
      "36": "dishwashertab_somat_pack",
      "37": "fish_tuna_starkist_can",
      "38": "foambrick",
      "39": "fork_dinner_blackgrip",
      "40": "fork_dinner_redgrip",
      "41": "gelatine_jello_box_strawberry",
      "42": "glasscleaner_windex_spraybottle",
      "43": "golfball",
      "44": "grapes",
      "45": "groundcoffee_coop_pack",
      "46": "groundcoffee_masterchef_can",
      "47": "hammer_woodgrip",
      "48": "honeywafers_jumbo_pack",
      "49": "icetea_lipton_can_green",
      "50": "icetea_pfanner_pack_green",
      "51": "icetea_pfanner_pack_peach",
      "52": "juice_albi_pack_raspberrypassionfruit",
      "53": "juice_dubbelfrisss_box_apple",
      "54": "ketchup_hela_bottle_curry",
      "55": "knife_butter_blackgrip",
      "56": "knife_butter_redgrip",
      "57": "lemon",
      "58": "liquorice_jumbo_gemengdedrop",
      "59": "marker_black",
      "60": "mayonnaise_remia_bottle",
      "61": "milk_baerenmarke_pack",
      "62": "milk_hansano_pack",
      "63": "milk_ja_pack",
      "64": "milk_jumbo_pack_halfvoll",
      "65": "milk_jumbo_pack_voll",
      "66": "milk_milbona_pack",
      "67": "milk_weihenstephan_pack",
      "68": "mill_salt_transparent",
      "69": "muesli_koelln_box_honeynut",
      "70": "muesli_koelln_box_chocolatenutbrittle",
      "71": "mug_cylinder_blue",
      "72": "mug_grey",
      "73": "mug_highgrip_blue",
      "74": "mug_red_metal",
      "75": "mustard_frenchs_bottle",
      "76": "pasta_barilla_box_fusilli",
      "77": "orange",
      "78": "oregano_ostmann_shaker",
      "79": "pancakemix_koopmans_original",
      "80": "peach",
      "81": "pear",
      "82": "pitcher_blue",
      "83": "plate_red_metal",
      "84": "plum",
      "85": "pot_silver",
      "86": "pot_white",
      "87": "pottedmeat_spam_can",
      "88": "pudding_jello_box_chocolate",
      "89": "racquetball",
      "90": "rice_gutguenstig_pack_langkorn",
      "91": "rubikscube",
      "92": "salt_aquasale_can",
      "93": "salt_aquasale_mill_transparent",
      "94": "salt_gutguenstig_pack",
      "95": "sauce_knorr_basilikum",
      "96": "saucer_grey",
      "97": "sausage_jumbo_can",
      "98": "scissors_blackgrip",
      "99": "screwdriver_blackgreyredgrip",
      "100": "skillet_whitegold",
      "101": "soap_jumbo_bottle_almond",
      "102": "soccerball_mini",
      "103": "soda_cocacocla_can_zero",
      "104": "soda_fanta_can_zero",
      "105": "softball",
      "106": "sojadrink_biobio_pack",
      "107": "soup_jumbo_can",
      "108": "spatula_black",
      "109": "sponge",
      "110": "sponge_abrasive",
      "111": "spoon_dinner_blackgrip",
      "112": "spoon_dinner_redgrip",
      "113": "spoon_tea_metal",
      "114": "sprinkles_jumbo_melk",
      "115": "sprinkles_jumbo_puur",
      "116": "strawberry",
      "117": "sugar_domino_box",
      "118": "tennisball",
      "119": "tomatosoup_campbelli_can",
      "120": "vase_curved_blackblue",
      "121": "vase_cylinder_black",
      "122": "vegetables_gutguenstig_can",
      "123": "wafer_kinder_pack_bueno",
      "124": "water_jumbo_bottle",
      "125": "wineglass_shortstem",
      "126": "woodblock",
      "127": "muesli_koelln_box_cranberry",
      "128": "sponge_jumbo_pack",
      "129": "cucumber",
      "130": "zucchini",
      "131": "milk_milsani_pack_lactosefree",
      "132": "muesli_vitalis_box_nutmix",
      "133": "freezerbags",
      "134": "teabag_westminster_box",
      "135": "energy_redbull_can_white",
      "136": "candy_autodrop_cadillacs",
      "137": "candy_autodrop_totalloss",
      "138": "seasalt_hes_coarse",
      "139": "coffeefilter_bellarom_box",
      "140": "energy_drink_red_bull",
      "141": "muesli_crownfield_box_fruitoat",
      "142": "marshmallow_cream",
      "143": "tea_messmer_pack_mulledwine",
      "144": "spaghetti_classic",
      "145": "tangerine",
      "146": "trail_mix_alesto",
      "147": "tray_grey"
"""
