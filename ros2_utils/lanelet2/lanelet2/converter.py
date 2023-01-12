from ros2_utils.lanelet2.lanelet2.models import (Lanelet, Lanelet2, LineString,
                                                 Parameter, Point,
                                                 RegulatoryElement)
from ros2_utils.lanelet2.lanelet2.network import create_network


def convert_to_lanelet2(osm_data, left_hand_traffic):
    """
    Convert to lanelet2 from osm
    :param osm_data:
    :return: lanelet2_data
    """

    # Create lanelet2 data
    lanelet2_data = Lanelet2(version=osm_data.version)
    # Convert Point of Lanelet2 from node of OSM
    lanelet2_data.points = convert_points(osm_data.nodes)
    # Convert Linestring of Lanelet2 from way of OSM
    lanelet2_data.line_strings = convert_line_strings(
        osm_data.ways, lanelet2_data.points
    )
    # Convert Lanelet of Lanelet2 from relation of OSM
    lanelet2_data.lanelets, lanelet_regulatory_element_relations = convert_lanelets(
        osm_data.relations, lanelet2_data.line_strings
    )
    # Convert regulatory_element of Lanelet2 from relation of OSM
    lanelet2_data.regulatory_elements = convert_regulatory_elements(
        osm_data.relations, lanelet2_data.line_strings, lanelet2_data.lanelets
    )
    # Add regulatory elements to lanelet
    lanelet2_data.lanelets = add_regulatory_element_to_lanelet(
        lanelet2_data.lanelets,
        lanelet_regulatory_element_relations,
        lanelet2_data.regulatory_elements,
    )
    # validate
    lanelet2_data.validate()

    # create line_string and lanelet network
    (
        lanelet2_data.line_string_relations,
        lanelet2_data.lanelet_relations,
    ) = create_network(osm_data, left_hand_traffic)

    return lanelet2_data


def convert_points(osm_nodes):
    ll2_points = {}
    for osm_node_id, osm_node in osm_nodes.items():
        ll2_points[osm_node_id] = Point(
            id=osm_node_id,
            lat=osm_node.lat,
            lon=osm_node.lon,
            ele=osm_node.ele,
            attributes=osm_node.tags,
        )
    return ll2_points


def convert_line_strings(osm_ways, ll2_points):
    ll2_line_strings = {}
    for osm_way_id, osm_way in osm_ways.items():
        # get points of line_string
        ll2_line_string_points = []
        for osm_way_node in osm_way.nodes:
            ll2_line_string_points.append(ll2_points[osm_way_node.ref])
        ll2_line_strings[osm_way_id] = LineString(
            id=osm_way_id, points=ll2_line_string_points, attributes=osm_way.tags
        )
    return ll2_line_strings


def convert_regulatory_elements(osm_relations, line_strings, lanelets):
    regulatory_elements = {}
    for osm_relation_id, osm_relation in osm_relations.items():
        is_regulatory_element, subtype, attributes, parameters = get_regulatory_element(
            osm_relation, line_strings, lanelets
        )
        if is_regulatory_element is False:
            continue
        regulatory_elements[osm_relation_id] = RegulatoryElement(
            id=osm_relation_id,
            subtype=subtype,
            attributes=attributes,
            parameters=parameters,
        )

    return regulatory_elements


def get_regulatory_element(osm_relation, line_strings, lanelets):
    # check type of relations
    is_regulatory_element = False
    subtype = None
    attributes = {}
    parameters = None

    # check regulatory_element
    for tag in osm_relation.tags:
        if tag.key == "type" and tag.value == "regulatory_element":
            is_regulatory_element = True
        elif tag.key == "subtype":
            subtype = tag.value
        else:
            attributes[tag.key] = tag.value

    # if not regulatory_element, return
    if is_regulatory_element is False:
        return is_regulatory_element, subtype, attributes, parameters

    parameters = Parameter()
    for member in osm_relation.members:
        ref = None
        if member.type == "way":
            ref = line_strings[member.ref]
        elif member.type == "relation":
            ref = lanelets[member.ref]
        else:
            print("Not Support refer type. ", member.type)

        if member.role == "refers":
            parameters.refers = ref
        elif member.role == "ref_line":
            parameters.ref_line = ref
        elif member.role == "right_of_way":
            parameters.right_of_way = ref
        elif member.role == "yield":
            parameters.param_yield = ref
        elif member.role == "cancels":
            parameters.cancels = ref
        elif member.role == "cancel_line":
            parameters.cancel_line = ref
        else:
            print("Not Support role type. ", member.role)
    parameters.validate()

    return is_regulatory_element, subtype, attributes, parameters


def is_lanelet(osm_relation):
    # check type of relations
    is_lanelet = False
    for tag in osm_relation.tags:
        if tag.key == "type" and tag.value == "lanelet":
            is_lanelet = True
    return is_lanelet


def convert_lanelets(osm_relations, ll2_line_strings):
    ll2_lanelets = {}
    lanelet_regulatory_element_relations = (
        {}
    )  # key: lanelet_id, value: [regulatory_element_id]
    for osm_relation_id, osm_relation in osm_relations.items():
        # check type of relations
        if is_lanelet(osm_relation) is False:
            continue
        left_bound = None
        right_bound = None
        for member in osm_relation.members:
            if member.role == "left" and member.type == "way":
                left_bound = ll2_line_strings[member.ref]
            elif member.role == "right" and member.type == "way":
                right_bound = ll2_line_strings[member.ref]
            elif member.role == "regulatory_element":
                if osm_relation_id not in lanelet_regulatory_element_relations:
                    lanelet_regulatory_element_relations[osm_relation_id] = []
                lanelet_regulatory_element_relations[osm_relation_id].append(member.ref)

        ll2_lanelets[osm_relation_id] = Lanelet(
            id=osm_relation_id,
            left_bound=left_bound,
            right_bound=right_bound,
            center_line=None,
            attributes=osm_relation.tags,
            regulatory_element=[],
        )
    return ll2_lanelets, lanelet_regulatory_element_relations


def add_regulatory_element_to_lanelet(
    lanelets, lanelet_regulatory_element_relations, regulatory_elements
):
    for (
        lanelet_id,
        regulatory_element_ids,
    ) in lanelet_regulatory_element_relations.items():
        for regulatory_element_id in regulatory_element_ids:
            lanelets[lanelet_id].regulatory_element.append(
                regulatory_elements[regulatory_element_id]
            )
    return lanelets
