from ros2_utils.lanelet2.exceptions import (InvalidFileFormatError,
                                            InvalidMapDateError,
                                            MissingResourceError)
from ros2_utils.lanelet2.lanelet2.models import (LaneletRelation,
                                                 LineStringRelation)


def create_network(osm_data, left_hand_traffic):
    # create line_string relations
    line_string_relations = create_line_string_network(osm_data)
    # validate line_string network
    ret = validate_line_string_network(line_string_relations, osm_data)

    # create lanenet relations
    lanelet_relations = create_lanelet_network(
        osm_data, line_string_relations, left_hand_traffic
    )
    # validate lanelet network
    ret = validate_lanelet_network(lanelet_relations)

    return line_string_relations, lanelet_relations


def create_line_string_network(osm_data):
    """
    Create line string connection network
    :param osm_data:
    :return line_string_relations:
    """

    line_string_relations = {}
    way_node_relation = {}  # {"way_id": {"start": "node_id", "end": "node_id"}}
    way_start_node = {}  # {"start_node_id": ["way_id"]}
    way_end_node = {}  # {"end_nd_id": ["way_id"]}

    for way_id, way in osm_data.ways.items():
        if len(way.nodes) < 2:
            print("[WARN] Way " + str(way_id) + " have no node is exist.")
            continue
        way_start_node_id = way.nodes[0].ref
        way_end_node_id = way.nodes[-1].ref
        way_node_relation[way_id] = {
            "start_node_id": way_start_node_id,
            "end_node_id": way_end_node_id,
        }
        if way_start_node_id not in way_start_node:
            way_start_node[way_start_node_id] = []
        way_start_node[way_start_node_id].append(way_id)
        if way_end_node_id not in way_end_node:
            way_end_node[way_end_node_id] = []
        way_end_node[way_end_node_id].append(way_id)

    # create line_string network
    for way_id, way_node_id in way_node_relation.items():
        line_string_relations[way_id] = LineStringRelation(
            prev_ids=way_end_node[way_node_id["start_node_id"]]
            if way_node_id["start_node_id"] in way_end_node
            else [],
            next_ids=way_start_node[way_node_id["end_node_id"]]
            if way_node_id["end_node_id"] in way_start_node
            else [],
        )
    return line_string_relations


def create_lanelet_network(osm_data, line_string_relations, left_hand_traffic):
    """
    Create lanelet connection network
    :param osm_data:
    :param line_string_relations:
    :return lanelet_relations:
    """
    lanelet_relations = {}
    lanelet_way_relation = (
        {}
    )  # {lanelet_id: {"left_way_id": way_id, "right_way_id": way_id}}
    way_lanelet_relation = {}  # {(left_way_id or right_way_id): lanelet_id}
    way_LR_lanelet_relation = (
        {}
    )  # {way_id: {"left_lanelet_id": [lanelet_id], "right_lanelet_id": [lanelet_id]}}

    # (way_id) => lanelet_id の関係を算出
    for relation_id, relation in osm_data.relations.items():
        is_lanelet = False
        # check type of relations
        for tag in relation.tags:
            if tag.key == "type" and tag.value == "lanelet":
                is_lanelet = True
        # skip if type is not lanelet
        if is_lanelet is False:
            continue
        # create lanelet_way_relation
        if relation_id not in lanelet_way_relation:
            lanelet_way_relation[relation_id] = {
                "left_way_id": None,
                "right_way_id": None,
            }
        for member in relation.members:
            if member.ref not in way_LR_lanelet_relation:
                way_LR_lanelet_relation[member.ref] = {
                    "right_lanelet_id": [],
                    "left_lanelet_id": [],
                }
            if member.role == "left" and member.type == "way":
                lanelet_way_relation[relation_id]["left_way_id"] = member.ref
                way_LR_lanelet_relation[member.ref]["right_lanelet_id"].append(
                    relation_id
                )
            elif member.role == "right" and member.type == "way":
                lanelet_way_relation[relation_id]["right_way_id"] = member.ref
                way_LR_lanelet_relation[member.ref]["left_lanelet_id"].append(
                    relation_id
                )
        # raise error if left/right way id is none
        if (
            lanelet_way_relation[relation_id]["left_way_id"] == None
            or lanelet_way_relation[relation_id]["right_way_id"] == None
        ):
            raise InvalidMapDateError("Lanelet's left, right way are not found.")

        # create way lanelet relation
        if left_hand_traffic is True:
            way_lanelet_relation[
                lanelet_way_relation[relation_id]["left_way_id"]
            ] = relation_id
        else:
            way_lanelet_relation[
                lanelet_way_relation[relation_id]["right_way_id"]
            ] = relation_id

    # create lanelet network
    for lanelet_id, lanelet_way_id in lanelet_way_relation.items():
        if (
            lanelet_way_id["left_way_id"] not in line_string_relations
            or lanelet_way_id["right_way_id"] not in line_string_relations
        ):
            raise MissingResourceError("Way is not found.")

        target_line_string_relations = (
            line_string_relations[lanelet_way_id["left_way_id"]]
            if left_hand_traffic is True
            else line_string_relations[lanelet_way_id["right_way_id"]]
        )
        prev_ids = []
        next_ids = []
        left_ids = []
        right_ids = []
        # prev lanelet
        for prev_way_id in target_line_string_relations["prev_ids"]:
            if prev_way_id in way_lanelet_relation:
                prev_ids.append(way_lanelet_relation[prev_way_id])
        # next lanelet
        for next_way_id in target_line_string_relations["next_ids"]:
            if next_way_id in way_lanelet_relation:
                next_ids.append(way_lanelet_relation[next_way_id])
        # left lanelet
        if "right_lanelet_id" in way_LR_lanelet_relation[lanelet_way_id["left_way_id"]]:
            left_ids = way_LR_lanelet_relation[lanelet_way_id["left_way_id"]][
                "left_lanelet_id"
            ]
        if "left_lanelet_id" in way_LR_lanelet_relation[lanelet_way_id["right_way_id"]]:
            right_ids = way_LR_lanelet_relation[lanelet_way_id["right_way_id"]][
                "right_lanelet_id"
            ]
        # set lanelet relation
        lanelet_relations[lanelet_id] = LaneletRelation(
            prev_ids=prev_ids, next_ids=next_ids, left_ids=left_ids, right_ids=right_ids
        )
    return lanelet_relations


def validate_line_string_network(line_string_relations, osm_data):
    # check size
    # if len(line_string_relations.keys()) != len(osm_data.ways.keys()):
    #     raise InvalidMapDateError("Line string network size is miss match.")

    # check connection
    for way_id, prev_next_ids in line_string_relations.items():
        # prev
        for prev_way_id in prev_next_ids.prev_ids:
            # check exists
            if prev_way_id not in line_string_relations:
                raise InvalidMapDateError("Missing prev way id.")
            if way_id not in line_string_relations[prev_way_id].next_ids:
                raise InvalidMapDateError("Missing way connection.")
        # next
        for next_way_id in prev_next_ids.next_ids:
            # check exists
            if next_way_id not in line_string_relations:
                raise InvalidMapDateError("Missing next way id.")
            if way_id not in line_string_relations[next_way_id].prev_ids:
                raise InvalidMapDateError("Missing way connection.")
    return True


def validate_lanelet_network(lanelet_network):
    # check connection
    for lanelet_id, prev_next_ids in lanelet_network.items():
        # prev
        for prev_lanelet_id in prev_next_ids.prev_ids:
            # check exists
            if prev_lanelet_id not in lanelet_network:
                raise InvalidMapDateError("Missing prev lanelet id.")
            if lanelet_id not in lanelet_network[prev_lanelet_id].next_ids:
                raise InvalidMapDateError("Missing lanelet connection.")
        # next
        for next_lanelet_id in prev_next_ids.next_ids:
            # check exists
            if next_lanelet_id not in lanelet_network:
                raise InvalidMapDateError("Missing next lanelet id.")
            if lanelet_id not in lanelet_network[next_lanelet_id].prev_ids:
                raise InvalidMapDateError("Missing lanelet connection.")
    return True
