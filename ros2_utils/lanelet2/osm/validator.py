from ros2_utils.lanelet2.exceptions import (InvalidMapDateError,
                                            MissingResourceError)


def validate_osm_map(osm_data):
    # check exist of way's node
    for way_id, way in osm_data.ways.items():
        for node_ref in way.nodes:
            if node_ref.ref not in osm_data.nodes:
                raise MissingResourceError("Way's node is not found.")
    # check exist of relation's member
    for relation_id, relation in osm_data.relations.items():
        for member in relation.members:
            if member.type == "ways":
                if member.ref not in osm_data.ways:
                    raise MissingResourceError("Relation's way is not found.")
            if member.type == "relations":
                if member.ref not in osm_data.relations:
                    raise MissingResourceError("Relation's relation is not found.")
    return True
