import xml.etree.ElementTree as ET

from ros2_utils.lanelet2.exceptions import (InvalidMapDateError,
                                            MissingResourceError)
from ros2_utils.lanelet2.lanelet2.converter import convert_to_lanelet2
from ros2_utils.lanelet2.osm.models import OSM, Node, Relation, Way
from ros2_utils.lanelet2.osm.validator import validate_osm_map


class Lanelet2Parser(object):
    def parse(self, src_file):
        """
        Load OSM file and convert to dict format
        :param src_file: osm_file
        :return: osm_data
        """
        # load xml file
        try:
            tree = ET.parse(src_file)
        except:
            raise MissingResourceError()

        # convert to dict format
        osm = tree.getroot()

        # osm data
        osm_data = OSM(
            version=osm.attrib["version"] if "version" in osm.attrib else 0.0
        )

        # node
        for osm_node in osm.iter("node"):
            # skip if action is delete
            if "action" in osm_node.attrib and osm_node.attrib["action"] == "delete":
                continue
            _osm_node = Node(
                id=osm_node.attrib["id"],
                lat=osm_node.attrib["lat"],
                lon=osm_node.attrib["lon"],
                tags=self._get_tags(osm_node),
                visible=True
                if "visible" not in osm_node.attrib
                or osm_node.attrib["visible"] == "true"
                else False,
            )
            if _osm_node.id not in osm_data.nodes:
                osm_data.nodes[_osm_node.id] = _osm_node
            else:
                raise InvalidMapDateError("Duplicated Entry for Node ID.")

        # way
        for osm_way in osm.iter("way"):
            # skip if action is delete
            if "action" in osm_way.attrib and osm_way.attrib["action"] == "delete":
                continue
            # nd
            way_nodes = []
            for way_nd in osm_way.findall("nd"):
                _way_nd = {"ref": way_nd.attrib["ref"]}
                way_nodes.append(_way_nd)
            _osm_way = Way(
                id=osm_way.attrib["id"],
                nodes=way_nodes,
                tags=self._get_tags(osm_way.findall("tag")),
                visible=True
                if "visible" not in osm_way.attrib
                or osm_way.attrib["visible"] == "true"
                else False,
            )
            if _osm_way.id not in osm_data.ways:
                osm_data.ways[_osm_way.id] = _osm_way
            else:
                raise InvalidMapDateError("Duplicated Entry for Way ID.")

        # relation
        for osm_relation in osm.iter("relation"):
            # skip if action is delete
            if (
                "action" in osm_relation.attrib
                and osm_relation.attrib["action"] == "delete"
            ):
                continue
            # member
            _relation_members = []
            for relation_member in osm_relation.findall("member"):
                _relation_member = {
                    "type": relation_member.attrib["type"],
                    "ref": int(relation_member.attrib["ref"]),
                    "role": relation_member.attrib["role"],
                }
                _relation_members.append(_relation_member)
            _osm_relation = Relation(
                id=osm_relation.attrib["id"],
                members=_relation_members,
                tags=self._get_tags(osm_relation.findall("tag")),
            )

            if _osm_relation.id not in osm_data.relations:
                osm_data.relations[_osm_relation.id] = _osm_relation
            else:
                raise InvalidMapDateError("Duplicated Entry for Relation ID.")
        # validate
        validate_osm_map(osm_data)
        return osm_data

    def convert_to_lanelet2(self, osm_data, left_hand_traffic=True):
        """
        Convert osm to lanelet2
        :param osm_data:
        :return: lanelet2_data
        """
        lanelet2_data = convert_to_lanelet2(osm_data, left_hand_traffic)

        return lanelet2_data

    def _get_tags(self, tags):
        _tags = []
        for tag in tags:
            _tag = {
                "k": tag.attrib["k"],
                "v": tag.attrib["v"],
            }
            _tags.append(_tag)
        return _tags
