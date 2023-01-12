from ros2_utils.lanelet2.lanelet2.config import NodeLocalCoordinate
from schematics.models import Model
from schematics.types import (BaseType, BooleanType, DecimalType, FloatType,
                              IntType, ListType, ModelType, StringType)


class LaneletAttribute(Model):
    """
    Lanelet Attribute
    """

    type_of_lanelet = StringType(required=True)
    subtype = StringType(required=True)
    location = StringType(required=True)
    participants = StringType(required=True)
    one_way = BooleanType(required=True, default=False)
    speed_limit = FloatType(required=True)

    def __init__(
        self,
        type_of_lanelet,
        subtype,
        location,
        participants,
        one_way,
        turn_direction,
        speed_limit,
    ):
        super(LaneletAttribute, self).__init__()
        self.type_of_lanelet = type_of_lanelet
        self.subtype = subtype
        self.location = location
        self.participants = participants
        self.one_way = one_way
        self.turn_direction = turn_direction
        self.speed_limit = speed_limit
        self.validate()


class Attribute(Model):
    key = StringType(required=True)
    value = StringType(required=True)

    def __init__(self, key, value):
        super(Attribute, self).__init__()
        self.key = key
        self.value = value
        self.validate()


class Point(Model):
    """
    Point of Lanelet
    """

    id = IntType(required=True)
    lat = DecimalType(required=True)
    lon = DecimalType(required=True)
    ele = DecimalType(required=True)
    x = DecimalType(default=None)
    y = DecimalType(default=None)
    z = DecimalType(default=None)
    attributes = []

    def __init__(self, id, lat, lon, ele, attributes=[]):
        super(Point, self).__init__()
        self.id = id
        self.lat = lat
        self.lon = lon
        self.ele = ele
        self.x, self.y, self.z = None, None, ele
        self.attributes = []
        for attribute in attributes:
            # local cordinate
            if attribute.key == "local_x":
                self.x = attribute.value
            elif attribute.key == "local_y":
                self.y = attribute.value
            elif attribute.key == "local_z":
                self.z = attribute.value
            else:
                self.attributes.append(
                    Attribute(key=attribute.key, value=attribute.value)
                )
        self.validate()


class Lanelet2(Model):
    version = FloatType(required=True)
    points = {}
    line_strings = {}
    lanelets = {}
    areas = {}
    polygon = {}
    regulatory_elements = {}
    line_string_relations = {}
    lanelet_relations = {}

    def __init__(self, version):
        super(Lanelet2, self).__init__()
        self.version = version
        self.validate()


class LineString(Model):
    """
    LineString of Lanelet2
    """

    id = IntType(required=True)
    points = []
    attributes = []

    def __init__(self, id, points, attributes=[]):
        super(LineString, self).__init__()
        self.id = id
        self.points = points
        self.attributes = []
        for attribute in attributes:
            self.attributes.append(Attribute(key=attribute.key, value=attribute.value))
        self.validate()


class Lanelet(Model):
    """
    Lanelet
    """

    id = IntType(required=True)
    left_bound = IntType(required=True)
    right_bound = IntType(required=True)
    center_line = IntType(default=None)
    attribute = ModelType(LaneletAttribute)
    regulatory_element = []

    def __init__(
        self,
        id,
        left_bound,
        right_bound,
        center_line,
        attributes,
        regulatory_element=[],
    ):
        super(Lanelet, self).__init__()
        self.id = id
        self.left_bound = left_bound
        self.right_bound = right_bound
        self.center_line = center_line
        self.regulatory_element = regulatory_element
        # create attributes
        lanelet_attribute = {
            "type": None,
            "subtype": None,
            "location": "urban",
            "participants": "vehicle",
            "one_way": False,
            "speed_limit": 0.0,  # km/h
            "turn_direction": "straight",
        }
        for attribute in attributes:
            if attribute.key in lanelet_attribute:
                lanelet_attribute[attribute.key] = attribute.value
        self.attribute = LaneletAttribute(
            type_of_lanelet=lanelet_attribute["type"],
            subtype=lanelet_attribute["subtype"],
            location=lanelet_attribute["location"],
            participants=lanelet_attribute["participants"],
            one_way=True if lanelet_attribute["one_way"] == "yes" else False,
            turn_direction=lanelet_attribute["turn_direction"],
            speed_limit=lanelet_attribute["speed_limit"],
        )


class Parameter(Model):
    refers = BaseType(required=True)
    ref_line = BaseType(required=False)
    right_of_way = BaseType(required=False)
    param_yield = BaseType(required=False)
    cancels = BaseType(required=False)
    cancel_line = BaseType(required=False)

    def __init__(self):
        super(Parameter, self).__init__()
        self.refers = None
        self.ref_line = None
        self.param_yield = None
        self.cancels = None
        self.cancel_line = None


class RegulatoryElement(Model):
    """
    RegulatoryElement
    """

    id = IntType(required=True)
    subtype = StringType(required=True)
    attributes = []
    parameters = None

    def __init__(self, id, subtype, attributes={}, parameters=None):
        super(RegulatoryElement, self).__init__()
        self.id = id
        self.subtype = subtype
        self.attributes = []
        for key, value in attributes.items():
            self.attributes.append(Attribute(key=key, value=value))
        self.parameters = parameters
        self.validate()


class LineStringRelation(Model):
    """
    LineString Relation
    """

    prev_ids = ListType(IntType)
    next_ids = ListType(IntType)

    def __init__(self, prev_ids, next_ids):
        super(LineStringRelation, self).__init__()
        self.prev_ids = prev_ids
        self.next_ids = next_ids
        self.validate()


class LaneletRelation(Model):
    """
    Lanelet Relation
    """

    prev_ids = ListType(IntType)
    next_ids = ListType(IntType)
    left_ids = ListType(IntType)
    right_ids = ListType(IntType)

    def __init__(self, prev_ids, next_ids, left_ids, right_ids):
        super(LaneletRelation, self).__init__()
        self.prev_ids = prev_ids
        self.next_ids = next_ids
        self.left_ids = left_ids
        self.right_ids = right_ids
        self.validate()
