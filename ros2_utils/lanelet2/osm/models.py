from schematics.models import Model
from schematics.types import (BooleanType, DecimalType, FloatType, IntType,
                              StringType)


class OSM(Model):
    version = FloatType(required=True)
    nodes = {}
    ways = {}
    relations = {}

    def __init__(self, version):
        super(OSM, self).__init__()
        self.nodes = {}
        self.ways = {}
        self.relations = {}
        self.version = version
        self.validate()


class Node(Model):
    id = IntType(required=True)
    visible = BooleanType(default=True)
    lat = DecimalType(required=True)
    lon = DecimalType(required=True)
    ele = DecimalType(required=True)
    tags = []

    def __init__(self, id, lat, lon, tags, visible=True):
        super(Node, self).__init__()
        self.id = id
        self.lat = lat
        self.lon = lon
        self.ele = 0.0
        self.visible = visible
        self.tags = []
        for tag in tags:
            if tag["k"] == "ele":
                self.ele = tag["v"]
            else:
                self.tags.append(Tag(tag["k"], tag["v"]))
        self.validate()


class Way(Model):
    id = IntType(required=True)
    nodes = []
    tags = []
    visible = BooleanType(default=True)

    def __init__(self, id, nodes, tags, visible=True):
        super(Way, self).__init__()
        self.id = id
        self.nodes = []
        self.tags = []
        self.visible = visible
        # node
        for node in nodes:
            self.nodes.append(NodeRef(ref=node["ref"]))
        # tag
        for tag in tags:
            if tag["k"] == "ele":
                self.ele = tag["v"]
            else:
                self.tags.append(Tag(tag["k"], tag["v"]))
        self.validate()


class NodeRef(Model):
    ref = IntType(required=True)

    def __init__(self, ref):
        super(NodeRef, self).__init__()
        self.ref = ref
        self.validate()


class Relation(Model):
    id = IntType(required=True)
    members = []
    tags = []
    visible = BooleanType(default=True)

    def __init__(self, id, members, tags, visible=True):
        super(Relation, self).__init__()
        self.id = id
        self.members = []
        self.tags = []
        self.visible = visible
        # member
        for member in members:
            self.members.append(
                Member(type=member["type"], ref=member["ref"], role=member["role"])
            )
        # tag
        for tag in tags:
            if tag["k"] == "ele":
                self.ele = tag["v"]
            else:
                self.tags.append(Tag(tag["k"], tag["v"]))
        self.validate()


class Tag(Model):
    key = StringType(required=True)
    value = StringType(required=True)

    def __init__(self, key, value):
        super(Tag, self).__init__()
        self.key = key
        self.value = value
        self.validate()


class Member(Model):
    type = StringType(required=True)
    ref = IntType(required=True)
    role = StringType(required=True)

    def __init__(self, type, ref, role):
        super(Member, self).__init__()
        self.type = type
        self.ref = ref
        self.role = role
        self.validate()
