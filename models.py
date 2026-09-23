"""Domain objects and a custom, bidirectional adjacency-list graph."""

from dataclasses import dataclass
from enum import Enum


class FlyInError(Exception):
    """Report an invalid input or simulation without a traceback."""


class ZoneType(str, Enum):
    """Define supported zone types."""

    NORMAL = "normal"
    BLOCKED = "blocked"
    RESTRICTED = "restricted"
    PRIORITY = "priority"


@dataclass(frozen=True)
class Zone:
    """Describe a zone, its coordinates and occupancy limit."""

    name: str
    x: int
    y: int
    kind: ZoneType = ZoneType.NORMAL
    color: str | None = None
    capacity: int = 1

    @property
    def cost(self) -> int:
        """Return the time required to enter this zone."""
        return 2 if self.kind is ZoneType.RESTRICTED else 1


@dataclass(frozen=True)
class Connection:
    """Represent a shared capacity in both travel directions."""

    first: str
    second: str
    capacity: int = 1

    @property
    def key(self) -> tuple[str, str]:
        """Return a direction-independent resource identifier."""
        first, second = sorted((self.first, self.second))
        return first, second

    @property
    def name(self) -> str:
        """Return the connection name in declaration order."""
        return f"{self.first}-{self.second}"

    def other(self, name: str) -> str:
        """Return the opposite endpoint of an incident connection."""
        return self.second if name == self.first else self.first


class Network:
    """Store zones and connections without a graph library."""

    def __init__(self, drones: int) -> None:
        """Initialize an empty map with a fleet size."""
        self.drones = drones
        self.zones: dict[str, Zone] = {}
        self.links: dict[tuple[str, str], Connection] = {}
        self.adjacency: dict[str, list[Connection]] = {}
        self.start = ""
        self.end = ""

    def add_zone(self, zone: Zone, role: str) -> None:
        """Add a unique zone and optionally designate a terminal."""
        if zone.name in self.zones:
            raise FlyInError(f"duplicate zone '{zone.name}'")
        if role == "start_hub" and self.start:
            raise FlyInError("duplicate start_hub")
        if role == "end_hub" and self.end:
            raise FlyInError("duplicate end_hub")
        self.zones[zone.name] = zone
        self.adjacency[zone.name] = []
        if role == "start_hub":
            self.start = zone.name
        elif role == "end_hub":
            self.end = zone.name

    def add_link(self, link: Connection) -> None:
        """Connect existing, distinct zones with a unique edge."""
        if link.first not in self.zones or link.second not in self.zones:
            raise FlyInError("connection references an undefined zone")
        if link.first == link.second:
            raise FlyInError("a connection must join two distinct zones")
        if link.key in self.links:
            raise FlyInError(f"duplicate connection '{link.name}'")
        self.links[link.key] = link
        self.adjacency[link.first].append(link)
        self.adjacency[link.second].append(link)

    def capacity(self, name: str) -> int:
        """Treat terminal zones as having room for the whole fleet."""
        if name in (self.start, self.end):
            return self.drones
        return self.zones[name].capacity

    def connection(self, first: str, second: str) -> Connection:
        """Look up the shared edge between two zones."""
        left, right = sorted((first, second))
        key = (left, right)
        if key not in self.links:
            raise FlyInError(
                f"zones '{first}' and '{second}' are not connected"
            )
        return self.links[key]


@dataclass(frozen=True)
class Route:
    """Store one drone's location at each turn, including turn zero."""

    drone_id: int
    positions: tuple[str, ...]


@dataclass
class Drone:
    """Track a drone at a zone or in mandatory one-turn transit."""

    drone_id: int
    location: str
    destination: str | None = None


@dataclass(frozen=True)
class Movement:
    """Describe an output token and its destination color."""

    drone_id: int
    location: str
    zone: str

    @property
    def token(self) -> str:
        """Format a movement according to the subject."""
        return f"D{self.drone_id}-{self.location}"


@dataclass(frozen=True)
class Turn:
    """Expose one validated turn and its resource usage to renderers."""

    number: int
    movements: tuple[Movement, ...]
    occupancy: dict[str, int]
    link_usage: dict[tuple[str, str], int]
    delivered: int
