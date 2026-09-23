"""Replay planned routes and independently validate every simulation turn."""

from collections import Counter
from collections.abc import Iterator

from models import Drone, FlyInError, Movement, Network, Route, Turn, ZoneType


class Simulation:
    """Apply simultaneous movements and enforce all runtime constraints."""

    def __init__(self, network: Network, routes: list[Route]) -> None:
        """Accept one complete route per uniquely identified drone."""
        self.network = network
        self.routes = routes
        expected = set(range(1, network.drones + 1))
        if len(routes) != network.drones or {
            route.drone_id for route in routes
        } != expected:
            raise FlyInError("expected exactly one route per drone")
        for route in routes:
            if (len(route.positions) < 2
                    or route.positions[0] != network.start
                    or route.positions[-1] != network.end):
                raise FlyInError("routes must connect start_hub to end_hub")
        self.drones = {identifier: Drone(identifier, network.start)
                       for identifier in expected}
        self.delivered = 0

    def turns(self) -> Iterator[Turn]:
        """Yield validated snapshots until the entire fleet is delivered."""
        self.drones = {route.drone_id: Drone(route.drone_id,
                                             self.network.start)
                       for route in self.routes}
        self.delivered = 0
        horizon = max(len(route.positions) for route in self.routes)
        for number in range(1, horizon):
            movements: list[Movement] = []
            usage: Counter[tuple[str, str]] = Counter()
            for route in self.routes:
                if number >= len(route.positions):
                    continue
                if route.drone_id not in self.drones:
                    raise FlyInError("a delivered drone cannot move again")
                drone = self.drones[route.drone_id]
                target = route.positions[number]
                if target == drone.location and drone.destination is None:
                    continue
                zone = self._move(drone, target, usage)
                movements.append(Movement(drone.drone_id, target, zone))
                if target == self.network.end:
                    del self.drones[drone.drone_id]
                    self.delivered += 1
            occupancy = Counter(drone.location
                                for drone in self.drones.values()
                                if drone.destination is None)
            self._validate_capacity(occupancy, usage, number)
            yield Turn(number, tuple(sorted(movements,
                                            key=lambda move: move.drone_id)),
                       dict(occupancy), dict(usage), self.delivered)
            if not self.drones:
                return
        raise FlyInError("schedule ended before all drones arrived")

    def _move(self, drone: Drone, target: str,
              usage: Counter[tuple[str, str]]) -> str:
        """Validate one movement, including compulsory transit completion."""
        network = self.network
        if drone.destination is not None:
            if target != drone.destination:
                raise FlyInError("restricted transit must finish next turn")
            first, second = drone.location.split("-")
            link = network.connection(first, second)
            drone.destination = None
            zone_name = target
        elif target in network.zones:
            zone = network.zones[target]
            if zone.kind in (ZoneType.BLOCKED, ZoneType.RESTRICTED):
                raise FlyInError(
                    "invalid direct entry into blocked/restricted zone"
                )
            link = network.connection(drone.location, target)
            zone_name = target
        else:
            if target.count("-") != 1:
                raise FlyInError(f"unknown zone or connection '{target}'")
            first, second = target.split("-")
            link = network.connection(first, second)
            if (first != drone.location
                    or network.zones[second].kind is not ZoneType.RESTRICTED):
                raise FlyInError("invalid restricted-zone departure")
            drone.destination = second
            zone_name = second
        usage[link.key] += 1
        drone.location = target
        return zone_name

    def _validate_capacity(self, occupancy: Counter[str],
                           usage: Counter[tuple[str, str]], turn: int) -> None:
        """Check snapshots after departures and arrivals have all occurred."""
        for name, count in occupancy.items():
            if count > self.network.capacity(name):
                raise FlyInError(
                    f"turn {turn}: zone '{name}' exceeds capacity"
                )
        for key, count in usage.items():
            if count > self.network.links[key].capacity:
                raise FlyInError(
                    f"turn {turn}: connection {key} exceeds capacity"
                )
