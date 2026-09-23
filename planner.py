"""Capacity-aware space-time A* with cached reverse Dijkstra distances."""

from heapq import heappop, heappush

from models import Connection, FlyInError, Network, Route, ZoneType

State = tuple[str, int]


class Reservations:
    """Reserve zone snapshots and all turns of each edge traversal."""

    def __init__(self, network: Network) -> None:
        """Create sparse capacity tables for one planning run."""
        self.network = network
        self.zones: dict[State, int] = {}
        self.links: dict[tuple[tuple[str, str], int], int] = {}
        self.last_arrival = 0

    def zone_free(self, name: str, turn: int) -> bool:
        """Check end-of-turn occupancy, allowing simultaneous departures."""
        if name in (self.network.start, self.network.end):
            return True
        return self.zones.get((name, turn), 0) < self.network.capacity(name)

    def link_free(self, link: Connection, start: int, end: int) -> bool:
        """Check a shared edge on every turn, including restricted arrival."""
        return all(self.links.get((link.key, turn), 0) < link.capacity
                   for turn in range(start + 1, end + 1))

    def reserve(self, route: Route) -> None:
        """Commit a complete route only after its safe arrival is known."""
        self.last_arrival = max(self.last_arrival, len(route.positions) - 1)
        for turn, position in enumerate(route.positions[1:], start=1):
            previous = route.positions[turn - 1]
            if position in self.network.zones:
                if position not in (self.network.start, self.network.end):
                    key = (position, turn)
                    self.zones[key] = self.zones.get(key, 0) + 1
            if previous == position:
                continue
            if "-" in position:
                first, second = position.split("-")
            elif "-" in previous:
                first, second = previous.split("-")
            else:
                first, second = previous, position
            resource = (self.network.connection(first, second).key, turn)
            self.links[resource] = self.links.get(resource, 0) + 1


class RoutePlanner:
    """Plan earliest safe arrivals one drone at a time, using all routes."""

    def __init__(self, network: Network) -> None:
        """Cache admissible distances and initialize reservation tables."""
        self.network = network
        self.reservations = Reservations(network)
        self.distances = self._distances()
        if network.start not in self.distances:
            raise FlyInError("no accessible path from start_hub to end_hub")

    def _distances(self) -> dict[str, tuple[int, int]]:
        """Run reverse Dijkstra with destination costs and priority ties."""
        network = self.network
        if network.zones[network.end].kind is ZoneType.BLOCKED:
            return {}
        distances = {network.end: (0, 0)}
        queue = [(0, 0, network.end)]
        while queue:
            cost, penalty, name = heappop(queue)
            if distances[name] != (cost, penalty):
                continue
            zone = network.zones[name]
            for link in network.adjacency[name]:
                neighbor = link.other(name)
                if network.zones[neighbor].kind is ZoneType.BLOCKED:
                    continue
                candidate = (cost + zone.cost,
                             penalty + (zone.kind is not ZoneType.PRIORITY))
                if (neighbor not in distances
                        or candidate < distances[neighbor]):
                    distances[neighbor] = candidate
                    heappush(queue, (*candidate, neighbor))
        return distances

    def plan(self) -> list[Route]:
        """Return deterministic routes for the complete fleet."""
        self.reservations = Reservations(self.network)
        routes = []
        for drone_id in range(1, self.network.drones + 1):
            route = self._route(drone_id)
            self.reservations.reserve(route)
            routes.append(route)
        return routes

    def _successors(self, state: State) -> list[tuple[State, int]]:
        """Offer safe moves and waits; restricted transit is indivisible."""
        name, turn = state
        successors: list[tuple[State, int]] = []
        for link in self.network.adjacency[name]:
            target = link.other(name)
            if target not in self.distances or target == self.network.start:
                continue
            zone = self.network.zones[target]
            arrival = turn + zone.cost
            if (self.reservations.zone_free(target, arrival)
                    and self.reservations.link_free(link, turn, arrival)):
                successors.append(((target, arrival),
                                   int(zone.kind is not ZoneType.PRIORITY)))
        if self.reservations.zone_free(name, turn + 1):
            successors.append(((name, turn + 1), 0))
        return successors

    def _route(self, drone_id: int) -> Route:
        """Search finite time-expanded states with a proven safe horizon."""
        start = (self.network.start, 0)
        distance, priority = self.distances[start[0]]
        # After the last delivery all resources are free: waiting at start
        # then following a shortest route is always a feasible upper bound.
        horizon = self.reservations.last_arrival + distance
        queue = [(distance, priority, 0, start[0], 0)]
        penalties: dict[State, int] = {start: 0}
        parents: dict[State, State] = {}
        while queue:
            _, _, turn, name, penalty = heappop(queue)
            state = (name, turn)
            if penalties[state] != penalty:
                continue
            if name == self.network.end:
                return self._reconstruct(drone_id, state, parents)
            for target, extra in self._successors(state):
                heuristic, preference = self.distances[target[0]]
                if target[1] + heuristic > horizon:
                    continue
                candidate = penalty + extra
                if target in penalties and penalties[target] <= candidate:
                    continue
                penalties[target] = candidate
                parents[target] = state
                heappush(queue, (target[1] + heuristic,
                                 candidate + preference,
                                 target[1], target[0], candidate))
        raise FlyInError("unable to construct a capacity-safe schedule")

    @staticmethod
    def _reconstruct(drone_id: int, final: State,
                     parents: dict[State, State]) -> Route:
        """Expand two-turn moves into connection and destination positions."""
        states = [final]
        while states[-1] in parents:
            states.append(parents[states[-1]])
        states.reverse()
        positions = [states[0][0]]
        for previous, current in zip(states, states[1:]):
            if current[1] - previous[1] == 2:
                positions.append(f"{previous[0]}-{current[0]}")
            positions.append(current[0])
        return Route(drone_id, tuple(positions))
