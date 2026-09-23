"""Render exact movement output and an optional terminal state display."""

from hashlib import sha256
from typing import TextIO

from models import Network, Turn


class TerminalRenderer:
    """Keep simulation tokens on stdout and diagnostic views on stderr."""

    COLORS = {
        "black": 16, "red": 196, "green": 34, "yellow": 226,
        "blue": 33, "magenta": 201, "cyan": 51, "white": 255,
        "gray": 244, "grey": 244, "orange": 208, "purple": 93,
        "brown": 130, "lime": 118, "gold": 220, "maroon": 88,
        "darkred": 88, "violet": 177, "crimson": 161,
    }

    def __init__(self, network: Network, output: TextIO, diagnostics: TextIO,
                 color: bool, visual: bool, capacity_info: bool) -> None:
        """Configure colors, state snapshots and capacity diagnostics."""
        self.network = network
        self.output = output
        self.diagnostics = diagnostics
        self.color = color
        self.visual = visual
        self.capacity_info = capacity_info

    def _paint(self, text: str, zone_name: str) -> str:
        """Use named colors and a deterministic palette for arbitrary names."""
        value = self.network.zones[zone_name].color
        if not self.color or value is None:
            return text
        # The subject accepts arbitrary single-word color names (rainbow too).
        # Unknown names remain valid and receive a stable visible ANSI color.
        fallback = 16 + int.from_bytes(sha256(value.encode()).digest()[:2],
                                       "big") % 216
        index = self.COLORS.get(value.lower(), fallback)
        return f"\033[38;5;{index}m{text}\033[0m"

    def begin(self) -> None:
        """Show topology and turn-zero positions in visual mode."""
        if not self.visual:
            return
        print("Map (bidirectional connections):", file=self.diagnostics)
        for name, zone in self.network.zones.items():
            neighbors = ", ".join(link.other(name)
                                  for link in self.network.adjacency[name])
            label = self._paint(name, name)
            print(f"  {label} ({zone.x}, {zone.y}) [{zone.kind.value}]"
                  f" <-> {neighbors or '(isolated)'}", file=self.diagnostics)
        print(f"Turn 0: {self.network.drones} drones at {self.network.start}",
              file=self.diagnostics)

    def render(self, turn: Turn) -> None:
        """Print exactly one movement line and optional state information."""
        print(" ".join(self._paint(move.token, move.zone)
                       for move in turn.movements), file=self.output)
        if self.visual:
            print(f"Turn {turn.number}: delivered {turn.delivered}/"
                  f"{self.network.drones}", file=self.diagnostics)
            for move in turn.movements:
                print(f"  D{move.drone_id} -> "
                      f"{self._paint(move.location, move.zone)}",
                      file=self.diagnostics)
        if self.visual or self.capacity_info:
            self._capacities(turn)

    def _capacities(self, turn: Turn) -> None:
        """Show zone and connection usage, including transit turns."""
        print(f"Capacity, turn {turn.number}:", file=self.diagnostics)
        for name in self.network.zones:
            count = turn.occupancy.get(name, 0)
            limit = str(self.network.capacity(name))
            if name in (self.network.start, self.network.end):
                limit = "unlimited"
            if name == self.network.end:
                count = turn.delivered
            label = self._paint(name, name)
            print(f"  Zone {label}: {count}/{limit} drones",
                  file=self.diagnostics)
        for key, link in self.network.links.items():
            used = turn.link_usage.get(key, 0)
            print(f"  Connection {link.name}: {used}/{link.capacity} used",
                  file=self.diagnostics)
