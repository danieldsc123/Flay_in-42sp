"""Strict, line-aware parser for the Fly-in map format."""

from pathlib import Path
import re

from models import Connection, FlyInError, Network, Zone, ZoneType


class MapParser:
    """Parse input without silently accepting unknown directives or tags."""

    def parse(self, path: Path) -> Network:
        """Read UTF-8 input using a context manager and parse its records."""
        try:
            with path.open(encoding="utf-8") as source:
                return self.parse_text(source.read())
        except (OSError, UnicodeError) as exc:
            raise FlyInError(f"cannot read '{path}': {exc}") from exc

    def parse_text(self, text: str) -> Network:
        """Validate records, retaining original line numbers for errors."""
        network: Network | None = None
        line_number = 0
        for line_number, raw in enumerate(text.splitlines(), start=1):
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            try:
                directive, separator, body = line.partition(":")
                if not separator:
                    raise FlyInError("expected a directive followed by ':'")
                directive = directive.strip()
                body = body.strip()
                if network is None:
                    if directive != "nb_drones":
                        raise FlyInError("first record must be nb_drones")
                    network = Network(self._positive(body, "nb_drones"))
                elif directive in ("start_hub", "end_hub", "hub"):
                    self._zone(network, directive, body)
                elif directive == "connection":
                    self._link(network, body)
                else:
                    raise FlyInError(f"unexpected directive '{directive}'")
            except (FlyInError, ValueError) as exc:
                raise FlyInError(f"line {line_number}: {exc}") from exc
        missing = []
        if network is None:
            missing.append("nb_drones")
        if network is None or not network.start:
            missing.append("start_hub")
        if network is None or not network.end:
            missing.append("end_hub")
        if missing:
            raise FlyInError(
                f"line {line_number + 1} (EOF): missing {', '.join(missing)}"
            )
        assert network is not None
        return network

    @staticmethod
    def _positive(value: str, label: str) -> int:
        """Require an ASCII positive integer, without an arbitrary maximum."""
        if not re.fullmatch(r"[0-9]+", value) or int(value) <= 0:
            raise FlyInError(f"{label} must be a positive integer")
        return int(value)

    @staticmethod
    def _metadata(body: str, allowed: set[str]) -> tuple[str, dict[str, str]]:
        """Separate a single optional bracket block and validate its tags."""
        match = re.fullmatch(r"([^\[\]]*?)(?:\s*\[([^\[\]]*)\])?", body)
        if match is None:
            raise FlyInError("malformed metadata brackets")
        values: dict[str, str] = {}
        for tag in (match.group(2) or "").split():
            if tag.count("=") != 1:
                raise FlyInError(f"invalid metadata token '{tag}'")
            key, value = tag.split("=", 1)
            if key not in allowed or not value or key in values:
                raise FlyInError(
                    f"unknown, empty or duplicate metadata '{tag}'"
                )
            if not value.isprintable():
                raise FlyInError("metadata must contain printable values")
            values[key] = value
        return match.group(1).strip(), values

    def _zone(self, network: Network, role: str, body: str) -> None:
        """Validate a zone and ignore max_drones on terminal zones."""
        parts = body.split(maxsplit=3)
        if len(parts) < 3:
            raise FlyInError("expected zone name and two integer coordinates")
        name, x, y = parts[:3]
        remainder, tags = self._metadata(
            parts[3] if len(parts) == 4 else "",
            {"zone", "color", "max_drones"},
        )
        if remainder:
            raise FlyInError("unexpected content after zone coordinates")
        if "-" in name or not name.isprintable():
            raise FlyInError("zone names cannot contain dashes or controls")
        if not all(re.fullmatch(r"[+-]?[0-9]+", v) for v in (x, y)):
            raise FlyInError("zone coordinates must be integers")
        try:
            kind = ZoneType(tags.get("zone", "normal"))
        except ValueError as exc:
            raise FlyInError(f"invalid zone type '{tags['zone']}'") from exc
        capacity = 1
        if role == "hub":
            capacity = self._positive(tags.get("max_drones", "1"),
                                      "max_drones")
        network.add_zone(
            Zone(name, int(x), int(y), kind, tags.get("color"), capacity),
            role,
        )

    def _link(self, network: Network, body: str) -> None:
        """Validate endpoints, metadata and undirected duplicate edges."""
        parts = body.split(maxsplit=1)
        if not parts:
            raise FlyInError("missing connection endpoints")
        definition = parts[0]
        remainder, tags = self._metadata(
            parts[1] if len(parts) == 2 else "", {"max_link_capacity"}
        )
        if remainder:
            raise FlyInError("unexpected content after connection endpoints")
        if definition.count("-") != 1:
            raise FlyInError("expected connection: <zone1>-<zone2>")
        first, second = definition.split("-")
        capacity = self._positive(tags.get("max_link_capacity", "1"),
                                  "max_link_capacity")
        network.add_link(Connection(first, second, capacity))
