#!/usr/bin/env python3
"""Command-line entry point for the Fly-in drone routing simulator."""

import argparse
import os
from pathlib import Path
import sys
import time

from models import FlyInError
from parser import MapParser
from planner import RoutePlanner
from simulation import Simulation
from visualizer import TerminalRenderer


class Application:
    """Coordinate parsing, planning, validated simulation and rendering."""

    @staticmethod
    def arguments(argv: list[str] | None) -> argparse.Namespace:
        """Parse the input map and optional presentation settings."""
        parser = argparse.ArgumentParser(
            description="Route drones through a capacity-constrained network.",
        )
        parser.add_argument("map", type=Path, help="UTF-8 map file")
        parser.add_argument("--color", choices=("auto", "always", "never"),
                            default="auto",
                            help="terminal colors (auto by default)")
        parser.add_argument("--visual", action="store_true",
                            help="show topology and turn states on stderr")
        parser.add_argument("--capacity-info", action="store_true",
                            help="show capacities per turn on stderr")
        parser.add_argument("--stats", action="store_true",
                            help="show turn count and elapsed time on stderr")
        return parser.parse_args(argv)

    def run(self, argv: list[str] | None = None) -> int:
        """Run the application and turn expected failures into clear errors."""
        args = self.arguments(argv)
        try:
            started = time.perf_counter()
            network = MapParser().parse(args.map)
            routes = RoutePlanner(network).plan()
            color = args.color == "always" or (
                args.color == "auto" and sys.stdout.isatty()
                and "NO_COLOR" not in os.environ
            )
            renderer = TerminalRenderer(network, sys.stdout, sys.stderr,
                                        color, args.visual, args.capacity_info)
            renderer.begin()
            turns = 0
            for turn in Simulation(network, routes).turns():
                renderer.render(turn)
                turns = turn.number
            if args.stats:
                elapsed = time.perf_counter() - started
                print(f"Delivered: {network.drones}; turns: {turns}; "
                      f"elapsed: {elapsed:.3f}s", file=sys.stderr)
            return 0
        except BrokenPipeError:
            # Redirect before interpreter shutdown so piping to head is quiet.
            with open(os.devnull, "w") as sink:
                os.dup2(sink.fileno(), sys.stdout.fileno())
            return 0
        except (FlyInError, OSError, ValueError) as exc:
            print(f"Error: {exc}", file=sys.stderr)
            return 1
        except KeyboardInterrupt:
            print("Interrupted.", file=sys.stderr)
            return 130


if __name__ == "__main__":
    sys.exit(Application().run())
