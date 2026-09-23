*This project has been created as part of the 42 curriculum by danda-si.*

# Fly-in

## Description

Fly-in routes a fleet of drones from a shared starting hub to a shared destination
through a bidirectional network. It schedules simultaneous movements while
respecting zone occupancy, shared connection capacity, blocked zones, and the
two turns required to enter restricted zones. The objective is to deliver the
whole fleet in as few turns as possible.

This implementation follows Fly-in subject **version 1.6**. It uses Python 3.10+
and the standard library for all runtime functionality. The graph, pathfinding,
reservations, simulation and terminal renderer are implemented locally; no graph
library is used. All Python source files and supplied maps are at the repository
root.

## Instructions

From the root of this repository:

```sh
make install
make run
make run MAP=medium_02_circular_loop.txt ARGS="--visual --stats"
```

`make install` creates `.venv` and installs the pinned development tools,
**flake8** and **mypy**. Running the simulation itself requires no third-party
packages and does not require activation of the environment:

```sh
python3 main.py easy_01_linear_path.txt
./main.py --capacity-info hard_02_capacity_hell.txt
python3 main.py --color always --visual challenger_01_the_impossible_dream.txt
python3 main.py --help
```

The required Makefile targets are available:

| Target | Purpose |
| --- | --- |
| `make install` | Create a virtual environment and install checking tools. |
| `make run MAP=file.txt ARGS="..."` | Run the simulator; defaults to the linear map. |
| `make debug MAP=file.txt` | Run with Python's `pdb` debugger. |
| `make clean` | Remove Python bytecode and tool caches. |
| `make lint` | Run flake8 and mypy with all flags required by the subject. |
| `make lint-strict` | Run flake8 and `mypy . --strict`. |

To invoke the tools directly:

```sh
. .venv/bin/activate
flake8 .
mypy .
```

### Command-line options and output

- `--color auto|always|never`: color movement tokens using destination metadata.
  The default is `auto`, enabled when stdout is a terminal and `NO_COLOR` is not
  set. Redirected output is plain text by default.
- `--visual`: print network topology, coordinates, zone types, movements,
  delivered counts, and zone/link usage to stderr.
- `--capacity-info`: print every zone and connection's used/available capacity
  each turn to stderr. Both turns of restricted travel count as link usage.
- `--stats`: print the final drone count, turn count and elapsed time to stderr.

Stdout contains exactly one space-separated movement line per turn. Stationary
and previously delivered drones are omitted. An idle turn, if present, is an
empty line. No summaries or diagnostic text are mixed into stdout.

```sh
python3 main.py hard_01_maze_nightmare.txt --stats > movements.txt
```

Errors are reported on stderr with exit code 1; argument errors use exit code 2.
Parser errors identify the source line and cause, including missing records at
EOF. Ctrl-C exits with code 130. Unreachable destinations are reported before
movement output begins.

## Input format

The first nonempty, noncomment record must be `nb_drones: <positive integer>`.
Exactly one `start_hub` and one `end_hub` are required. Zone definitions contain a
unique name and two integer coordinates. Connections may only reference zones
already declared. Connections are bidirectional, so `a-b` and `b-a` are duplicates.
A connection must join two distinct zones. Blank lines and text beginning with
`#`, including trailing comments, are ignored.

```text
nb_drones: 2
start_hub: start 0 0 [color=green]
hub: roof 1 0 [zone=restricted color=red max_drones=1]
end_hub: goal 2 0 [color=yellow]
connection: start-roof [max_link_capacity=1]
connection: roof-goal
```

Names must be printable tokens without whitespace or dashes; `#` is reserved
for comments. Coordinates may be negative. Separate tokens and optional metadata
blocks with whitespace. Metadata tags may appear in any order, inside one pair
of brackets, as `key=value` tokens.

| Metadata | Default | Meaning |
| --- | --- | --- |
| `zone` | `normal` | `normal`, `priority`, `restricted` or `blocked`. |
| `color` | No color | Any printable single-word color name. |
| `max_drones` | `1` | Positive occupancy limit for a regular hub. |
| `max_link_capacity` | `1` | Positive shared edge limit, in either direction. |

`max_link_capacity` is connection metadata; the other tags are zone metadata.
Unknown tags, duplicate tags, malformed brackets, duplicate names/edges, invalid
zone types and invalid regular capacities are rejected. As specified in the
subject, `max_drones` on start/end hubs is ignored, including values that would
be invalid on a regular hub. Start and end have no occupancy limit; this does
not override a `blocked` zone type.

There is no configured maximum fleet size, coordinate magnitude or capacity.
Practical limits are available memory, execution time and Python integer parsing.

### Example output

For the two-drone restricted-roof example above, with colors disabled:

```text
D1-start-roof
D1-roof
D1-goal D2-start-roof
D2-roof
D2-goal
```

`D1-start-roof` means D1 is in flight on the connection from `start` to `roof`.
It must arrive at `roof` on the next turn. The shared connection is occupied on
both turns, so D2 cannot start its traversal until turn 3. The source-to-destination
order in a transit token indicates direction even if the input declares the
bidirectional edge in the opposite order.

For `easy_01_linear_path.txt`, the exact plain output is:

```text
D1-waypoint1
D1-waypoint2 D2-waypoint1
D1-goal D2-waypoint2
D2-goal
```

## Algorithm explanation

### Object-oriented structure

| File | Responsibility |
| --- | --- |
| `models.py` | Zone/connection values, custom adjacency-list graph, drone and turn state. |
| `parser.py` | Syntax, semantic validation, line-aware errors and file handling. |
| `planner.py` | Reverse Dijkstra, space-time A*, and capacity reservations. |
| `simulation.py` | Independent turn-by-turn schedule validation and drone delivery. |
| `visualizer.py` | Movement tokens, terminal colors, topology and capacity views. |
| `main.py` | CLI configuration and application orchestration. |

Immutable dataclasses represent map values and completed snapshots; mutable
objects own graph construction, search resources and live drone state. Composition
separates responsibilities without an unnecessary inheritance hierarchy. Every
class and function has a docstring and all public/internal functions are typed.

### 1. Cache distances to the destination

A custom reverse Dijkstra search computes a shortest remaining travel time from
every reachable zone. A forward edge costs the **destination's** entry cost:
normal/priority costs 1, restricted costs 2. Reverse relaxation consequently uses
the current node's entry cost, rather than the predecessor's.

Blocked zones are excluded. Priority preference is a secondary, nonnegative cost:
entering a priority zone adds 0; entering another accessible zone adds 1. The
search compares `(travel time, nonpriority entries)` lexicographically, so priority
preference never outweighs an earlier arrival. These distances are cached for the
entire fleet. An unreachable start terminates planning with a clear error.

### 2. Plan in space and time

Each drone is planned using A* over states `(zone, turn)`. A successor can:

1. Wait one turn, if the current zone has capacity at the next snapshot.
2. Enter a normal/priority neighbor after one turn.
3. Enter a restricted neighbor after two turns, reserving the connection for both
   turns and checking the destination's capacity at the mandatory arrival time.

The search uses cached remaining time as an admissible heuristic and cached
priority preference for ties. It explores the graph directly rather than a fixed
list of paths. Consequently, later drones can wait or choose alternative routes
when reservations make a previously used route slower.

Only complete routes are committed to the sparse reservation tables. Zone
capacity is checked at the **end of each turn**, allowing another drone to enter
as a previous occupant leaves. A connection's resource key is independent of
travel direction, preventing opposite-direction movements from exceeding its
shared capacity. A restricted transition is indivisible: no waiting in flight is
possible, and arrival capacity is checked before departure is scheduled.

### 3. Bound the search without an arbitrary timeout

Let `H_previous` be the latest delivery already reserved and `d(start)` the cached
shortest travel time. The next search uses the horizon
`H_previous + d(start)`.

This is always a feasible upper bound on an accessible map: the next drone can
wait at the unlimited start until all previous drones are delivered, then follow
a shortest path through an empty network. This prevents unbounded waiting states
and ensures the planner can deliver every drone without deadlock. Moving back
into the start is unnecessary and excluded; waiting there is always possible.

### 4. Replay and validate

The simulator replays the resulting routes independently of reservation checks.
It verifies adjacency, movement cost, compulsory next-turn restricted arrivals,
zone capacity, shared link capacity and unique drone IDs. All departures and
arrivals are applied before checking zone occupancy. Drones are removed on
arrival at the destination; simulation stops immediately after the final delivery.

### Complexity and tradeoffs

For `V` zones, `E` connections, `D` drones and maximum planning horizon `H`:

- Cached reverse Dijkstra: `O((V + E) log V)` time and `O(V + E)` storage.
- One space-time search has up to `O(VH)` states and `O((V + E)H)` transitions.
  Its usual heap-search bound is `O((V + E)H log(VH))` time.
- Planning the fleet is bounded by `O(D(V + E)H log(VH))` time.
- Search/reservation storage is bounded by `O((V + E)H)`; complete routes add
  `O(DH)` storage. Sparse dictionaries avoid allocating all possible states.
- Replay takes `O(DH)` time, plus any requested topology/capacity rendering.

This is **prioritized cooperative planning**: each new drone gets an earliest
safe arrival given already committed routes. It is not a proof of globally
minimum fleet makespan for every possible graph. Earlier routes are not rerouted
using residual flow or exhaustive joint search. This keeps the implementation
small, deterministic and explainable while meeting all provided benchmarks.
Large fleets with narrow bottlenecks increase the time horizon and can lead to
quadratic fleet-size behavior. There is no artificial turn cutoff.

## Visual representation

On an interactive terminal, colored movement tokens use the destination zone's
`color` metadata. In-flight tokens use the restricted destination's color. Common
names (including orange, gold, brown and crimson) map to ANSI 256-color values.
The subject accepts arbitrary color names; unknown names such as `rainbow` are
accepted and assigned a deterministic palette color using SHA-256. There is no
fixed parser whitelist, and color labels are never inserted as terminal escapes.

`--visual` adds an initial adjacency view with the supplied coordinates and zone
types, followed by labeled movements and occupancy/link usage for each turn.
This makes bottlenecks, active flights and delivered drones visible. Stationary
drones remain represented in occupancy counts. Color is supplementary: explicit
names, types and counts remain readable when colors are disabled.

`--capacity-info` exposes the same capacity reporting without the topology view.
For example:

```text
Capacity, turn 1:
  Zone start: 1/unlimited drones
  Zone roof: 0/1 drones
  Zone goal: 0/unlimited drones
  Connection start-roof: 1/1 used
  Connection roof-goal: 0/1 used
```

## Validation and benchmark results

The implementation passed `make lint`, `make lint-strict`, and an external
unittest suite covering parser failures, defaults, unusual names, ignored
terminal capacities, single drones, priority ties, disconnected/blocked routes,
simultaneous departures, restricted start/end zones, high-capacity links,
opposite-direction conflicts, invalid schedules, CLI behavior and rendering.

The external suite independently replays output tokens, in addition to the
production simulator's checks. It exercises 250 deterministic random maps, a
501-zone linear graph, and a 1,000-drone fleet. Test programs remain outside the
submission repository, as requested by the common instructions.

Measured results for the unchanged official maps:

| Map | Drones | Turns | Reference target |
| --- | ---: | ---: | ---: |
| Easy: linear path | 2 | 4 | ≤ 6 |
| Easy: simple fork | 4 | 4 | ≤ 8 |
| Easy: basic capacity | 4 | 4 | ≤ 6 |
| Medium: dead end trap | 5 | 8 | ≤ 12 |
| Medium: circular loop | 6 | 15 | ≤ 15 |
| Medium: priority puzzle | 5 | 7 | ≤ 12 |
| Hard: maze nightmare | 8 | 13 | ≤ 30 |
| Hard: capacity hell | 12 | 16 | ≤ 35 |
| Hard: ultimate challenge | 15 | 26 | ≤ 45 |
| Challenger: the impossible dream | 25 | **43** | **< 45** |

All nine mandatory reference targets are met, and the Challenger beats the
45-turn reference. Reproduce a result with:

```sh
python3 main.py challenger_01_the_impossible_dream.txt --stats
```

## Resources

- [Fly-in subject, version 1.6](https://cdn.intra.42.fr/pdf/pdf/221200/en.subject.pdf)
  — authoritative project requirements.
- [Official map archive](https://cdn.intra.42.fr/document/document/55008/maps.tar.gz)
  — map contents copied unchanged; category prefixes replace their original
  directories so all deliverable files are at the repository root.
- [Python heapq](https://docs.python.org/3/library/heapq.html) — priority queues
  used by Dijkstra and A*.
- [Python dataclasses](https://docs.python.org/3/library/dataclasses.html) —
  domain values and snapshots.
- [Python typing](https://docs.python.org/3/library/typing.html),
  [mypy documentation](https://mypy.readthedocs.io/en/stable/), and
  [flake8 documentation](https://flake8.pycqa.org/en/latest/) — type and style checks.
- Dijkstra, E. W. (1959), *A note on two problems in connexion with graphs* —
  shortest-path foundations.
- Hart, P. E., Nilsson, N. J., and Raphael, B. (1968), *A Formal Basis for the
  Heuristic Determination of Minimum Cost Paths* — A* foundations.
- Silver, D. (2005), *Cooperative Pathfinding* — space-time search and reservations.

### AI use

OpenAI Codex assisted with reading the subject and evaluation checklist,
designing the object-oriented architecture and reservation algorithm, generating
the Python implementation, Makefile and documentation, and creating/running the
external validation suite. This includes all six Python modules. Generated
behavior was checked against the official maps, malformed inputs, randomized
networks, an independent output replay checker, flake8 and strict mypy. These
checks support correctness; they do not replace understanding and explaining the
implementation during peer review.
