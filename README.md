# Foxity

A city builder on a small planet, built on the
[laiue](https://github.com/landiunload/laiue) engine and inspired by SimCity and
Cities: Skylines.

![The demo town](docs/town.png)

Early development. Space is unbounded; in it hangs one planet, a ball a
kilometre in radius under an always bright sky. The planet is solid through
and through: it can be dug down to its core and piled up anywhere. Roads are
drawn as straight lines or curves draped over the ground; where they meet or
cross, intersections form by themselves. Cars drive their lanes, keep their
distance, wait their turn at intersections, take the fastest route and avoid
jams. Each city chooses whether it keeps right or left when it is founded,
and districts can be laid over it that drive on the other side.

The simulation uses only integer arithmetic at a fixed tick rate, so the same
commands give the same city on every machine.

![The planet](docs/planet.png)

The engine's window, input, graphics, camera math, meshing and navigation are
loaded at run time. Any of them can be missing; the game then runs without that
technology instead of failing to start.

## Controls

| Input | Action |
|---|---|
| Mouse wheel | Zoom from 4 m to 9 km |
| Right mouse button | Rotate |
| W A S D, Shift | Move, faster |
| Left mouse button | Use the tool |
| 1 | Road: click a start and an end; the next road continues from the end |
| 2 | Curved road: a start, a point to bend towards and an end |
| 3 | Bulldoze a road |
| 4, 5 | Dig the ground away or pile it up, held |
| 6 | Turn the district around the point to the other drive side |
| T | Next road type: street, avenue, boulevard, one-way street, highway |
| 7, 8, 9 | Time at one, three or ten times real time |
| Space | Pause |
| E, G | Fifty cars more or fewer |
| Esc | Drop the road being placed, or quit |

## Running

```bash
Foxity --drive left
```

| Option | Meaning |
|---|---|
| `--drive right\|left` | The city's drive side |
| `--cars N` | How many cars the city aims for |
| `--empty` | Start on bare ground instead of the demo town |
| `--seed N` | The city's seed |
| `--camera-distance M` | The camera's distance to its focus |
| `--graphics d3d12\|vulkan` | Choose the graphics provider |
| `--capture FILE.bmp`, `--capture-frame N` | Save frame N as a picture and quit |
| `--smoke` | Present three frames and exit |

`FoxityHeadless` runs the city without presentation:
`--ticks N --time-scale X --cars N --drive right|left --no-routing`.

## Building

The engine is cloned next to the game and built with it:

```bash
git clone https://github.com/landiunload/laiue.git ../laiue
cmake --preset windows-msvc
cmake --build --preset windows-msvc-release --parallel
ctest --preset windows-msvc-release --output-on-failure
```

Without `../laiue`, configure downloads the engine (`FOX_ENGINE_REVISION`,
`origin/main` by default, `locked` for the revision in `engine.lock`). A
prepared engine SDK is chosen with `-Dlaiue_DIR=...`.

Linux and macOS build the headless game and its tests. The design, rules and
checks are in [docs/architecture.md](docs/architecture.md) and
[CONTRIBUTING.md](CONTRIBUTING.md).

## License

MIT, see [LICENSE](LICENSE).
