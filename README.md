# raylib-pathfinder

Grid pathfinding visualizer (Dijkstra and A*) written in C with raylib.

Draw walls, then drag the start and goal nodes. The path updates live.

![A* with hand-drawn walls](docs/raylib-pathfinder_7ozXybCBOT.gif)

Dijkstra solving a generated maze.

![Dijkstra on a generated maze](docs/raylib-pathfinder_5WfAUWtsha.gif)

## Controls

| Input | Action |
|---|---|
| Left mouse drag | Paint walls, or drag the start/goal node |
| Right mouse drag | Erase walls |
| `S` / `Space` | Start search (single step while paused) |
| `Shift` + `S` | Step one frame's worth while paused |
| `P` | Pause / resume |
| `Left` / `Right` | Speed -1 / +1 steps per frame (`Shift`: 4) |
| `1` / `2` | Dijkstra / A* |
| `G` | Generate maze |
| `R` | Clear grid |
| `Esc` | Quit |

## Build

Requires CMake 3.26+, Ninja and a C compiler. raylib is fetched automatically.

```sh
cmake --preset release
cmake --build --preset release
```

Binary: `build/release/raylib-pathfinder`. Use `debug` in place of `release` for a debug build.
