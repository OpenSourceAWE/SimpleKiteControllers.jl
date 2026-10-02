```@meta
CurrentModule = SimpleKiteControllers
```

# API overview

The docstrings of all exported types, functions and constants, split by the part of a
kite-power run they belong to. The pages go from the controllers outward:

| Page | Contents |
|:-----|:---------|
| [Flight control](flight_control.md) | course controller, figure-of-eight guidance, turn-rate law |
| [Flight-path geometry](path_geometry.md) | building, querying and checking a reference path |
| [Winch](winch.md) | wind-dependent winch table, winch settings and controllers |
| [Settings and data files](settings.md) | `FC_Settings`, `TrajOptSettings`, project files, menu state |
| [Reel-out runs](reelout_run.md) | inputs, time budget, feasibility gates, run state and log |
| [Run evaluation](evaluation.md) | metrics of a logged run and its commented summary |
| [Course-loop stability](stability.md) | linear model of the course loop |
| [Internals](internals.md) | non-exported names the docstrings refer to |
