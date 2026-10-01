```@meta
CurrentModule = SimpleKiteControllers
```

# Winch

## Wind-dependent winch table

The parameters of the reel-out law that change with wind speed, from the project's
`winch_table` file.

```@docs
winch_f_low
winch_force_limit
winch_table_lookup
winch_table_select
```

## Winch settings and controllers

```@docs
load_wc_settings
build_winch
build_controllers
winch_force_gains
```
