# Plan: static HTML export of `notebooks/results.jl`

`notebooks/results.jl` is a KaimonSlate notebook that works live. The goal is that
**☰ → Export HTML** produces a self-contained page where

- the checkboxes still show and hide columns of the overview table, and
- the slider still switches between the eight flight-pattern plots,

with no Julia behind the page.

`notebooks/interactive_table.jl` is the reference example for the mechanism, and
`docs/` in KaimonSlate is the reference for the mechanism's limits.

## What `@replay` actually covers

Read from the installed KaimonSlate 1.3.1 (`~/.julia/packages/KaimonSlate/bXFXo`), which is
the version the running Slate server resolves to (`~/.julia/environments/kaimon-ext/slate/Manifest.toml`).

`@replay(ctl, expr)` evaluates `expr` once per value `ctl` can take, packs the results, and ships
them; the page then indexes that data when the control moves (`src/widgets.jl:801`, `_do_replay`).
Three properties decide whether it can be used here:

1. **One control per marked value.** The closure's parameter *is* the control
   (`src/widgets.jl:1296`). A value that depends on two controls has no single domain to sweep. For a
   table the export can compose the chain from the dependency graph, but it still picks exactly one
   control and *holds* the rest at their export-time value
   (`src/server_export.jl:305`, `_chain_sweep_plan`).
2. **A replayed table varies its ROWS, never its columns.** What ships is the union of the rows plus,
   per position, a row order. Differing column counts across positions is a hard error:
   *"A replayed table writes ONE header into the page, so every position must agree on its columns."*
   (`src/widgets.jl:967`, `_table_replay_pack`).
3. **Only ECharts series and tables are wired.** `Slate.replay.wire` is called from the chart runtime
   and from the table enhancer (`src/server_export.jl:1101`, `:1709`). Markdown and web-cell `{{ }}`
   interpolation is not replayed — it renders once, at export, and freezes.

## Consequence: neither goal is reachable with `@replay`

**The column checkboxes.** The four boxes are bound as one `MultiCheckBox`, so the control is already
the right shape for a sweep: a single control with a finite domain (the power set of its four options,
16 positions). Finding 2 is what blocks it. Hiding a column changes the column count, and a replayed
table refuses that outright — `select!(overview_df, Not(...))` is exactly the operation the mechanism
cannot carry, whatever drives it.

**The pattern plot.** The cell is a `#%% web` cell interpolating the wind speed into an image URL.
Nothing replays a web cell's `{{ }}`, so the `src` freezes at whatever `wind_speed` was set to during
the export. Worse, the export collects referenced asset files by scanning cell **source** and
**rendered output** for literal `/n/<id>/asset/…` URLs (`src/server_export.jl:766`,
`_embedded_asset_files`); `pattern_v{{ … }}.png` is not a literal, so only the one PNG that happened
to be rendered would ride along. Seven of the eight would be missing from the page even if the
switching worked.

So the TODO as written — "wrap the relevant expressions in the `@replay` macro" — does not lead
anywhere for either goal. The macro is the right tool for a chart whose *numbers* move; both of these
are cases where the *page structure* moves.

## What does work: drive both from the page's own JS

The exported page ships `Slate.replay` as a general control API, independent of whether any sweep was
shipped for a given control (`src/server_export.jl:1185`):

- `Slate.replay.hosts(name)` — every DOM node bound to that control (a control can render more than once)
- `.read(h)` — its current value, in the same shape Julia's domain uses (a number for a slider, an array of checked values for a multi-check)
- `.listen(h, run)` — fire on change
- `.enable(h, true)` — **required**: every exported control renders disabled, and only `wire()` enables
  one. A control we drive ourselves must be enabled by our own code.
- `Slate.isLive()` — true in the notebook, false in the export.

A web cell's HTML/CSS/JS travels into the export and its script is revived on load
(`docs/src/frontend-extensions.md`), and `<img src="/n/<id>/asset/…">` in a cell's rendered HTML is
rewritten to an inline base64 `data:` URI (`src/server_export.jl:722`, `_export_embed_html`). That is
enough for both goals, and it is the same idiom twice.

### Change 1 — the pattern plot

Replace the single interpolated `<img>` with **eight literal `<img>` tags**, one per wind speed, all
but one hidden. Literal URLs are what makes all eight get collected and inlined; the JS only toggles
which is visible.

```julia
#%% web id=pattern_plot controls=wind_speed
@web(html"""
<div id="pattern">
  <img data-v="3"  src="/n/results/asset/notebooks/pattern_v03.png" alt="flight pattern, 3 m/s">
  …one per wind speed, 3 through 10…
</div>
""",
css"""
#pattern img { display: block; margin: 0 auto; }
#pattern img[hidden] { display: none; }
""",
js"""
const pick = v => document.querySelectorAll("#pattern img")
    .forEach(im => im.hidden = Number(im.dataset.v) !== Number(v));
pick({{ wind_speed }});
if (window.Slate && !Slate.isLive()) {
  Slate.replay.hosts("wind_speed").forEach(h => {
    Slate.replay.enable(h, true);
    Slate.replay.listen(h, () => pick(Slate.replay.read(h)));
  });
}
""")
```

Live, `{{ wind_speed }}` re-renders the cell as it does today and the `isLive()` branch is skipped.
Exported, the initial `pick` runs from the frozen value and the listener takes over. Cost: eight PNGs
inlined, about 550 KB base64'd.

### Change 2 — the column checkboxes

Ship the table **once, with every column**, and hide columns in the browser. Same shape as change 1:
a `WebPage` cell (or a web cell) next to the table whose JS maps the header text to a column index and
sets `display:none` on that `<th>` and its `<td>`s.

No sweep runs and no per-position data ships: the `columns` value is read straight from the DOM.

The two things to settle while implementing it:

- **Finding the table.** The exported table is static HTML inside its cell's output. Give the JS a
  stable hook rather than relying on document order.
- **Column identity.** Match on header text (`opt_requests`, `min_force`, …), which is what the group
  names in `results.jl` already select on, rather than on a fixed index — the index shifts as soon as
  another group is hidden.

The `columns` control needs one `Slate.replay.enable` + `listen`, as in change 1.
`Slate.replay.read` on a `multicheck` host hands back the array of checked values
(`src/server_export.jl:1207`), so the JS can compare against the same option strings the Julia guards
in the table cell already use.


## Steps

1. [x] Apply change 1 and export. Confirm in the exported HTML that eight `data:image/png` URIs are
   present and the slider moves the image. This is the cheapest test of the whole client-side
   premise — if it holds, change 2 is the same code again.
   Done — `output/results_export.html` carries all 8 pattern PNGs (+ the powercurve, 9 total)
   inlined as base64, and the `Slate.replay` wiring for `wind_speed` survived the export intact.
2. [x] Apply change 2 and export. Confirm each box of the `columns` control hides its column group.
   Done — `overview_table` now ships every column unconditionally; the `columns_toggle` web cell
   hides them client-side, verified live (unchecking "force" hid exactly `min_force`/`av_force`/
   `max_force` in both header and matching row cells) and in the export (18 headers, toggler JS
   intact). It also turned up a wrinkle the plan didn't anticipate: the live table
   (`table.st-table`) and the exported one (`table.exp-table`) are different DOM structures, so the
   toggler targets both; and independent cells can evaluate in parallel, so the toggler cell reads
   `nrow(overview_df)` purely to force the dependency graph to run it after `overview_table`.

