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

3. [x] Hide the `overview_table` cell's source in the export. It rendered as a full syntax-highlighted
   Julia listing directly above the table — noise for a results page with no Julia behind it.
   Done — tagged the cell `#%% code id=overview_table hidecode`
   ([notebooks/results.jl:13](notebooks/results.jl#L13)), KaimonSlate's existing tag for this
   (`_KNOWN_TAGS`, `src/engine.jl:294`; checked in the export at `src/server_export.jl:2537`).
   Verified in a fresh export: the `overview_lines = split(...)`/`DataFrame(...)` listing no longer
   appears, while the table (18 headers), the 9 inlined images and the toggler script are unchanged.

4. [x] Make the exported table scroll horizontally. With Change 2 shipping all 18 columns
   unconditionally, the table is reliably wider than the page, and KaimonSlate's own export CSS for
   its wrapper — `.exp-tblwrap{width:fit-content;max-width:100%;margin:10px auto}`
   (`src/server_export.jl:1470`) — clamps the box's width but never sets `overflow-x`, so there is
   nothing to scroll within; content just overflows. The live notebook doesn't have this problem
   because its own table wrapper already carries `.st-scroll{overflow-x:auto;max-width:100%}`
   (`src/assets/notebook.css:802`) — the export's wrapper is simply missing that half of the same
   rule.
   Done — added a `css"""..."""` pane to `columns_toggle`
   ([notebooks/results.jl:24-33](notebooks/results.jl#L24-L33)) with
   `.exp-tblwrap { overflow-x: auto; }`, mirroring the live wrapper's own rule. Web-cell CSS ships
   as a plain global `<style>` tag (`src/widgets.jl:2056`), not shadow-scoped, so it can target an
   ancestor class outside the cell's own markup; it lands later in the page than Slate's built-in
   rule, so at equal specificity it wins the cascade (verified by source position in the export).
   Verified in a fresh export: both `.exp-tblwrap` rules are present, ours last, and the table (18
   headers), 9 images and hidden source from steps 1–3 are all still unaffected.

5. [x] Make the wind-speed slider's number readout dynamic and carry the unit. The export renders a
   frozen `<output class="exp-ctl-val">3</output>` beside every slider (`_export_control_html`,
   `src/server_export.jl:519`); `Slate.replay.wire` keeps that readout live by updating it itself
   (`src/server_export.jl:1291`), but `wind_speed` is wired by hand in `pattern_plot` instead of
   through `wire()`, so nothing was touching it — the "3" never moved and never carried units.
   Done — added a `relabel(h, v)` helper in `pattern_plot`'s JS
   ([notebooks/results.jl:89-104](notebooks/results.jl#L89-L104)) that writes `v + " m/s"` into the
   host's sibling `.exp-ctl-val`, called once on load (so "3" becomes "3 m/s" immediately) and again
   inside the existing `Slate.replay.listen` callback, next to `pick`.
   Verified end-to-end rather than by inspecting markup: fetched the real export, loaded it into an
   iframe in the live browser tab, and drove its actual script — readout read "3 m/s" on load;
   after simulating the slider moving to 7, the readout updated to "7 m/s" and the pattern image
   switched to `data-v="7"` in the same event.

## TODO
- [x] Publish the static html page on GitHub, using the repo https://github.com/OpenSourceAWE/SimulationResults

Done — copied `output/results_export.html` to
[SimulationResults/docs/index.html](https://github.com/OpenSourceAWE/SimulationResults/blob/main/docs/index.html),
committed and pushed (`7e97a2e`), and enabled GitHub Pages on that repo (source: `main`
branch, `/docs` folder). Live at <https://opensourceawe.github.io/SimulationResults/>.

- [x] Add the time_series_vXXX plots from the notebooks folder to the notebook in the same way as the pattern_vXXX plots, using the same wind speed from the slider.

Done — added `time_series_hint` (md) and `time_series_plot` (web) cells to
[notebooks/results.jl](notebooks/results.jl), right after `pattern_plot`. `time_series_plot` is the
same idiom as change 1: eight literal `<img>` tags (`time_series_v03.png` … `time_series_v10.png`),
toggled by a `pick(v)` that hides all but the selected `data-v`, with its own
`Slate.replay.hosts("wind_speed")` listener — a second, independent host on the same control,
alongside `pattern_plot`'s (which already keeps the slider's `.exp-ctl-val` readout live, so this
cell only needs to swap its own image, not repeat `relabel`).
Verified live (all 13 cells `fresh`, browser console clean via `slate_diag`) and in a fresh export:
`output/results_export.html` grew to 17 inlined `data:image/png` URIs (8 pattern + 8 time-series +
the powercurve) and carries two independent `Slate.replay.hosts("wind_speed")` listeners, one per
plot.

- [x] Fix step 5's readout wiring for the now-shared `wind_speed` control: with TWO cells declaring
  `controls=wind_speed`, the export renders TWO independent slider copies (one embedded per
  consuming cell), and `pattern_plot`'s original wiring only ever discovered one of them.

  Root cause, found by loading the real export into an iframe and driving it (same method as step
  5): each web cell's inline `<script>` runs as the parser reaches it, so `pattern_plot`'s script —
  positioned before `time_series_plot` in the document — called `Slate.replay.hosts("wind_speed")`
  before `time_series_plot`'s slider copy existed in the DOM at all. It therefore attached its
  `enable`/`relabel`/`listen` to only the first copy. Confirmed with the iframe test: dragging the
  *second* slider swapped the time-series image but left the pattern image, both readouts, and the
  first slider's handle all stale.

  Done — both cells now defer their `Slate.replay.hosts(...)` wiring to `DOMContentLoaded` (running
  immediately instead if `document.readyState` is already past `"loading"`), so both slider copies
  exist by the time either cell looks for hosts. `pattern_plot`'s `sync(v)` now also writes every
  host's `.value` (not just the one that changed) alongside its readout, so both copies' handles
  and text move together regardless of which one the reader drags.
  Verified with the same export+iframe method as step 5, this time dragging EACH copy in turn:
  moving either slider now moves both handles to the same value, both `.exp-ctl-val` readouts read
  e.g. "9 m/s", and both the pattern and time-series images switch to the matching `data-v`.

- [x] Add the plots starting with power_ in the same way

Done — added `power_hint` (md) and `power_plot` (web) cells to
[notebooks/results.jl](notebooks/results.jl), right after `time_series_plot`. Same idiom as the
other two: eight literal `<img>` tags (`power_v03.png` … `power_v10.png`), toggled by `pick(v)`, with
its own deferred `Slate.replay.hosts("wind_speed")` listener that only swaps its own image (`sync`
and `.exp-ctl-val` upkeep already belongs to `pattern_plot`).
Verified live (all 15 cells `fresh`, browser console clean via `slate_diag`) and in a fresh export
fetched straight from `/api/results/export.html`: `output/results_export.html` grew to 25 inlined
`data:image/png` URIs (8 pattern + 8 time-series + 8 power + the powercurve) and carries three
independent `Slate.replay.hosts("wind_speed")` listeners, one per plot.

- [x] Add the plots starting with aerodynamics_ in the same way

Done — added `aerodynamics_hint` (md) and `aerodynamics_plot` (web) cells to
[notebooks/results.jl](notebooks/results.jl), right after `power_plot`. Same idiom again: eight
literal `<img>` tags (`aerodynamics_v03.png` … `aerodynamics_v10.png`), toggled by `pick(v)`, with
its own deferred `Slate.replay.hosts("wind_speed")` listener that only swaps its own image.
Verified live (all 17 cells `fresh`, browser console clean via `slate_diag`) and in a fresh export
fetched from `/api/results/export.html`: `output/results_export.html` grew to 33 inlined
`data:image/png` URIs (8 pattern + 8 time-series + 8 power + 8 aerodynamics + the powercurve) and
carries four independent `Slate.replay.hosts("wind_speed")` listeners, one per plot.

- [x] Can you add a text with the following content at the end:

These results were achieved using the following research software:

- [SimpleKiteControllers.jl](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl), path-following kite control software by Uwe Fechner, Delft, 2026
- [AWETrim](https://github.com/awegroup/AWETrim/tree/develop) was used to provide optimal flight paths, depending on the inflow conditions, written by Oriol Canyon, Delft, 2026
- [V3Kite.jl](https://github.com/OpenSourceAWE/V3Kite.jl) was used as detailed, validated
kite model of the V3 kite of TU Delft. It was developed by Jelle Poland, Rotterdam, The Netherlands and Bart van de Lint, Delft, The Netherlands, 2026

Done — added an `acknowledgements` md cell to [notebooks/results.jl](notebooks/results.jl) at the
very end, right before the `Slate.config` marker. Verified live (18 cells, `fresh`) and in a fresh
export: the text and all three GitHub links render correctly in `output/results_export.html`.
