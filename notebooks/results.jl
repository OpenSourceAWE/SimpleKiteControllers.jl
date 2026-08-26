try; import KaimonSlate; catch; error("This is a Kaimon Slate notebook — running it as plain Julia needs the KaimonSlate runtime in this environment. Add it with `import Pkg; Pkg.add(\"KaimonSlate\")`, or open it in Kaimon Slate."); end; KaimonSlate.standalone!(@__MODULE__; dir=@__DIR__)

#%% md id=intro
@md"""
# Simulation Results Reel-Out testing at Maasvlakte, NL
"""

#%% code id=columns_bind
@bind columns MultiCheckBox(["optimization", "force", "reel-out speeds", "performance"], String[];
                            label="Columns")

#%% md id=overview_table_heading
@md"""
### Overview of reel-out runs per wind speed

The test scenario:
- the kite is parked at zenith (keep it still above your head while in equilibrium)
- then it is steered to the left and takes a dive
- then it is steered towards the planned figure-of-eight that promises the highest power
- it flies figures of eight and reels out

The initial tether length is 150 m, the final length 380 m. The wind speed is the ground wind speed, measured at a height of 6 m, using a vertical wind profile law combining the exponential and logarithmic laws, matched to measurements at a near-shore site in Maasvlakte, NL.
"""

#%% code id=overview_table hidecode
using DataFrames

overview_lines = split(@asset("notebooks/overview.md"), '\n')
header = strip.(split(strip(overview_lines[1], '|'), '|'))
rows = [strip.(split(strip(l, '|'), '|')) for l in overview_lines[3:end] if !isempty(strip(l))]
overview_df = DataFrame([header[i] => [something(tryparse(Float64, r[i]), r[i]) for r in rows] for i in eachindex(header)])

align = Dict(:opt_requests => :center, :opts_installed => :center)
slate_table(overview_df; align)

#%% md id=columns_hint
@md"""
Click on one of the labels to show an additional column group, ctrl+click to deselect, shift+click for multi-select.
"""

#%% web id=columns_toggle controls=columns
@web(html"""
<!-- reads overview_df so this cell runs after overview_table, not just after the checkboxes -->
<span hidden>{{ size(overview_df, 1) }}</span>
""",
css"""
/* the exported table's own wrapper has no scroll of its own — mirrors the live table's .st-scroll */
.exp-tblwrap { overflow-x: auto; }
""",
js"""
const GROUPS = {
  optimization: ["opt_requests", "opts_installed"],
  force: ["min_force", "av_force", "max_force"],
  "reel-out speeds": ["v_ro_min", "v_ro_av", "v_ro_max"],
  performance: ["total_time", "rt_factor"],
};
const groupOf = name => Object.keys(GROUPS).find(g => GROUPS[g].includes(name));
const apply = checked => {
  document.querySelectorAll("table.st-table, table.exp-table").forEach(table => {
    table.querySelectorAll("thead th").forEach((th, i) => {
      const g = groupOf(th.dataset.label || th.textContent.trim());
      if (!g) return;
      const show = checked.includes(g);
      th.hidden = !show;
      table.querySelectorAll("tbody tr").forEach(tr => {
        const td = tr.children[i];
        td && (td.hidden = !show);
      });
    });
  });
};
apply({{ columns }});
if (window.Slate && !Slate.isLive()) {
  Slate.replay.hosts("columns").forEach(h => {
    Slate.replay.enable(h, true);
    Slate.replay.listen(h, () => apply(Slate.replay.read(h)));
  });
}
""")

#%% md id=powercurve_hint
@md"""
The average mechanical power during reel-out, the force, and the speed are shown in the following plot. At about <span style="white-space: nowrap">9 m/s</span> the maximal force of 8400 N is reached. From this wind speed onwards the force is limited by reeling out faster.
"""

#%% md id=powercurve_plot
@md"""
![power curve](/n/results/asset/notebooks/images/powercurve.png)
"""

#%% md id=wind_speed_hint
@md"""
The optimal trajectory of the kite, the shape of the figure of eight, depends on the tether length and on the wind speed. Below, use the slider to adjust the ground wind speed, and see the shape of the desired and actual trajectory the kite flies. For each figure-of-eight, the AWETrim quasi-static optimizer calculated the optimal trajectory.
"""

#%% code id=wind_speed_bind
@bind wind_speed Slider(3:10; default=3, label="Wind")

#%% web id=pattern_plot controls=wind_speed
@web(html"""
<div id="pattern">
  <img data-v="3" src="/n/results/asset/notebooks/images/pattern_v03.png" alt="flight pattern, 3 m/s">
  <img data-v="4" src="/n/results/asset/notebooks/images/pattern_v04.png" alt="flight pattern, 4 m/s">
  <img data-v="5" src="/n/results/asset/notebooks/images/pattern_v05.png" alt="flight pattern, 5 m/s">
  <img data-v="6" src="/n/results/asset/notebooks/images/pattern_v06.png" alt="flight pattern, 6 m/s">
  <img data-v="7" src="/n/results/asset/notebooks/images/pattern_v07.png" alt="flight pattern, 7 m/s">
  <img data-v="8" src="/n/results/asset/notebooks/images/pattern_v08.png" alt="flight pattern, 8 m/s">
  <img data-v="9" src="/n/results/asset/notebooks/images/pattern_v09.png" alt="flight pattern, 9 m/s">
  <img data-v="10" src="/n/results/asset/notebooks/images/pattern_v10.png" alt="flight pattern, 10 m/s">
</div>
""",
css"""
#pattern img { display: block; margin: 0 auto; max-width: 100%; height: auto; }
#pattern img[hidden] { display: none; }
""",
js"""
const pick = v => document.querySelectorAll("#pattern img")
    .forEach(im => im.hidden = Number(im.dataset.v) !== Number(v));
pick({{ wind_speed }});
function wire() {
  if (!(window.Slate && !Slate.isLive())) return;
  // wind_speed is `controls=` for two cells (pattern_plot, time_series_plot), so the export
  // renders TWO independent slider copies. Deferred to DOMContentLoaded so both exist by the
  // time hosts() runs — this script sits earlier in the document than time_series_plot's copy,
  // and hosts() only sees what has been parsed into the DOM so far.
  const hosts = Slate.replay.hosts("wind_speed");
  // Exported sliders freeze their ".exp-ctl-val" readout at the export-time value: nothing updates
  // it unless the control is wired through Slate.replay.wire, and this one is wired by hand instead.
  const relabel = (h, v) => {
    const ro = h.parentElement && h.parentElement.querySelector(".exp-ctl-val");
    ro && (ro.textContent = v + " m/s");
  };
  // Keep every copy's handle position and readout in sync, not just the one the reader moved.
  const sync = v => hosts.forEach(h => { h.value = v; relabel(h, v); });
  hosts.forEach(h => {
    Slate.replay.enable(h, true);
    Slate.replay.listen(h, () => { const v = Slate.replay.read(h); pick(v); sync(v); });
  });
  sync(Slate.replay.read(hosts[0]));
}
document.readyState === "loading" ?
  document.addEventListener("DOMContentLoaded", wire) : wire();
""")

#%% md id=time_series_hint
@md"""
The time series below show, for the same wind speed selected above, the cross-track error, elevation,
course/heading tracking, steering, tether force and length, reel-out speed, depower, and the entry
state machine over the whole run. The first subplot, `d ', shows the distance between the actual and planned trajectory in degrees.
"""

#%% web id=time_series_plot controls=wind_speed
@web(html"""
<div id="time_series">
  <img data-v="3" src="/n/results/asset/notebooks/images/time_series_v03.png" alt="time series, 3 m/s">
  <img data-v="4" src="/n/results/asset/notebooks/images/time_series_v04.png" alt="time series, 4 m/s">
  <img data-v="5" src="/n/results/asset/notebooks/images/time_series_v05.png" alt="time series, 5 m/s">
  <img data-v="6" src="/n/results/asset/notebooks/images/time_series_v06.png" alt="time series, 6 m/s">
  <img data-v="7" src="/n/results/asset/notebooks/images/time_series_v07.png" alt="time series, 7 m/s">
  <img data-v="8" src="/n/results/asset/notebooks/images/time_series_v08.png" alt="time series, 8 m/s">
  <img data-v="9" src="/n/results/asset/notebooks/images/time_series_v09.png" alt="time series, 9 m/s">
  <img data-v="10" src="/n/results/asset/notebooks/images/time_series_v10.png" alt="time series, 10 m/s">
</div>
""",
css"""
#time_series img { display: block; margin: 0 auto; max-width: 100%; height: auto; }
#time_series img[hidden] { display: none; }
""",
js"""
const pick = v => document.querySelectorAll("#time_series img")
    .forEach(im => im.hidden = Number(im.dataset.v) !== Number(v));
pick({{ wind_speed }});
function wire() {
  if (!(window.Slate && !Slate.isLive())) return;
  // pattern_plot's listener keeps every "wind_speed" host's handle position and readout in
  // sync; this cell only needs to swap its own image on any of them changing.
  Slate.replay.hosts("wind_speed").forEach(h => {
    Slate.replay.enable(h, true);
    Slate.replay.listen(h, () => pick(Slate.replay.read(h)));
  });
}
document.readyState === "loading" ?
  document.addEventListener("DOMContentLoaded", wire) : wire();
""")

#%% md id=power_hint
@md"""
The plots below show, for the same wind speed selected above, the tether force, reel-out speed and
mechanical power over the whole run, and the cumulative mechanical energy.
"""

#%% web id=power_plot controls=wind_speed
@web(html"""
<div id="power">
  <img data-v="3" src="/n/results/asset/notebooks/images/power_v03.png" alt="power, 3 m/s">
  <img data-v="4" src="/n/results/asset/notebooks/images/power_v04.png" alt="power, 4 m/s">
  <img data-v="5" src="/n/results/asset/notebooks/images/power_v05.png" alt="power, 5 m/s">
  <img data-v="6" src="/n/results/asset/notebooks/images/power_v06.png" alt="power, 6 m/s">
  <img data-v="7" src="/n/results/asset/notebooks/images/power_v07.png" alt="power, 7 m/s">
  <img data-v="8" src="/n/results/asset/notebooks/images/power_v08.png" alt="power, 8 m/s">
  <img data-v="9" src="/n/results/asset/notebooks/images/power_v09.png" alt="power, 9 m/s">
  <img data-v="10" src="/n/results/asset/notebooks/images/power_v10.png" alt="power, 10 m/s">
</div>
""",
css"""
#power img { display: block; margin: 0 auto; max-width: 100%; height: auto; }
#power img[hidden] { display: none; }
""",
js"""
const pick = v => document.querySelectorAll("#power img")
    .forEach(im => im.hidden = Number(im.dataset.v) !== Number(v));
pick({{ wind_speed }});
function wire() {
  if (!(window.Slate && !Slate.isLive())) return;
  // pattern_plot's listener keeps every "wind_speed" host's handle position and readout in
  // sync; this cell only needs to swap its own image on any of them changing.
  Slate.replay.hosts("wind_speed").forEach(h => {
    Slate.replay.enable(h, true);
    Slate.replay.listen(h, () => pick(Slate.replay.read(h)));
  });
}
document.readyState === "loading" ?
  document.addEventListener("DOMContentLoaded", wire) : wire();
""")

#%% md id=aerodynamics_hint
@md"""
The plots below show, for the same wind speed selected above, the angle of attack (at the wing
centre and the span mean), the wing and effective lift-to-drag ratio, and the apparent wind and
kite speed over the whole run.
"""

#%% web id=aerodynamics_plot controls=wind_speed
@web(html"""
<div id="aerodynamics">
  <img data-v="3" src="/n/results/asset/notebooks/images/aerodynamics_v03.png" alt="aerodynamics, 3 m/s">
  <img data-v="4" src="/n/results/asset/notebooks/images/aerodynamics_v04.png" alt="aerodynamics, 4 m/s">
  <img data-v="5" src="/n/results/asset/notebooks/images/aerodynamics_v05.png" alt="aerodynamics, 5 m/s">
  <img data-v="6" src="/n/results/asset/notebooks/images/aerodynamics_v06.png" alt="aerodynamics, 6 m/s">
  <img data-v="7" src="/n/results/asset/notebooks/images/aerodynamics_v07.png" alt="aerodynamics, 7 m/s">
  <img data-v="8" src="/n/results/asset/notebooks/images/aerodynamics_v08.png" alt="aerodynamics, 8 m/s">
  <img data-v="9" src="/n/results/asset/notebooks/images/aerodynamics_v09.png" alt="aerodynamics, 9 m/s">
  <img data-v="10" src="/n/results/asset/notebooks/images/aerodynamics_v10.png" alt="aerodynamics, 10 m/s">
</div>
""",
css"""
#aerodynamics img { display: block; margin: 0 auto; max-width: 100%; height: auto; }
#aerodynamics img[hidden] { display: none; }
""",
js"""
const pick = v => document.querySelectorAll("#aerodynamics img")
    .forEach(im => im.hidden = Number(im.dataset.v) !== Number(v));
pick({{ wind_speed }});
function wire() {
  if (!(window.Slate && !Slate.isLive())) return;
  // pattern_plot's listener keeps every "wind_speed" host's handle position and readout in
  // sync; this cell only needs to swap its own image on any of them changing.
  Slate.replay.hosts("wind_speed").forEach(h => {
    Slate.replay.enable(h, true);
    Slate.replay.listen(h, () => pick(Slate.replay.read(h)));
  });
}
document.readyState === "loading" ?
  document.addEventListener("DOMContentLoaded", wire) : wire();
""")

#%% md id=acknowledgements
@md"""
These results were achieved using the following research software:

- [SimpleKiteControllers.jl](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl), path-following kite control software by Uwe Fechner, Delft, 2026
- [AWETrim](https://github.com/awegroup/AWETrim/tree/develop) was used to provide optimal flight paths, depending on the inflow conditions, written by Oriol Canyon, Delft, 2026
- [V3Kite.jl](https://github.com/OpenSourceAWE/V3Kite.jl) was used as detailed, validated
kite model of the V3 kite of TU Delft. It was developed by Jelle Poland, Rotterdam, The Netherlands and Bart van de Lint, Delft, The Netherlands, 2026
"""

# ╔═╡ Slate.env · notebook packages (auto-maintained — manage via the package panel)
#   DataFrames 1.8.2 a93c6f00-e57d-5684-b7b6-d8193f3e46c0
# ╚═╡
# ╔═╡ Slate.config · per-notebook settings (Settings panel)
#   docid = b94b3a69-a369-4ec8-bcb0-c8230e4ff1d3
# ╚═╡
