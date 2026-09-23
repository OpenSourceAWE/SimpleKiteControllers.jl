try; import KaimonSlate; catch; error("This is a Kaimon Slate notebook — running it as plain Julia needs the KaimonSlate runtime in this environment. Add it with `import Pkg; Pkg.add(\"KaimonSlate\")`, or open it in Kaimon Slate."); end; KaimonSlate.standalone!(@__MODULE__; dir=@__DIR__)

#%% web id=intro
@web(html"""
<div class="md">
<h1 class="intro-title">Simulation Results Reel-Out testing at Cabauw, NL</h1>
</div>
""",
css"""
/* the export's auto-generated doc header duplicates this cell's own title — this IS the title */
.exp-titleblock { display: none; }
""")

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

The initial tether length is 150 m, the final length 380 m. The inflow represents the yearly-averaged
onshore wind conditions in the Netherlands, based on one full year (2011) of 10-minute wind data from the
213 m tall KNMI meteorological mast at Cabauw, an inland site far from the coast. The vertical wind profile
is a power law with the exponent p = 0.234, fitted to the yearly-averaged wind speeds measured at the mast;
this gives a much stronger wind shear than at the near-shore Maasvlakte site, reflecting the higher surface
roughness of the inland terrain. The wind speed is the ground wind speed, measured at a height of 6 m.
The inflow is steady and uniform, without turbulence. A ground-station with 20 kW nominal power and
30 kW peak power is assumed. The nominal reel-out speed is 3.5 m/s, the maximum force 8400 N.
"""

#%% code id=overview_table hidecode
using DataFrames

overview_lines = split(@asset("notebooks/overview_cabauw.md"), '\n')
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
The average mechanical power during reel-out, the force, and the speed are shown in the following plot. At about <span style="white-space: nowrap">5.75 m/s</span> the reel-out speed reaches its limit of 3.5 m/s and the average power levels off at about 20 kW. From this wind speed onwards the depower setting is increased with the wind speed.
"""

#%% md id=powercurve_plot
@md"""
![power curve](/n/results_cabauw/asset/notebooks/images/cabauw/powercurve.png)
"""

#%% md id=wind_speed_hint
@md"""
The optimal trajectory of the kite, the shape of the figure of eight, depends on the tether length and on the wind speed. Below, select the ground wind speed [m/s] to see the shape of the desired and actual trajectory the kite flies. For each figure-of-eight, the AWETrim quasi-static optimizer calculated the optimal trajectory.
"""

#%% code id=wind_speed_bind
# A Select, not a Slider: the runs are unevenly spaced (5.5, 5.75, 6.25 m/s), and Slider only takes ranges.
@bind wind_speed Select(["3", "4", "5", "5.5", "5.75", "6", "6.25", "7", "8", "9", "10"], "3";
                        label="Wind [m/s]")

#%% web id=pattern_plot controls=wind_speed
@web(html"""
<div id="pattern">
  <img data-v="3" src="/n/results_cabauw/asset/notebooks/images/cabauw/pattern_v03.png" alt="flight pattern, 3 m/s">
  <img data-v="4" src="/n/results_cabauw/asset/notebooks/images/cabauw/pattern_v04.png" alt="flight pattern, 4 m/s">
  <img data-v="5" src="/n/results_cabauw/asset/notebooks/images/cabauw/pattern_v05.png" alt="flight pattern, 5 m/s">
  <img data-v="5.5" src="/n/results_cabauw/asset/notebooks/images/cabauw/pattern_v05.5.png" alt="flight pattern, 5.5 m/s">
  <img data-v="5.75" src="/n/results_cabauw/asset/notebooks/images/cabauw/pattern_v05.75.png" alt="flight pattern, 5.75 m/s">
  <img data-v="6" src="/n/results_cabauw/asset/notebooks/images/cabauw/pattern_v06.png" alt="flight pattern, 6 m/s">
  <img data-v="6.25" src="/n/results_cabauw/asset/notebooks/images/cabauw/pattern_v06.25.png" alt="flight pattern, 6.25 m/s">
  <img data-v="7" src="/n/results_cabauw/asset/notebooks/images/cabauw/pattern_v07.png" alt="flight pattern, 7 m/s">
  <img data-v="8" src="/n/results_cabauw/asset/notebooks/images/cabauw/pattern_v08.png" alt="flight pattern, 8 m/s">
  <img data-v="9" src="/n/results_cabauw/asset/notebooks/images/cabauw/pattern_v09.png" alt="flight pattern, 9 m/s">
  <img data-v="10" src="/n/results_cabauw/asset/notebooks/images/cabauw/pattern_v10.png" alt="flight pattern, 10 m/s">
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
  // wind_speed is `controls=` for several cells, so the export renders one dropdown copy per
  // cell. Deferred to DOMContentLoaded so all copies exist by the time hosts() runs.
  const hosts = Slate.replay.hosts("wind_speed");
  // Keep every copy's selection in sync, not just the one the reader changed.
  const sync = v => hosts.forEach(h => { h.value = v; });
  hosts.forEach(h => {
    Slate.replay.enable(h, true);
    Slate.replay.listen(h, () => { const v = Slate.replay.read(h); pick(v); sync(v); });
  });
  sync(Slate.replay.read(hosts[0]));
}
document.readyState === "loading" ?
  document.addEventListener("DOMContentLoaded", wire) : wire();
""")

#%% md id=path3d_hint
@md"""
The same run in 3D: the flown path in the world frame, coloured by the mechanical winch power, with the ground track underneath and the straight line to the ground station. Drag to rotate, scroll to zoom.
"""

#%% web id=path3d_plot controls=wind_speed
@web(html"""
<!-- One iframe, its src swapped per wind speed, rather than eleven: each page is a ~4 MB
     WebGL app, and browsers cap the number of live WebGL contexts (8-16). The files
     are NOT inlined by the export — publish.jl copies them next to the page. -->
<div id="path3d">
  <iframe id="path3d-frame" loading="lazy" title="3D flight path"></iframe>
  <p id="path3d-caption"></p>
</div>
""",
css"""
/* the page inside scales its 1000x780 canvas to the iframe width, so the height follows the same ratio */
#path3d iframe { display: block; margin: 0 auto; width: 100%; max-width: 1000px; aspect-ratio: 1000 / 780; border: 0; }
#path3d-caption { text-align: center; font-size: 0.9em; color: #666; margin: 0.3em 0 0; }
""",
js"""
// Live Slate serves the notebook tree under /n/<id>/asset/; the export sits in a folder the
// files are copied into (SimulationResults/docs/cabauw/), so there the bare file name resolves.
const BASE = (window.Slate && Slate.isLive()) ? "/n/results_cabauw/asset/notebooks/images/cabauw/" : "";
// "5.75" -> "v05.75", "3" -> "v03": the scenario folder naming.
const tag = v => { const [i, f] = String(v).split("."); return "v" + i.padStart(2, "0") + (f ? "." + f : ""); };
const frame = document.getElementById("path3d-frame");
const caption = document.getElementById("path3d-caption");
const pick = v => {
  const src = BASE + "path_webgl_" + tag(v) + ".html";
  if (frame.getAttribute("src") !== src) frame.src = src;
  caption.textContent = "3D flight path, " + v + " m/s run";
};
pick({{ wind_speed }});
function wire() {
  if (!(window.Slate && !Slate.isLive())) return;
  Slate.replay.hosts("wind_speed").forEach(h => {
    Slate.replay.enable(h, true);
    Slate.replay.listen(h, () => pick(Slate.replay.read(h)));
  });
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
  <img data-v="3" src="/n/results_cabauw/asset/notebooks/images/cabauw/time_series_v03.png" alt="time series, 3 m/s">
  <img data-v="4" src="/n/results_cabauw/asset/notebooks/images/cabauw/time_series_v04.png" alt="time series, 4 m/s">
  <img data-v="5" src="/n/results_cabauw/asset/notebooks/images/cabauw/time_series_v05.png" alt="time series, 5 m/s">
  <img data-v="5.5" src="/n/results_cabauw/asset/notebooks/images/cabauw/time_series_v05.5.png" alt="time series, 5.5 m/s">
  <img data-v="5.75" src="/n/results_cabauw/asset/notebooks/images/cabauw/time_series_v05.75.png" alt="time series, 5.75 m/s">
  <img data-v="6" src="/n/results_cabauw/asset/notebooks/images/cabauw/time_series_v06.png" alt="time series, 6 m/s">
  <img data-v="6.25" src="/n/results_cabauw/asset/notebooks/images/cabauw/time_series_v06.25.png" alt="time series, 6.25 m/s">
  <img data-v="7" src="/n/results_cabauw/asset/notebooks/images/cabauw/time_series_v07.png" alt="time series, 7 m/s">
  <img data-v="8" src="/n/results_cabauw/asset/notebooks/images/cabauw/time_series_v08.png" alt="time series, 8 m/s">
  <img data-v="9" src="/n/results_cabauw/asset/notebooks/images/cabauw/time_series_v09.png" alt="time series, 9 m/s">
  <img data-v="10" src="/n/results_cabauw/asset/notebooks/images/cabauw/time_series_v10.png" alt="time series, 10 m/s">
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
  <img data-v="3" src="/n/results_cabauw/asset/notebooks/images/cabauw/power_v03.png" alt="power, 3 m/s">
  <img data-v="4" src="/n/results_cabauw/asset/notebooks/images/cabauw/power_v04.png" alt="power, 4 m/s">
  <img data-v="5" src="/n/results_cabauw/asset/notebooks/images/cabauw/power_v05.png" alt="power, 5 m/s">
  <img data-v="5.5" src="/n/results_cabauw/asset/notebooks/images/cabauw/power_v05.5.png" alt="power, 5.5 m/s">
  <img data-v="5.75" src="/n/results_cabauw/asset/notebooks/images/cabauw/power_v05.75.png" alt="power, 5.75 m/s">
  <img data-v="6" src="/n/results_cabauw/asset/notebooks/images/cabauw/power_v06.png" alt="power, 6 m/s">
  <img data-v="6.25" src="/n/results_cabauw/asset/notebooks/images/cabauw/power_v06.25.png" alt="power, 6.25 m/s">
  <img data-v="7" src="/n/results_cabauw/asset/notebooks/images/cabauw/power_v07.png" alt="power, 7 m/s">
  <img data-v="8" src="/n/results_cabauw/asset/notebooks/images/cabauw/power_v08.png" alt="power, 8 m/s">
  <img data-v="9" src="/n/results_cabauw/asset/notebooks/images/cabauw/power_v09.png" alt="power, 9 m/s">
  <img data-v="10" src="/n/results_cabauw/asset/notebooks/images/cabauw/power_v10.png" alt="power, 10 m/s">
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
  <img data-v="3" src="/n/results_cabauw/asset/notebooks/images/cabauw/aerodynamics_v03.png" alt="aerodynamics, 3 m/s">
  <img data-v="4" src="/n/results_cabauw/asset/notebooks/images/cabauw/aerodynamics_v04.png" alt="aerodynamics, 4 m/s">
  <img data-v="5" src="/n/results_cabauw/asset/notebooks/images/cabauw/aerodynamics_v05.png" alt="aerodynamics, 5 m/s">
  <img data-v="5.5" src="/n/results_cabauw/asset/notebooks/images/cabauw/aerodynamics_v05.5.png" alt="aerodynamics, 5.5 m/s">
  <img data-v="5.75" src="/n/results_cabauw/asset/notebooks/images/cabauw/aerodynamics_v05.75.png" alt="aerodynamics, 5.75 m/s">
  <img data-v="6" src="/n/results_cabauw/asset/notebooks/images/cabauw/aerodynamics_v06.png" alt="aerodynamics, 6 m/s">
  <img data-v="6.25" src="/n/results_cabauw/asset/notebooks/images/cabauw/aerodynamics_v06.25.png" alt="aerodynamics, 6.25 m/s">
  <img data-v="7" src="/n/results_cabauw/asset/notebooks/images/cabauw/aerodynamics_v07.png" alt="aerodynamics, 7 m/s">
  <img data-v="8" src="/n/results_cabauw/asset/notebooks/images/cabauw/aerodynamics_v08.png" alt="aerodynamics, 8 m/s">
  <img data-v="9" src="/n/results_cabauw/asset/notebooks/images/cabauw/aerodynamics_v09.png" alt="aerodynamics, 9 m/s">
  <img data-v="10" src="/n/results_cabauw/asset/notebooks/images/cabauw/aerodynamics_v10.png" alt="aerodynamics, 10 m/s">
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
## Acknowledgements

This work has been supported by the MERIDIONAL project, which receives funding from the European Union’s Horizon Europe Program under the grant agreement no. [101084216](https://doi.org/10.3030/101084216). The opinions expressed in this document reflect only the author’s view and reflects in no way the European Commission’s opinions. The European Commission is not responsible for any use that may be made of the information it contains.

## These results were achieved using the following research software:

- [SimpleKiteControllers.jl](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl), path-following kite control software by Uwe Fechner, Delft, 2026
- [AWETrim](https://github.com/awegroup/AWETrim/tree/develop) was used to provide optimal flight paths, depending on the inflow conditions, written by Oriol Canyon, Delft, 2026
- [V3Kite.jl](https://github.com/OpenSourceAWE/V3Kite.jl) was used as detailed, validated
kite model of the V3 kite of TU Delft. It was developed by Jelle Poland, Rotterdam, The Netherlands and Bart van de Lint, Delft, The Netherlands, 2026
- [AtmosphericModels.jl](https://github.com/OpenSourceAWE/AtmosphericModels.jl) provided the wind profile of the Cabauw site
"""

# ╔═╡ Slate.env · notebook packages (auto-maintained — manage via the package panel)
#   DataFrames 1.8.2 a93c6f00-e57d-5684-b7b6-d8193f3e46c0
# ╚═╡
# ╔═╡ Slate.config · per-notebook settings (Settings panel)
#   docid = 2f4e99cf-0031-4bd0-93be-db5388f0142b
# ╚═╡
