try; import KaimonSlate; catch; error("This is a Kaimon Slate notebook — running it as plain Julia needs the KaimonSlate runtime in this environment. Add it with `import Pkg; Pkg.add(\"KaimonSlate\")`, or open it in Kaimon Slate."); end; KaimonSlate.standalone!(@__MODULE__; dir=@__DIR__)

#%% md id=intro
@md"""
# Simulation Results Reel-Out testing at Maasvlakte, NL
"""

#%% code id=columns_bind
@bind columns MultiCheckBox(["optimization", "force", "reel-out", "performance"], String[];
                            label="Columns")

#%% code id=overview_table hidecode
using DataFrames

overview_lines = split(@asset("notebooks/overview.md"), '\n')
header = strip.(split(strip(overview_lines[1], '|'), '|'))
rows = [strip.(split(strip(l, '|'), '|')) for l in overview_lines[3:end] if !isempty(strip(l))]
overview_df = DataFrame([header[i] => [something(tryparse(Float64, r[i]), r[i]) for r in rows] for i in eachindex(header)])

align = Dict(:opt_requests => :center, :opts_installed => :center)
slate_table(overview_df; align)

#%% web id=columns_toggle controls=columns
@web(html"""
<!-- reads overview_df so this cell runs after overview_table, not just after the checkboxes -->
<span hidden>{{ nrow(overview_df) }}</span>
""",
css"""
/* the exported table's own wrapper has no scroll of its own — mirrors the live table's .st-scroll */
.exp-tblwrap { overflow-x: auto; }
""",
js"""
const GROUPS = {
  optimization: ["opt_requests", "opts_installed"],
  force: ["min_force", "av_force", "max_force"],
  "reel-out": ["v_ro_min", "v_ro_av", "v_ro_max"],
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

#%% md id=powercurve_plot
@md"""
![power curve](/n/results/asset/notebooks/powercurve.png)
"""

#%% code id=wind_speed_bind
@bind wind_speed Slider(3:10; default=3, label="Wind speed [m/s]")

#%% web id=pattern_plot controls=wind_speed
@web(html"""
<div id="pattern">
  <img data-v="3" src="/n/results/asset/notebooks/pattern_v03.png" alt="flight pattern, 3 m/s">
  <img data-v="4" src="/n/results/asset/notebooks/pattern_v04.png" alt="flight pattern, 4 m/s">
  <img data-v="5" src="/n/results/asset/notebooks/pattern_v05.png" alt="flight pattern, 5 m/s">
  <img data-v="6" src="/n/results/asset/notebooks/pattern_v06.png" alt="flight pattern, 6 m/s">
  <img data-v="7" src="/n/results/asset/notebooks/pattern_v07.png" alt="flight pattern, 7 m/s">
  <img data-v="8" src="/n/results/asset/notebooks/pattern_v08.png" alt="flight pattern, 8 m/s">
  <img data-v="9" src="/n/results/asset/notebooks/pattern_v09.png" alt="flight pattern, 9 m/s">
  <img data-v="10" src="/n/results/asset/notebooks/pattern_v10.png" alt="flight pattern, 10 m/s">
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
  // Exported sliders freeze their ".exp-ctl-val" readout at the export-time value: nothing updates
  // it unless the control is wired through Slate.replay.wire, and this one is wired by hand instead.
  const relabel = (h, v) => {
    const ro = h.parentElement && h.parentElement.querySelector(".exp-ctl-val");
    ro && (ro.textContent = v + " m/s");
  };
  Slate.replay.hosts("wind_speed").forEach(h => {
    Slate.replay.enable(h, true);
    relabel(h, Slate.replay.read(h));
    Slate.replay.listen(h, () => { const v = Slate.replay.read(h); pick(v); relabel(h, v); });
  });
}
""")

# ╔═╡ Slate.config · per-notebook settings (Settings panel)
#   docid = b94b3a69-a369-4ec8-bcb0-c8230e4ff1d3
# ╚═╡
