try; import KaimonSlate; catch; error("This is a Kaimon Slate notebook — running it as plain Julia needs the KaimonSlate runtime in this environment. Add it with `import Pkg; Pkg.add(\"KaimonSlate\")`, or open it in Kaimon Slate."); end; KaimonSlate.standalone!(@__MODULE__; dir=@__DIR__)

#%% md id=intro
@md"""
# Simulation Results Reel-Out testing at Maasvlakte, NL
"""

#%% code id=columns_bind
@bind columns MultiCheckBox(["optimization", "force", "reel-out", "performance"],
                            ["optimization", "force", "reel-out", "performance"];
                            label="Columns")

#%% code id=overview_table
using DataFrames

overview_lines = split(@asset("notebooks/overview.md"), '\n')
header = strip.(split(strip(overview_lines[1], '|'), '|'))
rows = [strip.(split(strip(l, '|'), '|')) for l in overview_lines[3:end] if !isempty(strip(l))]
overview_df = DataFrame([header[i] => [something(tryparse(Float64, r[i]), r[i]) for r in rows] for i in eachindex(header)])

"optimization" in columns || select!(overview_df, Not([:opt_requests, :opts_installed]))
"force" in columns || select!(overview_df, Not([:min_force, :av_force, :max_force]))
"reel-out" in columns || select!(overview_df, Not(names(overview_df, r"^v_ro")))
"performance" in columns || select!(overview_df, Not([:total_time, :rt_factor]))

align = Dict(c => :center for c in (:opt_requests, :opts_installed) if c in propertynames(overview_df))
slate_table(overview_df; align)

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
  Slate.replay.hosts("wind_speed").forEach(h => {
    Slate.replay.enable(h, true);
    Slate.replay.listen(h, () => pick(Slate.replay.read(h)));
  });
}
""")

# ╔═╡ Slate.config · per-notebook settings (Settings panel)
#   docid = b94b3a69-a369-4ec8-bcb0-c8230e4ff1d3
# ╚═╡
