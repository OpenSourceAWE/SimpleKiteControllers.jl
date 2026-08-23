try; import KaimonSlate; catch; error("This is a Kaimon Slate notebook — running it as plain Julia needs the KaimonSlate runtime in this environment. Add it with `import Pkg; Pkg.add(\"KaimonSlate\")`, or open it in Kaimon Slate."); end; KaimonSlate.standalone!(@__MODULE__; dir=@__DIR__)

#%% md id=intro
@md"""
# Simulation Results Reel-Out testing at Maasvlakte, NL
"""

#%% code id=show_optimization_bind
@bind show_optimization Checkbox(true; label="Show optimization columns")

#%% code id=show_force_bind
@bind show_force Checkbox(true; label="Show force columns")

#%% code id=show_reelout_bind
@bind show_reelout Checkbox(true; label="Show reel-out speed columns")

#%% code id=show_performance_bind
@bind show_performance Checkbox(true; label="Show performance columns")

#%% code id=overview_table
using DataFrames

overview_path = expanduser("~/repos/SimulationResults/scenarios/overview.md")
overview_lines = split(readfile(overview_path), '\n')
header = strip.(split(strip(overview_lines[1], '|'), '|'))
rows = [strip.(split(strip(l, '|'), '|')) for l in overview_lines[3:end] if !isempty(strip(l))]
overview_df = DataFrame([header[i] => [something(tryparse(Float64, r[i]), r[i]) for r in rows] for i in eachindex(header)])

show_optimization || select!(overview_df, Not([:opt_requests, :opts_installed]))
show_force || select!(overview_df, Not([:min_force, :av_force, :max_force]))
show_reelout || select!(overview_df, Not(names(overview_df, r"^v_ro")))
show_performance || select!(overview_df, Not([:total_time, :rt_factor]))

slate_table(overview_df; align=Dict(:opt_requests => :center, :opts_installed => :center))

# ╔═╡ Slate.config · per-notebook settings (Settings panel)
#   docid = b94b3a69-a369-4ec8-bcb0-c8230e4ff1d3
# ╚═╡
