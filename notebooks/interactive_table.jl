try; import KaimonSlate; catch; error("This is a Kaimon Slate notebook — running it as plain Julia needs the KaimonSlate runtime in this environment. Add it with `import Pkg; Pkg.add(\"KaimonSlate\")`, or open it in Kaimon Slate."); end; KaimonSlate.standalone!(@__MODULE__; dir=@__DIR__)

#%% md id=intro title
@md"""
# Controls that still work in a static export
"""

#%% md id=abstract abstract
@md"""
640 synthetic orders, five `@bind` controls, four tables and nine figures, including a calendar of
every day in the book, a sunburst, a world map, and a what-if analysis you drive with a slider. No
packages beyond `Dates`, `Random`, `Printf` and `Statistics`. `slate_table` and `echart` are Slate
builtins, and every control is an ordinary `@bind`.

The through-line is `@replay`. It decides which of these controls still works in a standalone export
with no Julia behind it, and which ones cannot.
"""

#%% code id=data
using Dates, Random

# Synthetic order book. Seeded, so the notebook, the export and every reader see the same numbers.
# The generator plants two patterns worth finding with the controls: one region runs a structurally
# thinner margin, and one category grows through the period while the rest are flat.
#
# Nothing here is `const`. A reactive cell can re-run, and Julia refuses to re-declare an existing
# global as constant, so `const` in a notebook works once and errors ever after.
CATALOG = [
    ("Sensors",    ["Thermistor probe", "Pressure cell", "Flow meter", "Hygrometer"],   180.0),
    ("Loggers",    ["Field logger 4ch", "Field logger 16ch", "Telemetry base"],         640.0),
    ("Optics",     ["Fibre coupler", "Bandpass filter", "Collimator"],                  310.0),
    ("Enclosures", ["Weather housing", "Rack shelf", "Pole mount"],                      95.0),
    ("Cabling",    ["Marine cable 50m", "Patch set", "Junction box"],                    48.0),
    ("Software",   ["Acquisition seat", "Analysis seat", "Support year"],               520.0),
]
REGIONS = ["Americas", "EMEA", "APAC", "LATAM"]

ORDERS = let rng = Random.Xoshiro(20260822), start = Date(2024, 7, 1)
    map(1:640) do i
        day = start + Day(rand(rng, 0:545))
        cat, products, base = CATALOG[rand(rng, 1:length(CATALOG))]
        region = REGIONS[rand(rng, 1:4)]

        # Products are listed cheapest-first within a category, so the line item's position sets its
        # price tier. Otherwise a 4-channel logger can come out dearer than the 16-channel one.
        p = rand(rng, 1:length(products))
        price = round(base * (0.7 + 0.35(p - 1)) * (0.9 + 0.2rand(rng)); digits = 2)

        # Software ramps with time; everything else is flat with noise.
        age = (day - start).value / 545
        demand = cat == "Software" ? 0.4 + 1.6age : 1.0
        units = max(1, round(Int, demand * (1 + rand(rng, 0:14)) * (0.6 + 0.8rand(rng))))

        # LATAM carries more freight and local duty, so it clears several points thinner.
        margin = round(clamp(0.34 + 0.09randn(rng) - (region == "LATAM" ? 0.11 : 0.0), 0.02, 0.62);
                       digits = 3)

        (order = "SO-$(1000 + i)", date = day, region, category = cat, product = products[p],
         units, price, revenue = round(units * price; digits = 2), margin)
    end
end

"$(length(ORDERS)) orders · $(minimum(o -> o.date, ORDERS)) → $(maximum(o -> o.date, ORDERS))"

#%% md id=s_builtin
@md"""
---
## The table

Returning any Tables.jl source from a cell renders it. `slate_table` is the same thing with the
presentation knobs exposed: per-column number formats, alignment, and an in-cell bar or heat strip
for a numeric column.

Sorting, the search box and paging come with it and cost nothing to author. Click a header to sort,
or type in the box to narrow 640 rows to the ones you want.
"""

#%% code id=table_builtin
slate_table(ORDERS;
            format = (price = :currency, revenue = :currency,
                      margin = (kind = :percent, digits = 1), units = :integer),
            align  = (order = :left, product = :left),
            viz    = (revenue = :bar, margin = :heat))

#%% md id=s_filter
@md"""
---
## Filtering it from controls

The search box filters the table against itself. To drive anything *else* from the same choice you
want `@bind`: the variable holds the live value, and every cell that reads it recomputes.

The two controls below are different kinds. **Region** is a `Select` over five options, a small
finite domain that a standalone export can precompute. **Order size** is a `RangeSlider`, whose two
thumbs bind one interval, so its domain is the ordered pairs `lo ≤ hi`. From 97 stops that comes to
4 753 pairs, not 9 409.

Both domains are finite, and the figure the band drives further down does survive an export. But the
band has nearly a thousand times as many positions as the region select, and that gap decides which
of the two the table below can follow once no Julia is left behind the page.
"""

#%% code id=aggregates
# `Statistics` belongs here, with the aggregates, and not in the summary table further down that first
# needed it. `median` is called by cells ABOVE that one, and a name arrives through the dependency
# graph: importing it late leaves every earlier reader with nothing to depend on, so whether they work
# comes down to whether the worker happens to be warm from a previous run.
using Printf, Statistics

# Every axis is fixed up front, from the whole dataset instead of the filtered slice. That keeps the
# aggregates below the same length for every control position, which is the condition `@replay`
# checks. An empty selection returns zeros, never a shorter vector.
CATEGORIES = first.(CATALOG)
MONTHS = let lo = minimum(o -> o.date, ORDERS), hi = maximum(o -> o.date, ORDERS)
    [Dates.format(d, "yyyy-mm") for d in firstdayofmonth(lo):Month(1):firstdayofmonth(hi)]
end
DAYS = minimum(o -> o.date, ORDERS):Day(1):maximum(o -> o.date, ORDERS)
MARGIN_BINS = 0.0:0.05:0.65

orders_in(region) = region == "All" ? ORDERS : filter(o -> o.region == region, ORDERS)
month_of(d) = Dates.format(Date(first(split(string(d), 'T'))), "yyyy-mm")

total(rs) = sum(o -> o.revenue, rs; init = 0.0)
profit(rs) = sum(o -> o.revenue * o.margin, rs; init = 0.0)
usd(x) = x ≥ 1e6 ? @sprintf("\$%.2fM", x / 1e6) : @sprintf("\$%.0fk", x / 1e3)

revenue_by_category(region) = (rs = orders_in(region);
                               [total(filter(o -> o.category == c, rs)) for c in CATEGORIES])
revenue_by_month(region) = (rs = orders_in(region);
                            [total(filter(o -> month_of(o.date) == m, rs)) for m in MONTHS])
revenue_in_band(lo, hi) = [total(filter(o -> lo ≤ o.revenue ≤ hi && o.category == c, ORDERS))
                           for c in CATEGORIES]

# A centred moving average, drawn as its own series instead of as smoothing applied to the data. A
# month's revenue is a discrete total: there is no value between February and March, so a curve
# through those points states something the data does not. The bars carry the months, this carries
# the trend, and the legend says which is which.
#
# Same length as its input, with NaN where the window runs off an end. An average over a partial
# window is a different statistic drawn as if it were the same one, and `@replay` stacks one
# fixed-shape slice per control position, so a shorter vector at the edges would not stack at all.
#
# Within a window, missing months are skipped instead of counted as zero. One product has no orders
# in some months and so no median margin there, and folding that in as a zero would drag the trend
# toward a number nobody observed.
function moving_average(v, w::Int = 3)
    h = w ÷ 2
    map(eachindex(v)) do i
        (i - h < firstindex(v) || i + h > lastindex(v)) && return NaN
        seen = [x for x in @view(v[i-h:i+h]) if !isnan(x)]
        isempty(seen) ? NaN : sum(seen) / length(seen)
    end
end

# Upper edges come from the range, never from `lo + step`. `0.3 + 0.05` and `MARGIN_BINS[8]` are not
# the same Float64, so the arithmetic version leaves adjacent bins overlapping by an ulp, and an
# order sitting on an edge is counted twice: 640 orders totalling 641.
function margin_histogram(region)
    rs = orders_in(region)
    n = length(MARGIN_BINS)
    [count(o -> MARGIN_BINS[i] ≤ o.margin < (i < n ? MARGIN_BINS[i+1] : Inf), rs) for i in 1:n]
end

# One pass into a Dict, then a lookup per day. 546 days × 640 orders is small, but the calendar below
# is the one aggregate `@replay` evaluates for every region, so it may as well be linear.
function orders_per_day(region)
    tally = Dict{Date,Int}()
    for o in orders_in(region)
        tally[o.date] = get(tally, o.date, 0) + 1
    end
    [get(tally, d, 0) for d in DAYS]
end

"$(length(CATEGORIES)) categories · $(length(MONTHS)) months · $(length(DAYS)) days"

#%% code id=region_ctl
# Two `@bind`s in one cell become a single combined control strip instead of two stacked widgets.
#
# The band is a `RangeSlider` because its ends are one decision, not two: a pair of plain sliders
# lets a reader drag the low thumb past the high one and see an empty table with no clue why.
@bind region Select(["All"; REGIONS]; label = "Region")
@bind band RangeSlider(0:250:24_000; default = (0, 24_000), label = "Order size")

#%% code id=selection
# One binding for "what the reader is looking at", so the KPI line, the table and the download
# button can't drift apart. They all read this instead of each rebuilding the predicate.
selected = filter(o -> band.lo ≤ o.revenue ≤ band.hi, orders_in(region))

"$(length(selected)) of $(length(ORDERS)) orders selected"

#%% md id=kpi
@md"""
### {{ region == "All" ? "All regions" : region }} · {{ usd(band.lo) }}–{{ usd(band.hi) }} orders

**{{ length(selected) }}** orders&ensp;·&ensp;**{{ usd(total(selected)) }}** revenue&ensp;·&ensp;**{{ usd(profit(selected)) }}** gross profit&ensp;·&ensp;**{{ length(selected) == 0 ? "—" : string(round(100 * profit(selected) / total(selected); digits = 1)) * "%" }}** blended margin

Prose is reactive too. Every number in that line is a `{{ }}` interpolation, and those reads join
the graph the same way a figure's do. Move either control and the sentence moves with it.
"""

#%% code id=table_filtered controls=region
slate_table(selected;
            format = (price = :currency, revenue = :currency,
                      margin = (kind = :percent, digits = 1), units = :integer),
            align  = (order = :left, product = :left),
            viz    = (revenue = :bar, margin = :heat))

#%% code id=download controls=region
# The way a reader leaves with a result: no cell to run, no filesystem to look in. The bytes are
# generated here and registered as a cell asset, so the button works live, offline and published.
let io = IOBuffer()
    println(io, join(keys(first(ORDERS)), ","))
    for o in selected
        println(io, join(values(o), ","))
    end
    download_button("orders-$(lowercase(region)).csv", String(take!(io));
                    label = "⬇  Download these $(length(selected)) rows as CSV")
end

#%% md id=s_table_replay
@md"""
---
## The table itself, offline

The table above runs on a live kernel: the filter is an ordinary `filter` over `ORDERS`, and moving
**region** re-runs the cell. A standalone export has no kernel, and the table still moves.

It does not ship a copy of itself per position. The rows travel once, as the union of every
position's rows, which for a region filter is the whole book. Each position then carries a short list
of indices into that union, saying which rows it shows and in what order. Five regions over 640 rows
comes to 6.4 KB of indices, on top of a table the page was already carrying.

Indices, and not a present-or-absent flag per row. A control that sorts its table is as ordinary as
one that filters it, and a flag cannot say where a row went. The control below is that case: the same
twenty rows at every position, with only the order changing.

The filtered table needed no changes to make any of this work. A `@replay` mark has to sit on the
expression that reads the control, and a table is usually not near one. `table_filtered` reads
`selected`, which reads `region`, so there is no expression to mark. But the cells in between are
already in the dependency graph, so the export composes their sources into a single closure, wraps it
in a `let` to keep their assignments local, and sweeps that.

What it will not do is choose between two controls. That table sits downstream of **region** and of
the order-size band, and sweeping the pair would mean 5 × 4 753 evaluations of the same chain. So
`region` drives it, because `controls=region` on that cell is the author naming the knob that belongs
to it. The band is **held**: it still drives the figure further down, which carries its own mark,
while the value it had when you exported is baked into this table's rows.
"""

#%% code id=sort_ctl
@bind sortby Select(["margin", "revenue", "date"]; label = "Sort by")

#%% code id=table_sorted controls=sortby
# Twenty fixed rows, ordered three ways. Every position shows the same rows, so nothing is filtered
# and the only thing this control changes is the order.
#
# `sortby` arrives as the value the cell sees, so it is compared against the option instead of being
# converted to a `Symbol`, which is also why the sort keys are written out instead of derived.
@replay(sortby, let thin = sort(ORDERS; by = o -> o.margin)[1:20],
                    key = sortby == "revenue" ? (o -> -o.revenue) :
                          sortby == "date"    ? (o -> o.date) : (o -> o.margin)
    slate_table(sort(thin; by = key);
                format = (price = :currency, revenue = :currency,
                          margin = (kind = :percent, digits = 1), units = :integer),
                align  = (order = :left, product = :left))
end)

#%% md id=s_charts
@md"""
---
## The same filter, drawn four ways

Each figure reads a control, so moving one recomputes in Julia and ECharts animates the change in
place instead of redrawing. The first three follow **region**. The fourth follows the **order-size
band**, the two-thumb `RangeSlider` above.

Those controls survive a standalone export because the values they drive are marked with `@replay`.
Live, the macro is a pass-through: only the current value is computed, so the cell costs what the
bare call costs. At export, Slate evaluates the marked expression once per position the control can
take, packs the results into one binary array and ships it, and moving the control then indexes that
shipped data. You never restate the domain and you never name a series. The domain is read off the
control, and the value is found wherever you put it in the option.

The band is the interesting one for cost. Its two thumbs are **one** value, so its domain is the
ordered pairs `lo ≤ hi`: 4 753 positions from 97 stops, not 9 409. At six numbers a slice, that comes
to a couple of hundred kilobytes.
"""

#%% code id=fig_month controls=region
# Only the marked value replays, so nothing else in the option may depend on `region`. An
# interpolated `$region` in the title would freeze at export while the series kept moving.
#
# No `xAxis` kwarg: a top-level axis replaces the one the DSL inferred, so naming the axis would take
# the category labels with it and silently leave a value axis plotting revenue against itself.
#
# Bars for the months, because a month's revenue is a discrete total. There is no value between
# February and March, and a line drawn through them invites the eye to read one. The trend is a
# separate named series, so the smoothing belongs to a statistic instead of being applied to the
# data. Two marks on one control in one cell, the same shape the at-risk split uses.
echart(series(:bar, MONTHS, @replay(region, revenue_by_month(region));
              name = "monthly revenue", itemStyle = (borderRadius = [3, 3, 0, 0],)),
       series(:line, MONTHS, @replay(region, moving_average(revenue_by_month(region)));
              name = "3-month average", smooth = 0.2, symbol = "none",
              lineStyle = (width = 2.5,), itemStyle = (color = "#f0883e",));
       title = "Revenue by month",
       height = 300,
       legend = true,
       valuefmt = :currency,
       yAxis = (name = "revenue",),
       zoom = :inside)

#%% code id=fig_category controls=region
echart(:bar, CATEGORIES, @replay(region, revenue_by_category(region));
       title = "Revenue by category",
       height = 300,
       valuefmt = :currency,
       yAxis = (name = "revenue",),
       itemStyle = (borderRadius = [3, 3, 0, 0],))

#%% code id=fig_margin controls=region
# The one worth switching regions on: LATAM's whole distribution sits several points left of the
# others, which is the pattern the generator planted and the summary table below confirms.
echart(:bar, ["$(round(Int, 100lo))%" for lo in MARGIN_BINS],
       @replay(region, margin_histogram(region));
       title = "Margin distribution",
       height = 300,
       yAxis = (name = "orders",),
       itemStyle = (borderRadius = [3, 3, 0, 0],))

#%% code id=fig_band controls=band
# The two-thumb control driving a figure. `band` arrives wrapped, as the `(lo, hi)` NamedTuple a cell
# sees, so the expression reads the same way it would live.
echart(:bar, CATEGORIES, @replay(band, revenue_in_band(band.lo, band.hi));
       title = "Revenue by category, within the order-size band",
       height = 300,
       valuefmt = :currency,
       yAxis = (name = "revenue",),
       itemStyle = (color = "#a371f7", borderRadius = [3, 3, 0, 0]))

#%% md id=s_calendar
@md"""
---
## Every day of the book, at once

A calendar heatmap is the cheapest way to look at 546 days without aggregating them away. Weekly
rhythm, quiet spells and clusters all read straight off the grid. `echart(:calendar, …)` takes dates
and values and brings the calendar component and its colour scale with it.

Five regions × 546 days is 2 730 values, about 22 KB packed. The cost model for a replayed **figure**
is `points × positions × 8` bytes, which is cheap here and would not be for a million-row table. A
replayed **table** is priced differently and more cheaply: its rows travel once, and each position
adds two bytes a row on top of them.
"""

#%% code id=fig_calendar controls=region
# One calendar spanning the whole book: `:calendar` binds a single series, so splitting the range
# into a block per year leaves every block after the first empty.
#
# Alpha-only greys, because the calendar component isn't covered by the chart theme. A grey at 35%
# reads correctly against a light or a dark page without asking which one this is. A continuous
# colour ramp would waste itself on counts that only run 0–5, so the scale is piecewise and
# fixed: it must not rescale per region, or the same colour would mean a different number.
echart(:calendar, string.(DAYS), @replay(region, orders_per_day(region));
       title = "Orders per day",
       height = 215,
       calendar = (range = ["2024-07", "2025-12"],
                   cellSize = ["auto", 15],
                   top = 52, left = 45, right = 25,
                   itemStyle = (color = "transparent",
                                borderColor = "rgba(128,128,128,0.35)", borderWidth = 0.5),
                   splitLine = (lineStyle = (color = "rgba(128,128,128,0.55)", width = 1),),
                   dayLabel = (color = "rgba(128,128,128,0.9)", firstDay = 1),
                   monthLabel = (color = "rgba(128,128,128,0.9)",),
                   yearLabel = (show = false,)),
       visualMap = (type = "piecewise", orient = "horizontal", left = "center", bottom = 4,
                    itemWidth = 13, itemHeight = 13, itemGap = 6,
                    textStyle = (color = "rgba(128,128,128,0.9)",),
                    pieces = [(value = 0, label = "quiet", color = "rgba(128,128,128,0.10)"),
                              (value = 1, label = "1", color = "#2f5d9e"),
                              (value = 2, label = "2", color = "#3b7dd8"),
                              (value = 3, label = "3", color = "#58a6ff"),
                              (gte = 4, label = "4+", color = "#56d364")]))

#%% md id=s_shape
@md"""
---
## Two views the filter can't reach

These two figures show *all* the data at once, so following the region control would only take
information away from them.

They also mark the edge of what `@replay` covers. A sunburst is a **tree**, and a geo scatter is a
list of `(lon, lat, …)` tuples. Neither is a numeric array of fixed shape, so there is nothing to
sweep and pack. Both are computed once in Julia and shipped as they are, which costs nothing, since
neither ever needed to change.

Click a ring to descend into it, and the centre to come back up. Drag the map, and scroll to zoom.
"""

#%% code id=fig_sunburst
# No `const` on a notebook global: cells re-run, and Julia refuses to re-declare an existing global
# as constant, so a `const` here works once and then fails on every subsequent run.
REVENUE_TREE = [
    reg => [cat => [p => round(total(filter(o -> o.region == reg && o.category == cat &&
                                                 o.product == p, ORDERS)))
                    for p in prods
                    if any(o -> o.region == reg && o.category == cat && o.product == p, ORDERS)]
            for (cat, prods, _) in CATALOG]
    for reg in REGIONS]

# ECharts defaults a sunburst's slice border to literal `white` at the series level, which is a
# light-page assumption and outranks anything set per level. On a dark page the rim turns into a web
# of white hairlines. Overriding it here works because a top-level kwarg splices into the series;
# `levels[…].itemStyle.borderColor` does not.
#
# ECharts also brightens each ring towards white as it goes out. Instead of fighting that per level,
# the whole series drops to 62% opacity, which against a dark page pulls the palette back to
# something you can look at and keeps the inner-to-outer ordering intact.
#
# `levels[1]` is the hidden root, so the three visible rings are entries 2–4, and `minAngle` drops
# the label on any slice too thin to hold one instead of letting them collide around the rim.
echart(:sunburst, REVENUE_TREE;
       title = "Region → category → product",
       height = 560,
       valuefmt = :currency,
       radius = [0, "92%"],
       itemStyle = (borderColor = "rgba(13,17,23,0.9)", borderWidth = 1.5, opacity = 0.62),
       label = (color = "#e6edf3", fontWeight = 500),
       levels = [(;),
                 (r0 = "8%", r = "38%", label = (rotate = "tangential", fontSize = 12,
                                                 minAngle = 12)),
                 (r0 = "39%", r = "64%", label = (rotate = "radial", fontSize = 10,
                                                  minAngle = 10)),
                 (r0 = "65%", r = "92%", label = (rotate = "radial", fontSize = 9,
                                                  minAngle = 6))])

#%% code id=fig_geo
# Slate vendors a world GeoJSON, so a real coastline costs one `registerMap` and no network.
#
# This is the raw `echart(; series = […])` form instead of `series(:scatter, …)`. A geo scatter
# carries its coordinates inside `data` and has no x/y vectors to hand the express form, and
# `series(:scatter; data = …)` is rejected: the keyword-data path is only open to kinds whose arity
# the DSL doesn't already know.
#
# `silent = true` on the geo would kill `roam` by swallowing the mouse; `emphasis = (disabled = true,)`
# turns off hover highlighting without also turning off dragging. Drag the map, scroll to zoom.
REGION_COORDS = Dict("Americas" => (-98.0, 39.0), "EMEA" => (12.0, 48.0),
                     "APAC" => (114.0, 20.0), "LATAM" => (-58.0, -16.0))

geo_points = let peak = maximum(total(filter(o -> o.region == r, ORDERS)) for r in REGIONS)
    [begin
         rs = filter(o -> o.region == reg, ORDERS)
         lon, lat = REGION_COORDS[reg]
         (name = reg,
          value = [lon, lat, round(total(rs)), round(100median(o.margin for o in rs); digits = 1)],
          symbolSize = 20 + 44 * sqrt(total(rs) / peak))
     end for reg in REGIONS]
end

echart(; title = "Revenue by region, coloured by median margin",
       height = 460,
       tooltip = (trigger = "item",),
       registerMap = (name = "world", url = "/assets/maps/world.json"),
       geo = (map = "world", roam = true, top = 55, zoom = 1.15,
              itemStyle = (areaColor = "rgba(128,128,128,0.14)",
                           borderColor = "rgba(128,128,128,0.35)", borderWidth = 0.5),
              emphasis = (disabled = true,)),
       visualMap = (min = 20, max = 36, dimension = 3, calculable = true,
                    left = 10, bottom = 20, text = ["36%", "20%"],
                    textStyle = (color = "rgba(128,128,128,0.9)",),
                    inRange = (color = ["#da5b52", "#d29922", "#3fb950"],)),
       series = [(type = "scatter", coordinateSystem = "geo", data = geo_points,
                  itemStyle = (opacity = 0.88, borderColor = "rgba(13,17,23,0.8)",
                               borderWidth = 1),
                  label = (show = true, formatter = "{b}", position = "top",
                           color = "#e6edf3", fontSize = 12, fontWeight = 600))])

#%% md id=s_analysis
@md"""
---
## An analysis, not just a view

Everything so far re-slices data that already exists. This section asks a question of it instead.

Set a **margin floor**, the thinnest deal the business is willing to write, and the figure splits
every category's revenue into what clears it and what doesn't. The number that matters is the third
one: how much gross profit the book gives up by writing business below the floor. That is
`Σ revenueᵢ × max(0, floor − marginᵢ)`, and it is not a column you can sort.

A `Slider` suits this better than a dropdown, for the reason sliders usually do. The interesting part
is not any single floor, but watching where the split *moves* as you drag.
"""

#%% code id=analysis_fns
below(floor_pct, rs) = filter(o -> 100o.margin < floor_pct, rs)

revenue_below(floor_pct) = [total(below(floor_pct, filter(o -> o.category == c, ORDERS)))
                            for c in CATEGORIES]
revenue_above(floor_pct) = [total(filter(o -> o.category == c && 100o.margin ≥ floor_pct, ORDERS))
                            for c in CATEGORIES]

# Gross profit forgone: what each under-floor order would have cleared at the floor, minus what it
# actually cleared. Orders already above the floor contribute nothing, hence the `max(0, ·)`.
forgone(floor_pct) = sum(o -> o.revenue * max(0.0, floor_pct / 100 - o.margin), ORDERS; init = 0.0)

# Driven by the TableSelect. `sel` is the row, a NamedTuple, the same value the live cell sees, or
# `nothing` when no row is picked. That is a position in the domain like any other, so it has to
# return the same shape: zeros and NaNs, not an empty vector.
function product_monthly(sel)
    sel === nothing && return zeros(length(MONTHS))
    peers = filter(o -> o.product == sel.product, ORDERS)
    [total(filter(o -> month_of(o.date) == m, peers)) for m in MONTHS]
end

function product_margin(sel)
    sel === nothing && return fill(NaN, length(MONTHS))
    peers = filter(o -> o.product == sel.product, ORDERS)
    [let ms = [o.margin for o in peers if month_of(o.date) == m]
         isempty(ms) ? NaN : 100 * median(ms)
     end for m in MONTHS]
end

"floor 24% → $(length(below(24, ORDERS))) orders below · $(usd(forgone(24))) forgone"

#%% code id=floor_ctl
@bind floor_pct Slider(0:2:60; default = 24, label = "Margin floor (%)")

#%% code id=fig_atrisk controls=floor_pct
# Two marks on the same control in one cell. Each series is swept independently over the slider's 31
# positions and packed on its own, so a stacked chart replays as readily as a single line.
echart(series(:bar, CATEGORIES, @replay(floor_pct, revenue_above(floor_pct));
              name = "clears the floor", stack = "revenue",
              itemStyle = (color = "#3fb950", borderRadius = [0, 0, 0, 0])),
       series(:bar, CATEGORIES, @replay(floor_pct, revenue_below(floor_pct));
              name = "below the floor", stack = "revenue",
              itemStyle = (color = "#da5b52", borderRadius = [3, 3, 0, 0]));
       title = "Revenue split by margin floor",
       height = 340,
       legend = true,
       valuefmt = :currency,
       yAxis = (name = "revenue",))

#%% md id=analysis_kpi
@md"""
At a **{{ floor_pct }}%** floor, **{{ length(below(floor_pct, ORDERS)) }}** of 640 orders fall short.
They carry **{{ usd(total(below(floor_pct, ORDERS))) }}** of revenue and give up
**{{ usd(forgone(floor_pct)) }}** of gross profit against the floor, about
**{{ round(100 * forgone(floor_pct) / profit(ORDERS); digits = 1) }}%** of everything the book
actually cleared.

Drag it past 35% and the red overtakes the green everywhere. The median order clears
{{ round(100 * median(o.margin for o in ORDERS); digits = 1) }}%, so any floor above that condemns
most of the business by construction. That is easier to see by dragging than by picking one number.
"""

#%% md id=s_drill
@md"""
---
## Clicking a row

`TableSelect` is a table that *is* a control. It renders like any other, and the row a reader clicks
becomes the bound value: a `NamedTuple` with a field per column, so downstream cells read
`sel.product` and `sel.margin` directly. There is no id column to thread and no lookup to write.

Pick one of the twenty thinnest deals and the figure below rebuilds around that product, showing
every order of it in every region. Revenue by month appears as bars. The month-to-month median margin
appears as **points**, with a three-month average drawn through them, because for a single product a
month is only a handful of orders and joining those medians into one line would draw structure the
sample size cannot support.
"""

#%% code id=drill_ctl
THINNEST = sort(ORDERS; by = o -> o.margin)[1:20]

@bind sel TableSelect(THINNEST; label = "Order")

#%% code id=drill_fig
# Every series is `@replay`ed over the same control, so the whole drill-down survives an export. Each
# sweep walks the twenty rows plus "nothing selected", and clicking a row on a static page indexes
# shipped data. `sel` arrives as the row NamedTuple here, not the wire row index.
#
# Nothing outside the marked expressions may depend on `sel`. An interpolated product name in the
# title would freeze at export while the bars kept moving.
#
# The margin is drawn as points plus a trend, not as one connected line. For a single product a month
# is a handful of orders, so the month-to-month median is noisy and a line through it reads as
# structure that isn't there. Where a month has no orders at all, there is nothing to join to.
#
# `valuefmt` per series, because the tooltip vocabulary differs by axis: revenue is money, margin is
# a percentage, and a top-level format would have called both of them dollars.
echart(series(:bar, MONTHS, @replay(sel, product_monthly(sel));
              name = "revenue", valuefmt = :currency,
              itemStyle = (borderRadius = [3, 3, 0, 0],)),
       series(:scatter, MONTHS, @replay(sel, product_margin(sel));
              name = "monthly median margin", yAxisIndex = 1, symbolSize = 7,
              valuefmt = (kind = :fixed, digits = 1),
              itemStyle = (color = "#f0883e", opacity = 0.8)),
       series(:line, MONTHS, @replay(sel, moving_average(product_margin(sel)));
              name = "3-month average", yAxisIndex = 1, smooth = 0.2, symbol = "none",
              connectNulls = true, valuefmt = (kind = :fixed, digits = 1),
              lineStyle = (width = 2.5, color = "#f0883e"));
       title = "The picked order's product — every order of it, all regions",
       height = 340,
       legend = true,
       yAxis = [(name = "revenue",), (name = "margin %", min = 0, max = 65)])

#%% md id=s_pivot
@md"""
---
## A static table is still worth having

A table does not have to move to earn its place. The summary below is computed once in Julia and
rendered by `slate_table`. It answers in one glance the question the interactive view makes you hunt
for, and it survives into a PDF or markdown export as a real table instead of a picture of one.
LATAM leads on revenue and lands last on gross profit; the margin column is where that happens.
"""

#%% code id=table_pivot
summary_rows = map(REGIONS) do reg
    rs = filter(o -> o.region == reg, ORDERS)
    (region = reg, orders = length(rs), revenue = total(rs),
     median_order = median(o.revenue for o in rs),
     median_margin = median(o.margin for o in rs),
     gross_profit = profit(rs),
     share = total(rs) / total(ORDERS))
end

slate_table(sort(summary_rows; by = r -> -r.revenue);
            format = (revenue = :currency, median_order = :currency, gross_profit = :currency,
                      median_margin = (kind = :percent, digits = 1),
                      share = (kind = :percent, digits = 1), orders = :integer),
            align  = (region = :left,),
            viz    = (revenue = :bar, median_margin = :heat))

# ╔═╡ Slate.config · per-notebook settings (Settings panel)
#   docid = 82144881-9a16-4f32-9c85-88c33789d0c3
# ╚═╡
