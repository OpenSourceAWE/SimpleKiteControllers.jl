# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# Writing the commented run summaries of the example scripts.

"""
    write_yaml_commented(io, indent, node; comment_col = 36, color = false)

Serialize a nested dictionary (an `OrderedDict` keeps the key order) as YAML,
recursing into `AbstractDict` values and appending a trailing `# comment` for
leaves given as `(value, comment)` pairs, aligned to `comment_col` where the line
is short enough. `YAML.write_file` has no concept of comments, hence this by hand.
`color = true` adds ANSI syntax highlighting (keys, string/number values,
comments); leave it off when `io` is a file, so no escape codes end up on disk.
"""
function write_yaml_commented(io, indent, node; comment_col = 36, color = false)
    pad = "  "^indent
    for (k, v) in node
        if v isa AbstractDict
            print(io, pad)
            color ? printstyled(io, k; color = :cyan, bold = true) : print(io, k)
            println(io, ":")
            write_yaml_commented(io, indent + 1, v; comment_col, color)
        else
            value, comment = v isa Tuple ? v : (v, "")
            val = value isa AbstractString ? "\"$value\"" : string(value)
            prefix = string(pad, k, ": ", val)
            print(io, pad)
            if color
                printstyled(io, k; color = :cyan)
                print(io, ": ")
                printstyled(io, val; color = value isa AbstractString ? :green : :yellow)
            else
                print(io, k, ": ", val)
            end
            if isempty(comment)
                println(io)
            else
                print(io, " "^max(1, comment_col - length(prefix)))
                color ? printstyled(io, "# ", comment; color = :light_black) :
                    print(io, "# ", comment)
                println(io)
            end
        end
    end
end

"""
    time_keyed(row, rows) -> Vector{Pair{String, Any}}

Summary entries keyed by time, `t_<time>_s` (`%05.1f`), where `row(e)` gives
`(time, value)` for each element `e` of `rows`; pass the result to an
`OrderedDict`. Two rows CAN share a time — a blocking solve is requested and
collected on the same step, so a seed skipped from the failure cache carries the
same `t` as the install that follows it — and a plain comprehension silently
keeps the last of them (which hid the cache's own skips the first time it ran),
so a repeated key gets a suffix `_2`, `_3`, ...
"""
function time_keyed(row, rows)
    seen = Dict{String, Int}()
    out = Pair{String, Any}[]
    for e in rows
        t, value = row(e)
        k = @sprintf("t_%05.1f_s", t)
        n = get(seen, k, 0) + 1
        seen[k] = n
        push!(out, (n == 1 ? k : "$(k)_$n") => value)
    end
    return out
end
