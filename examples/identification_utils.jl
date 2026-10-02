# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# Helpers of the identification scripts that write their results into the course-loop model
# file (`identify_kite_delay_scaling.jl`, `identify_pattern_law.jl`): the file keeps its
# comments, so the values are rewritten line by line instead of with YAML.write_file.

using Printf

"""
    wrap_comment(text; indent = "  # ", width = 92) -> Vector{String}

`text` as YAML comment lines of at most `width` characters.
"""
function wrap_comment(text; indent = "  # ", width = 92)
    lines, line = String[], indent
    for word in split(text)
        if length(line) + length(word) + 1 > width && line != indent
            push!(lines, rstrip(line))
            line = indent
        end
        line *= (line == indent ? "" : " ") * word
    end
    push!(lines, line)
    return lines
end

"""
    update_yaml_values!(file, values; comments = Dict())

Set each `key => value` of `values` (the value already formatted as a string) in the YAML
file `file`, keeping the column of the inline comment, and replace the comment lines
directly above each key of `comments` with its lines. Every other line is kept.
"""
function update_yaml_values!(file, values; comments = Dict{String, Vector{String}}())
    lines = readlines(file)
    function find_key(key)
        found = findfirst(line -> occursin(Regex("^\\s*$key:"), line), lines)
        isnothing(found) && error("$file has no key $key.")
        return found
    end
    for (key, value) in values
        row = find_key(key)
        m = match(r"^(\s*\w+:\s*)(\S+)(\s*)(#.*)?$", lines[row])
        pad = max(length(m[2]) + length(m[3]) - length(value), 1)
        lines[row] = m[1] * value * " "^pad * something(m[4], "")
    end
    for (key, block) in comments
        row = find_key(key)
        first_comment = row
        while first_comment > 1 && startswith(lstrip(lines[first_comment - 1]), "#")
            first_comment -= 1
        end
        lines = [lines[1:first_comment - 1]; block; lines[row:end]]
    end
    open(io -> foreach(line -> println(io, line), lines), file, "w")
    return nothing
end
