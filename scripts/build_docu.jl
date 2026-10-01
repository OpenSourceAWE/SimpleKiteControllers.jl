# build and display the html documentation locally

using Pkg

root = dirname(@__DIR__)
if !("Documenter" ∈ keys(Pkg.project().dependencies))
    Pkg.activate(joinpath(root, "docs"))
end
# docs/ is a workspace project and shares the root manifest with examples/ and
# test/. If Documenter is not installed there, resolving it would re-resolve the
# examples' packages too, so build in a temporary environment instead.
if isnothing(Base.find_package("Documenter"))
    Pkg.activate(; temp = true)
    Pkg.develop(path = root)
    Pkg.add(["Documenter", "LiveServer"])
end
# LiveServer is not a docs dependency; install it in the global environment,
# which stays visible through the load path, if it cannot be found yet.
if isnothing(Base.find_package("LiveServer"))
    docs_project = Pkg.project().path
    Pkg.activate()
    Pkg.add("LiveServer")
    Pkg.activate(docs_project)
end
using LiveServer; servedocs(launch_browser=true)
