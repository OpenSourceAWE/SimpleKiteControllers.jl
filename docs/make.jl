using Pkg
if ! ("Documenter" ∈ keys(Pkg.project().dependencies))
    Pkg.activate(@__DIR__)
end
using SimpleKiteControllers
using Documenter

DocMeta.setdocmeta!(SimpleKiteControllers, :DocTestSetup, :(using SimpleKiteControllers); recursive=true)

makedocs(;
    modules=[SimpleKiteControllers],
    authors="Uwe Fechner <u.fechner-1@tudelft.nl> and contributors",
    repo="https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/{commit}{path}#{line}",
    sitename="SimpleKiteControllers.jl",
    checkdocs=:exports,
    format=Documenter.HTML(;
        repolink = "https://github.com/OpenSourceAWE/SimpleKiteControllers.jl",
        prettyurls=get(ENV, "CI", "false") == "true",
        canonical="https://OpenSourceAWE.github.io/SimpleKiteControllers.jl",
        assets=["assets/custom.css"],
        size_threshold_warn = 400 * 1024,
        size_threshold = 1024 * 1024,
    ),
    pages=[
        "Home" => "index.md",
        "Settings" => "settings.md",
        "Examples - general" => "examples_general.md",
        "Examples - identification" => "examples_identification.md",
        "Examples - figure-of-eight" => "examples_fig8.md",
        "Examples - reel-out" => "examples_reelout.md",
        "API" => [
            "Overview" => "api/index.md",
            "Flight control" => "api/flight_control.md",
            "Flight-path geometry" => "api/path_geometry.md",
            "Winch" => "api/winch.md",
            "Settings and data files" => "api/settings.md",
            "Reel-out runs" => "api/reelout_run.md",
            "Run evaluation" => "api/evaluation.md",
            "Course-loop stability" => "api/stability.md",
            "Shape optimization" => "api/optimization.md",
            "Internals" => "api/internals.md",
        ],
    ],
)

deploydocs(;
    repo="github.com/OpenSourceAWE/SimpleKiteControllers.jl",
    devbranch="main",
    push_preview=true,
)
