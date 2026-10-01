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
        assets=String[],
        size_threshold_warn = 400 * 1024,
        size_threshold = 1024 * 1024,
    ),
    pages=[
        "Home" => "index.md",
        "Examples - general" => "examples_general.md",
        "Examples - identification" => "examples_identification.md",
        "Examples - figure-of-eight" => "examples_fig8.md",
        "Examples - reel-out" => "examples_reelout.md",
        "API" => "api.md",
    ],
)

deploydocs(;
    repo="github.com/OpenSourceAWE/SimpleKiteControllers.jl",
    devbranch="main",
    push_preview=true,
)
