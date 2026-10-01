# build and display the html documentation locally

using Pkg

if !("Documenter" ∈ keys(Pkg.project().dependencies))
    Pkg.activate("docs")
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
