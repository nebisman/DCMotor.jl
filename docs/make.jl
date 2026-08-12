using Documenter
using DCMotor

DocMeta.setdocmeta!(DCMotor, :DocTestSetup, :(using DCMotor); recursive=true)

makedocs(;
    modules=[DCMotor],
    authors="LB, HD",
    sitename="DCMotor.jl",
    checkdocs=:none,
    format=Documenter.HTML(;
        canonical="https://nebisman.github.io/DCMotor.jl",
        edit_link="main",
        assets=String[],
    ),
    pages=[
        "Inicio" => "index.md",
        "Referencia de la API" => "api.md",
    ],
)

deploydocs(;
    repo="github.com/nebisman/DCMotor.jl",
    devbranch="main",
)
