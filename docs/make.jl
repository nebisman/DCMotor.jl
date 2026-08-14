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
        "Instalación" => [
            "Instalación del firmware" => "instalacion/firmware.md",
            "Instalación del software" => [
                "Instalación del paquete" => "instalacion/paquete.md",
                "Permisos del puerto serial en linux" => "instalacion/permisos_serial.md",
            ],
        ],
        "Funciones" => [
            "Información" => "funciones/informacion.md",
            "Identificación" => "funciones/identificacion.md",
            "Control" => "funciones/control.md",
        ],
    ],
)

deploydocs(;
    repo="github.com/nebisman/DCMotor.jl",
    devbranch="main",
)
