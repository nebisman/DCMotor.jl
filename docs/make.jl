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
            "Utilidades" => [
                "`MotorSystem`" => "funciones/informacion/MotorSystem.md",
                "`find_port`" => "funciones/informacion/find_port.md",
                "`speed_from_volts`" => "funciones/informacion/speed_from_volts.md",
                "`volts_from_speed`" => "funciones/informacion/volts_from_speed.md",
                "`transfer_function`" => "funciones/informacion/transfer_function.md",
            ],
            "Identificación" => [
                "`get_static_model`" => "funciones/identificacion/get_static_model.md",
                "`step_open`" => "funciones/identificacion/step_open.md",
                "`prbs_open`" => "funciones/identificacion/prbs_open.md",
                "`get_fomodel_step`" => "funciones/identificacion/get_fomodel_step.md",
                "`get_model_prbs`" => "funciones/identificacion/get_model_prbs.md",
            ],
            "Control" => [
                "`set_reference`" => "funciones/control/set_reference.md",
                "`set_pid`" => "funciones/control/set_pid.md",
                "`set_controller`" => "funciones/control/set_controller.md",
                "`step_closed`" => "funciones/control/step_closed.md",
                "`stairs_closed`" => "funciones/control/stairs_closed.md",
                "`profile_closed`" => "funciones/control/profile_closed.md",
                "`stepinfo_exp`" => "funciones/control/stepinfo_exp.md",
            ],
        ],
    ],
)

deploydocs(;
    repo="github.com/nebisman/DCMotor.jl",
    devbranch="main",
)
