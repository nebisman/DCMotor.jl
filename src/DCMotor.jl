# ═══════════════════════════════════════════════════════════════════════════════
#  DCMotor.jl – Módulo principal para control e identificación de motor DC.
#
#  LB y HD 2026 – MIT License
# ═══════════════════════════════════════════════════════════════════════════════

module DCMotor

# ── Dependencias externas ────────────────────────────────────────────────────
using Reexport
using LibSerialPort
using JSON3
using DelimitedFiles
@reexport using ControlSystemsBase
using LinearAlgebra
using Plots
using Polynomials
using Statistics
using Printf
using RobustAndOptimalControl
using ControlSystemIdentification
using LaTeXStrings
using Base64
using HypertextLiteral: @htl

# ── Código fuente ────────────────────────────────────────────────────────────
include("motorsys.jl")
include("controlsys.jl")
include("identsys.jl")
include("control_design.jl")
include("graphics.jl")

# ── Inicialización ───────────────────────────────────────────────────────────
# Se ejecuta cada vez que se hace `using DCMotor`: fija PATH_DATA a la carpeta
# (actual, superior o dos niveles arriba) que ya tenga datafiles/ o ejemplos/,
# o al directorio actual si no hay ninguna, y crea allí las que falten.
function __init__()
    raiz = _buscar_raiz_datos(pwd())
    PATH_DATA[] = raiz === nothing ? pwd() : raiz
    _ensure_dirs()
    close = MotorSystem(port="")
    disconnect!(close)
end

# ── API pública ──────────────────────────────────────────────────────────────

# Constantes
export SAMPLING_TIME, BUFFER_SIZE, PRBS_LENGTH

# Tipo principal
export MotorSystem

# Conexión
export connect!, disconnect!, find_port

# Conversiones hex (utilidades)
export float2hex, hex2float, long2hex, hex2long
export signal2hex, time2hex, matrix2hex, hexframe_to_array

# Lectura de archivos
export read_csv_file, read_csv_file3, get_deadzone

#figuras
export redraw!, screen_pluto, entorno

# Comandos de comunicación
export send_command!, receive_frames!

# Modelos de la planta
export get_last_model, speed_from_volts, volts_from_speed

# Funciones de control (controlsys)
export set_reference, set_pid, set_controller, set_ss_controller
export step_closed, stairs_closed, profile_closed, stepinfo

# Funciones de identificación (identsys)
export step_open, prbs_open
export get_static_model, get_model_step, get_model_prbs

# Funciones de diseño de controladores (control_design)
export cont2dof, cont1dof, lqmodel

end # module DCMotor
