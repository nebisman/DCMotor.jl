# ═══════════════════════════════════════════════════════════════════════════════
#  motorsys.jl – Comunicación serial con el sistema UNDCMotor (ESP32).
#  LB 2026 – MIT License
# ═══════════════════════════════════════════════════════════════════════════════

using LibSerialPort
using JSON3
using Interpolations
using DelimitedFiles
using Printf
using Statistics

# ── Constantes del protocolo ─────────────────────────────────────────────────
const SAMPLING_TIME = 0.02
const BUFFER_SIZE   = 20
const PRBS_LENGTH   = 1023
const FONT_SIZE     = 12

# ── Rutas por defecto ────────────────────────────────────────────────────────
# PATH_DATA apunta a la carpeta desde la cual el usuario invoca `using DCMotor`
# (el directorio de trabajo en el momento de cargar el paquete).
const PATH_DATA = Ref{String}("")

"""Ruta absoluta de un archivo dentro de PATH_DATA/datafiles."""
_datafile(name::AbstractString) = joinpath(PATH_DATA[], "datafiles", name)

"""
Crea (si no existen) los directorios "datafiles" y "ejemplos" dentro de
PATH_DATA, y los puebla con los archivos del paquete instalado. Si ya
existen, no hace nada.
"""
function _ensure_dirs()
    pkg_root = joinpath(@__DIR__, "..")

    datafiles_dst = joinpath(PATH_DATA[], "datafiles")
    if !isdir(datafiles_dst)
        mkpath(datafiles_dst)
        datafiles_src = joinpath(pkg_root, "datafiles")
        for f in filter(f -> endswith(f, ".csv"), readdir(datafiles_src))
            cp(joinpath(datafiles_src, f), joinpath(datafiles_dst, f); force=true)
        end
    end

    ejemplos_dst = joinpath(PATH_DATA[], "ejemplos")
    if !isdir(ejemplos_dst)
        ejemplos_src = joinpath(pkg_root, "ejemplos")
        cp(ejemplos_src, ejemplos_dst; force=true)
    end
    return nothing
end


# ═══════════════════════════════════════════════════════════════════════════════
#  Funciones de conversión hex  (idénticas al protocolo del firmware)
#  El firmware usa big-endian IEEE-754 float32 y big-endian uint32.
#  Nota: reinterpret(UInt32, Float32(x)) devuelve el bit-pattern IEEE-754,
#  que coincide con la representación big-endian usada por Python struct.pack('>f').
# ═══════════════════════════════════════════════════════════════════════════════

"""Convierte Float32 a string hexadecimal de 8 caracteres (bit-pattern IEEE-754)."""
function float2hex(value::Real)::String
    u = reinterpret(UInt32, Float32(value))
    return string(u; base=16, pad=8)
end

"""Convierte string hexadecimal de 8 caracteres a Float64 (bit-pattern IEEE-754)."""
function hex2float(h::AbstractString)::Float64
    u = parse(UInt32, h; base=16)
    return Float64(reinterpret(Float32, u))
end

"""Convierte UInt32 a string hexadecimal de 8 caracteres."""
function long2hex(value::Integer)::String
    return string(UInt32(value); base=16, pad=8)
end

"""Convierte string hexadecimal de 8 caracteres a Int."""
function hex2long(h::AbstractString)::Int
    return Int(parse(UInt32, h; base=16))
end

"""Convierte un vector de floats a un string hex concatenado."""
function signal2hex(signal)::String
    return join(float2hex(v) for v in signal)
end

"""Convierte un vector de enteros (puntos de tiempo) a string hex concatenado."""
function time2hex(time_points)::String
    return join(long2hex(Int(t)) for t in time_points)
end

"""Convierte una matriz (2D array) a string hex fila por fila."""
function matrix2hex(mat)::String
    if ndims(mat) == 1
        return join(float2hex(e) for e in mat)
    else
        return join(float2hex(mat[i, j]) for i in axes(mat, 1) for j in axes(mat, 2))
    end
end

"""Convierte un string hex separado por comas a un vector de Float64."""
function hexframe_to_array(hexframe::AbstractString)::Vector{Float64}
    return [hex2float(strip(p)) for p in split(hexframe, ",")]
end


# ═══════════════════════════════════════════════════════════════════════════════
#  Lectura de archivos CSV
# ═══════════════════════════════════════════════════════════════════════════════

"""Lee archivo CSV de 2 columnas (u, y). Retorna (u, y)."""
function read_csv_file(filepath::String = _datafile("DCmotor_static_gain_response.csv"))
    data = readdlm(filepath, ','; header=true)[1]
    return Float64.(data[:, 1]), Float64.(data[:, 2])
end

"""Lee archivo CSV de 3 columnas (t, u, y). Retorna (t, u, y)."""
function read_csv_file3(filepath::String)
    data = readdlm(filepath, ','; header=true)[1]
    return Float64.(data[:, 1]), Float64.(data[:, 2]), Float64.(data[:, 3])
end

function read_model(filepath::String)
    data = readdlm(filepath, ','; header=true)[1]
    return data[:, 1], data[:, 2], data[:, 3]
end


# ═══════════════════════════════════════════════════════════════════════════════
#  Estructura principal:  MotorSystem
# ═══════════════════════════════════════════════════════════════════════════════

"""
    MotorSystem(; port="/dev/ttyUSB0", bauds=460800)

Objeto que representa la interfaz serial entre Julia y la plataforma
UNDCMotor (placa ESP32). Todas las funciones de comunicación, control e
identificación del paquete reciben una instancia de `MotorSystem` como primer
argumento.

El constructor solo crea la estructura; no abre el puerto serial. Use
[`connect!`](@ref) para abrir la conexión y [`disconnect!`](@ref) para
cerrarla (la mayoría de las funciones de alto nivel, como [`set_reference`](@ref)
o [`step_closed`](@ref), ya se conectan y desconectan automáticamente).

# Argumentos de palabra clave
- `port::String="/dev/ttyUSB0"`: nombre del puerto serial al que está
  conectada la placa ESP32 (por ejemplo `"/dev/ttyUSB0"` en Linux, `"COM3"`
  en Windows).
- `bauds::Int=460800`: velocidad de transmisión (baudios) del puerto serial;
  debe coincidir con la configurada en el firmware.

# Campos
- `port::String`: puerto serial configurado.
- `bauds::Int`: velocidad de transmisión.
- `sp::Union{SerialPort,Nothing}`: objeto de bajo nivel del puerto serial
  abierto (`nothing` si no está conectado).
- `topics::Dict{String,String}`: mapa entre comandos internos (p. ej.
  `"set_pid"`) y los tópicos del protocolo serial del firmware (p. ej.
  `"/set_pid"`).
- `queue::Channel{Dict{String,Any}}`: cola interna donde el hilo lector
  deposita las tramas JSON recibidas del firmware.
- `reader_task::Union{Task,Nothing}`: tarea asíncrona que lee continuamente
  el puerto serial.
- `stop_reader::Threads.Atomic{Bool}`: bandera atómica usada para detener la
  tarea lectora.

# Ejemplos
```julia-repl
julia> using DCMotor

julia> sys = MotorSystem(port="/dev/ttyUSB0", bauds=460800);

julia> connect!(sys)

julia> set_reference(sys, 90.0)

julia> disconnect!(sys)
```
"""
mutable struct MotorSystem
    port         :: String
    bauds        :: Int
    sp           :: Union{SerialPort, Nothing}
    topics       :: Dict{String, String}
    queue        :: Channel{Dict{String, Any}}
    reader_task  :: Union{Task, Nothing}
    stop_reader  :: Threads.Atomic{Bool}

    function MotorSystem(;                        
            port::String         = "/dev/ttyUSB0",
            bauds::Int           = 460800)

        topics = Dict(
            "set_pid"       => "/set_pid",
            "set_ref"       => "/set_ref",
            "step_closed"   => "/step_closed",
            "stairs_closed" => "/stairs_closed",
            "prbs_open"     => "/prbs_open",
            "step_open"     => "/step_open",
            "set_gencon"    => "/set_gencon",
            "prof_closed"   => "/prof_closed",
        )
        new(port, bauds, nothing, topics,
            Channel{Dict{String, Any}}(256), nothing, Threads.Atomic{Bool}(false))
    end
end

# ── Conexión / desconexión ───────────────────────────────────────────────────

"""Abre el puerto serial e inicia la tarea lectora."""
function connect!(sys::MotorSystem)
    if sys.sp === nothing
        sys.sp = LibSerialPort.open(sys.port, sys.bauds)
        # Limpiar buffer de entrada (ignorar si falla)
        try sp_flush(sys.sp, SP_BUF_INPUT) catch end
    end
    _start_reader!(sys)
    return nothing
end

"""Detiene la tarea lectora y cierra el puerto serial."""
function disconnect!(sys::MotorSystem)
    _stop_reader!(sys)
    if sys.sp !== nothing
        close(sys.sp)
        sys.sp = nothing
    end
    return nothing
end

# ── Hilo lector en segundo plano ─────────────────────────────────────────────

function _start_reader!(sys::MotorSystem)
    if sys.reader_task !== nothing && !istaskdone(sys.reader_task)
        return  # ya corriendo
    end
    sys.stop_reader[] = false
    # Vaciar la cola de datos viejos
    while isready(sys.queue)
        take!(sys.queue)
    end
    sys.reader_task = Threads.@spawn _reader_loop(sys)
    return nothing
end

function _stop_reader!(sys::MotorSystem)
    sys.stop_reader[] = true
    if sys.reader_task !== nothing
        # Esperar hasta 3 segundos
        t0 = time()
        while !istaskdone(sys.reader_task) && (time() - t0) < 3.0
            sleep(0.05)
        end
        sys.reader_task = nothing
    end
    return nothing
end

function _reader_loop(sys::MotorSystem)
    buf = IOBuffer()
    while !sys.stop_reader[]
        try
            # Leer bytes disponibles del serial
            if sys.sp === nothing
                break
            end
            nb = bytesavailable(sys.sp)
            if nb == 0
                sleep(0.001)
                continue
            end
            bytes = read(sys.sp, nb)
            write(buf, bytes)

            # Procesar líneas completas
            raw = String(take!(buf))
            lines = split(raw, '\n')
            # La última parte puede ser incompleta → devolverla al buffer
            if !endswith(raw, '\n')
                write(buf, lines[end])
                lines = lines[1:end-1]
            end

            for line in lines
                line = strip(line)
                isempty(line) && continue

                # Extraer la parte JSON
                json_str = if startswith(line, "D:")
                    line[3:end]
                elseif startswith(line, "{")
                    line
                else
                    continue  # línea de debug → ignorar
                end
               
                msg = try
                    JSON3.read(json_str, Dict{String, Any})
                catch
                    continue
                end

                # Solo tramas con campo "frame"
                if haskey(msg, "frame")
                    try
                        put!(sys.queue, msg)
                    catch
                        break
                    end
                end
            end
        catch e
            if e isa InterruptException
                break
            end
            # Otros errores → continuar
            sleep(0.01)
        end
    end
end

# ── Envío de comandos ────────────────────────────────────────────────────────

"""
    send_command!(sys, topic_key, payload)

Envía `topic\\npayload_json\\n` al firmware (protocolo serialCommandTask).
"""
function send_command!(sys::MotorSystem, topic_key::String, payload::Dict)
    topic = sys.topics[topic_key]
    raw = topic * "\n" * JSON3.write(payload) * "\n"
    write(sys.sp, raw)
    return nothing
end

# ── Recepción síncrona de tramas ─────────────────────────────────────────────

"""
    receive_frames!(sys, total_frames, on_frame; timeout_factor=20.0)

Bloquea hasta recibir `total_frames` tramas. Invoca `on_frame(msg, frame_no)`
por cada trama recibida. Lanza `TimeoutError` si no llegan datos.
"""
function receive_frames!(sys::MotorSystem, total_frames::Int, on_frame::Function;
                         timeout_factor::Float64 = 10.0)
    timeout = timeout_factor * BUFFER_SIZE * SAMPLING_TIME
    curr_frame = -1
    sync = false

    while curr_frame < total_frames 
        msg = try
            # Esperar con timeout usando timedwait
            result = nothing
            deadline = time() + timeout
            while time() < deadline
                if isready(sys.queue)
                    result = take!(sys.queue)
                    break
                end
                sleep(0.005)
            end
            if result === nothing
                error("Timeout")
            end
            result
        catch
            throw(ErrorException(
                "No se recibieron datos del ESP32. " *
                "Verifique la conexión y que el firmware envíe las tramas por Serial."))
        end

        curr_frame = hex2long(string(msg["frame"]))
        if curr_frame == 1
            sync = true
        end
        if sync
            on_frame(msg, curr_frame)
        end
    end
    return nothing
end

# ── Modelos de la planta ─────────────────────────────────────────────────────

"""
    transfer_function(sys::MotorSystem, output::Symbol=:angle)

Retorna la función de transferencia nominal del motor DC, estimada
previamente con [`get_fomodel_step`](@ref) o [`get_model_prbs`](@ref) y
almacenada en `datafiles/DCmotor_fo_model.csv`.

El modelo identificado es de primer orden, `b/(s+a)`, con ganancia `b` y polo
en `-a`. Según el valor de `output`, la función retorna:
- `:angle` → `b / (s*(s+a))` (se agrega un integrador: de velocidad a
  posición angular).
- `:speed` → `b / (s+a)` (modelo directo de la velocidad angular).

# Argumentos
- `sys::MotorSystem`: objeto de la plataforma (no necesita estar conectado;
  solo se usa para ubicar el archivo del modelo).
- `output::Symbol=:angle`: variable de salida del modelo, `:angle` (ángulo,
  en grados) o `:speed` (velocidad angular, en grados/s).

# Retorna
- `G::ControlSystems.TransferFunction`: función de transferencia continua
  del motor DC para la salida solicitada.

# Ejemplos
```julia-repl
julia> using DCMotor, ControlSystems

julia> sys = MotorSystem();

julia> G_speed = transfer_function(sys, :speed)

julia> G_angle = transfer_function(sys, :angle)   # incluye el integrador
```
"""
function transfer_function(sys::MotorSystem,
                           output::Symbol = :angle)
    
    b, a , L = read_model(_datafile("DCmotor_fo_model.csv"))
    b, a  = b[1], a[1] 

    if output == :angle   # angle   
            num = [b]
            den = [1.0000,  a, 0]
 
            

    else # :speed 
            num = [b]
            den = [1.0000, a]
       
    end
    return ControlSystems.tf(num, den)
end

"""
    speed_from_volts(sys::MotorSystem, volts::Real)

Calcula la velocidad angular estacionaria (en °/s) que alcanza el motor DC
al aplicarle un voltaje constante `volts`, usando el modelo estático lineal
`y = K*u + sign(u)*b` ajustado por [`get_static_model`](@ref) y almacenado en
`datafiles/DCmotor_static_pars.csv`.

Si `|volts|` está dentro de la zona muerta del motor (`|volts| <= zm`), la
velocidad estacionaria retornada es `0.0`.

# Argumentos
- `sys::MotorSystem`: objeto de la plataforma (usado solo para ubicar el
  archivo del modelo estático).
- `volts::Real`: voltaje aplicado, en voltios. Debe cumplir `|volts| <= 5.0`.

# Retorna
- `Float64`: velocidad angular estacionaria estimada, en grados/segundo.

# Lanza
- `ErrorException` si `|volts| > 5.0`.

# Ejemplos
```julia-repl
julia> using DCMotor

julia> sys = MotorSystem();

julia> speed_from_volts(sys, 3.0)
412.35
```
"""
function speed_from_volts(sys::MotorSystem, volts::Real)
    K, b, zm = read_csv_file3(_datafile("DCmotor_static_pars.csv"))
    K, b, zm = K[1], b[1], zm[1]
    if abs(volts) <= zm
        return 0.0
    end
    
    if zm < abs(volts) <= 5.0
        return K * volts + sign(volts)*b
    end
    error("volts debe estar en [0, 5]")  

end

"""
    volts_from_speed(sys::MotorSystem, speed::Real)

Calcula el voltaje constante necesario para que el motor DC alcance, en
estado estacionario, la velocidad angular `speed` (en °/s). Es la función
inversa de [`speed_from_volts`](@ref) y usa el mismo modelo estático lineal
almacenado en `datafiles/DCmotor_static_pars.csv` y
`datafiles/DCmotor_static_gain_response.csv`.

# Argumentos
- `sys::MotorSystem`: objeto de la plataforma (usado solo para ubicar los
  archivos del modelo estático).
- `speed::Real`: velocidad angular deseada, en grados/segundo. Debe estar
  entre `0` y la velocidad máxima medida experimentalmente (típicamente
  cercana a `800` °/s a 5V).

# Retorna
- `Float64`: voltaje, en voltios, necesario para alcanzar `speed` en estado
  estacionario.

# Lanza
- `ErrorException` si `|speed|` excede el rango medido experimentalmente.

# Ejemplos
```julia-repl
julia> using DCMotor

julia> sys = MotorSystem();

julia> volts_from_speed(sys, 400.0)
2.87
```
"""
function volts_from_speed(sys::MotorSystem, speed::Real)
    u, y = read_csv_file(_datafile("DCmotor_static_gain_response.csv"))
    K, b, zm = read_model(_datafile("DCmotor_static_pars.csv"))
    K, b, zm = K[1], b[1], zm[1]
    
    if abs(speed) <= 2 
        return sign(speed)*zm
    end

    if 2 < abs(speed) <= y[end]
        return (speed - sign(speed)*b)/K
    end
    error("speed debe estar entre [0 y $(round(y[end],digits=1))]")
end


# ── Guardar experimentos ─────────────────────────────────────────────────────

"""Guarda las columnas como CSV en PATH_DATA/datafiles."""
function save_experiment(columns::Vector, filename::String, header::String)
    mat = hcat(columns...)
    open(_datafile(filename), "w") do io
        println(io, header)
        for i in axes(mat, 1)
            println(io, join([@sprintf("%.8f", mat[i, j]) for j in axes(mat, 2)], ","))
        end
    end
    return nothing
end










