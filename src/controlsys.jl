# ═══════════════════════════════════════════════════════════════════════════════
#  controlsys.jl – Funciones de control para el sistema UNDCMotor via serial.
#  LB 2026 – MIT License
# ═══════════════════════════════════════════════════════════════════════════════

using ControlSystemsBase
using LinearAlgebra
using Plots
using Printf
using Statistics
using RobustAndOptimalControl
using LaTeXStrings





# ═══════════════════════════════════════════════════════════════════════════════
#  set_reference
# ═══════════════════════════════════════════════════════════════════════════════

"""
    set_reference(sys::MotorSystem, ref_value::Real=50.0)

Fija la referencia de velocidad o ángulo de la plataforma DCMMotor al valor `ref_value`.

# Argumentos
- `sys::MotorSystem`: objeto que representa la plataforma.
- `ref_value::Real=90.0`: valor de referencia a fijar. Sus unidades dependen
  del tipo de control activo en la plataforma (grados si `output=:angle`,
  grados/s si `output=:speed`; ver [`set_pid`](@ref) y [`set_controller`](@ref)).

# Retorna
- `nothing`. Como efecto secundario, imprime en consola la referencia fijada.

# Ejemplo
```julia
using DCMotor
sys = MotorSystem();
set_reference(sys, 180.0)

    "Referencia fijada a 180.00"
```
"""
function set_reference(sys::MotorSystem, ref_value::Real =90.0)
    connect!(sys)
    send_command!(sys, "set_ref", Dict("reference" => float2hex(ref_value)))
    disconnect!(sys)      
    println("Referencia fijada a $(@sprintf("%.2f", ref_value)) \n")
    return nothing
end



# ═══════════════════════════════════════════════════════════════════════════════
#  set_pid
# ═══════════════════════════════════════════════════════════════════════════════

"""
    set_pid(sys::MotorSystem; kp=0.2164, ki=1.8122, kd=0.004244, N=10.0, beta=0.0,
            output=:angle, deadzone=0.125)

Esta función configura los cinco parámetros `kp`, `ki`, `kd`, `N` y `beta` de
un controlador PID de 2-GDL (dos grados de libertad), como el mostrado en la siguiente figura:

![Diagrama de bloques del controlador PID de dos grados de libertad](../../assets/pid2gdl.png)

# Argumentos
- `sys::MotorSystem`: objeto que representa la plataforma DCMotor.

# Argumentos de palabra clave
- `kp::Real=0.2164`: ganancia proporcional del controlador PID.
- `ki::Real=1.8122`: ganancia integral del controlador PID.
- `kd::Real=0.004244`: ganancia derivativa del controlador PID.
- `N::Real=10.0`: coeficiente del filtro del derivador.
- `beta::Real=0.0`: ponderación del punto de ajuste (*setpoint weighting*)
  del controlador PID.
- `output::Symbol=:angle`: variable controlada, `:angle` (ángulo) o
  `:speed` (velocidad angular).
- `deadzone::Real=0.125`: zona muerta de la señal de control, en voltios,
  para evitar *chattering* del motor cerca de cero.

# Retorna
- `nothing`, una vez que los parámetros del PID se han actualizado
  correctamente. Imprime en consola los parámetros configurados.

# Notas
- Cuando `beta=1.0`, el PID resultante corresponde al PID estándar de un grado
  de libertad (1-DOF) descrito en la mayoría de los libros de texto.
- Esta implementación se basa en el algoritmo formulado en el libro [*Feedback
  Systems: An Introduction for Scientists and Engineers*](https://fbswiki.org/wiki/index.php/PID_Control),
  segunda edición, de Åström y Murray. El algoritmo usa un periodo de muestreo
  del controlador de 0.02 segundos, apropiado para la plataforma DCMotor.

# Ejemplo
Primero, asegúrese de haber importado el paquete DCMotor y de haber definido
el sistema, así:

```julia
using DCMotor
sys = MotorSystem();
```

Luego, el controlador PID se programa así:

```julia
set_pid(sys; kp=0.2, ki=1.0, kd=0.0082, N=10.0, beta=0.0, output=:angle, deadzone=0.15)

    "PID actualizado: kp=0.2  ki=1.0  kd=0.0082  N=10.0  β=0.0"
```
"""
function set_pid(sys::MotorSystem;
                  kp::Real = 0.2164, ki::Real = 1.8122, kd::Real = 0.004244,
                  N::Real = 10.0, beta::Real = 0.0,
                  output::Symbol = :angle, deadzone::Real = 0.125)
    type_map = Dict(:angle => 0, :speed => 1)
    haskey(type_map, output) || error("output debe ser :angle o :speed")

    payload = Dict(
        "kp"          => float2hex(kp),
        "ki"          => float2hex(ki),
        "kd"          => float2hex(kd),
        "N"           => float2hex(N),
        "beta"        => float2hex(beta),
        "typeControl" => long2hex(type_map[output]),
        "deadzone"    => float2hex(deadzone),
    )
    connect!(sys)
    send_command!(sys, "set_pid", payload)
    set_reference(sys, 0)
    println("PID actualizado: kp=$kp  ki=$ki  kd=$kd  N=$N  β=$beta")
    return nothing
end


# ═══════════════════════════════════════════════════════════════════════════════
#  set_controller  (controlador general en espacio de estados)
# ═══════════════════════════════════════════════════════════════════════════════

"""
    set_controller(sys::MotorSystem, controller; output=:angle, deadzone=0.2)

Programa un controlador en la plataforma DCMotor. El controlador se define como una función de transferencia en el dominio de
Laplace `s`, la cual es automáticamente discretizada.

Esta función permite programar un controlador estándar de 1 grado de
libertad (1-GDL), como el mostrado en la siguiente figura:

![Controlador de 1 grado de libertad](../../assets/controller_1gdl.png)

A su vez, esta misma función permite programar un controlador de 2
grados de libertad, como se muestra a continuación:

![Controlador de 2 grados de libertad](../../assets/controller_2gdl.png)

# Argumentos
- `sys::MotorSystem`:  objeto que representa la plataforma DCMotor.
- `controller`: función de transferencia continua (`ControlSystems.TransferFunction`)
  en el dominio de Laplace `s`.
  - Si ``C`` es 1x1 (una entrada y una salida), se interpreta como controlador de 1 grado de
    libertad:
    
    ``u = C·(r - y)``

  - Si  ``C=[C_1\\quad C_2]`` es una matriz 1×2 (dos entradas y una salida), se interpreta como controlador de 2 grados de
    libertad: 

    ``u = C_1·r - C_2·y``
    
    donde ``C_1=\\frac{L}{A}`` es la primera columna y
    ``C_2=\\frac{M}{A}`` es la segunda.

# Argumentos de palabra clave
- `output::Symbol=:angle`: variable controlada, `:angle` (ángulo) o
  `:speed` (velocidad angular).
- `deadzone::Real=0.2`: zona muerta de la señal de control, en voltios.

# Retorna
- `nothing`. Imprime un mensaje de confirmación en consola.

# Notas
- Esta función admite tanto controladores de 1-GDL como de 2-GDL.
- El controlador se convierte a espacio de estados para su implementación
  digital. Se discretiza usando el método bilineal (Tustin) con un periodo
  de muestreo de 0.02 segundos.
- Se implementa un mecanismo anti-windup basado en un observador, cuya
  ganancia se calcula mediante el filtro de Kalman.

# Ejemplos
## Ejemplo 1: controlador de  un grado de libertad (1-GDL)
Supongamos que queremos implementar un controlador para el ángulo del **DCMotor** definido por la siguiente función de transferencia:

``C(s)=\\frac{0.1071s + 2.2463}{s + 35.5428}``

Para ello usamos el siguiente código:

```julia
using DCMotor, ControlSystemsBase
sys = MotorSystem();
s = tf("s");
C = (0.1071s + 2.2463)/(s + 35.5428);    
set_controller(sys, C; output=:angle, deadzone=0.2)

    "Controlador cargado en DCMotor"
```

## Ejemplo 2: controlador de dos grados de libertad (2-GDL)
Supongamos que queremos implementar un controlador de 2-GDL para el ángulo del **DCMotor**, definido por la siguientematriz de transferencia:

``C(s)=\\begin{bmatrix} \\frac{0.0374s + 2.6207}{s + 69.5428} & -\\frac{0.0279s + 2.6207}{s + 69.5428} \\end{bmatrix}``

Para ello usamos el siguiente código:

```julia
using DCMotor, ControlSystemsBase
sys = MotorSystem();
s = tf("s");
C1 = (0.0374s + 2.6207)/(s + 69.5428)
C2 = -(0.0279s + 2.6207)/(s + 69.5428)
C = [C1 C2]
set_controller(sys, C; output=:angle, deadzone=0.2);

    "Controlador cargado en DCMotor"
```
"""
function set_controller(sys::MotorSystem, controller;
     output::Symbol = :angle, deadzone::Real = 0.2)
    # Determinar si es 1 DOF o 2 DOF
    ni = size(controller, 2)  # número de entradas

    code_map = Dict(
        (:angle, 1) => 2, (:speed, 1) => 3,
        (:angle, 2) => 4, (:speed, 2) => 5,
    )
    type_control = get(code_map, (output, ni), nothing)
    type_control === nothing && error("output=:angle|:speed, struct 1 ó 2 DOF")

    # # ── Construir sistema continuo con 2 entradas [r, y] ────────────

    
    sys_c = minreal(ss(controller))

    # ── Discretización bilineal (Tustin) ──────────────────────────────
    sys_int = c2d(sys_c, SAMPLING_TIME, :tustin)
    sys_d, S, E = modal_form(sys_int)
    Ad, Bd, Cd, Dd = ssdata(sys_d)
    order = size(Ad, 1)
    Q = 100000*diagm(ones(size(Ad,1)))
    
    #Lg = lqr(Discrete, Ad',Cd',Q, 100000)# xxx
    Lg = kalman(sys_d, Q,1) 

    if ni == 1
       Bd =Matrix{Float64}( hcat(Bd, -Bd))
       Dd =Matrix{Float64}( hcat(Dd, -Dd))
    end


    order = size(Ad, 1)
    Ac_fw = Ad - Lg * Cd
    Bc_fw = Bd - Lg * Dd

    payload = Dict(
        "order"       => long2hex(order),
        "A"           => matrix2hex(Ac_fw),
        "B"           => matrix2hex(Bc_fw),
        "C"           => matrix2hex(Cd),
        "D"           => matrix2hex(Dd),
        "L"           => matrix2hex(Lg),
        "typeControl" => long2hex(type_control),
        "deadzone"    => float2hex(deadzone),
    )
    connect!(sys)
    send_command!(sys, "set_gencon", payload)
    set_reference(sys, 0)
    println("Controlador cargado en Motor")
    return nothing
end


# ═══════════════════════════════════════════════════════════════════════════════
#  step_closed
# ═══════════════════════════════════════════════════════════════════════════════

"""
    step_closed(sys::MotorSystem; r0=0, r1=100, t0=0.0, t1=1.0)

Ejecuta un experimento de respuesta al escalón en lazo cerrado: la
referencia permanece en `r0` durante `t0` segundos y luego salta a `r1`
durante `t1` segundos, mientras el controlador previamente cargado (con
[`set_pid`](@ref) o [`set_controller`](@ref)) actúa sobre la planta. Los
datos se grafican en tiempo real a medida que llegan del ESP32 y, al
finalizar, se guardan en `datafiles/DCmotor_step_closed_exp.csv` y la
referencia se vuelve a fijar en `0`.

# Argumentos
- `sys::MotorSystem`: objeto de la plataforma, previamente configurado con
  un controlador.

# Argumentos de palabra clave
- `r0::Real=0`: valor inicial (bajo) de la referencia.
- `r1::Real=100`: valor final (alto) de la referencia, tras el escalón.
- `t0::Real=0.0`: duración en segundos con la referencia en `r0`.
- `t1::Real=1.0`: duración en segundos con la referencia en `r1`.

# Retorna
- `(t, r, y, u)`: tupla de vectores `Vector{Float64}` con el tiempo (s), la
  referencia, la salida medida (ángulo o velocidad, según el controlador
  cargado) y la señal de control (voltios), respectivamente.

# Ejemplos
```julia
using DCMotor
sys = MotorSystem();
set_pid(sys; kp=2.0, ki=0.5, output=:angle);
t, r, y, u = step_closed(sys; r0=0, r1=180, t0=0.5, t1=2.0);
```
"""
function step_closed(sys::MotorSystem;
                     r0::Real = 0, r1::Real = 100,
                     t0::Real = 0.0, t1::Real = 1.0)
    h  = SAMPLING_TIME
    bs = BUFFER_SIZE
    points_low  = round(Int, t0 / h)
    points_high = round(Int, t1 / h)
    total       = points_low + points_high
    frames      = ceil(Int, total / bs)

    payload = Dict(
        "low_val"     => float2hex(r0),
        "high_val"    => float2hex(r1),
        "points_low"  => long2hex(points_low),
        "points_high" => long2hex(points_high),
    )

    # Vectores acumuladores
    tv, rv, yv, uv = Float64[], Float64[], Float64[], Float64[]

    # Gráfica inicial
    delta = abs(r1 - r0)
    plt = plot(layout=(2, 1), size=(900, 500),
        title=["Step cerrado  r0=$(@sprintf("%.1f",r0)) → r1=$(@sprintf("%.1f",r1))" ""],
        ylabel=["Grados (o °/s)" "Voltios"],
        xlabel=["Tiempo (s)" "Tiempo (s)"],
        xlims=[(0, t0 + t1 - h) (0, t0 + t1 - h)],
        ylims=[(min(r0, r1) - 0.6delta, max(r0, r1) + 0.6delta) (-5, 5)],
        background_color_subplot=[:ivory :mintcream],
        legend=[:bottomright :bottomright],
        grid=true, gridalpha=0.25, margin=5Plots.mm)
    display(plt)

    # Comunicación
    connect!(sys) 
    send_command!(sys, "step_closed", payload)

    on_frame = function(msg, frame_no)
        rf = hexframe_to_array(string(msg["r"]))
        uf = hexframe_to_array(string(msg["u"]))
        yf = hexframe_to_array(string(msg["y"]))
        tf = h .* (collect(0:length(rf)-1) .+ (frame_no - 1) * bs)
        append!(rv, rf); append!(yv, yf); append!(uv, uf); append!(tv, tf)

        # Reconstruir gráfica desde cero para evitar trazas duplicadas
        plt = plot(layout=(2, 1), size=(900, 500),
             title=["Step cerrado  r0=$(@sprintf("%.1f",r0)) → r1=$(@sprintf("%.1f",r1))" ""],
             ylabel=["Grados (o °/s)" "Volts"],
             xlabel=["Tiempo (s)" "Tiempo (s)"],
             xlims=[(0, t0 + t1 - h) (0, t0 + t1 - h)],
             ylims=[(min(r0, r1) - 0.6delta, max(r0, r1) + 0.6delta) (-5, 5)],
             background_color_subplot=[:ivory :mintcream],
             legend=[:bottomright :bottomleft],
             grid=true, gridalpha=0.15, margin=5Plots.mm)

        
        plot!(plt, subplot=1, tv, rv, label="", color=:teal,
              linewidth=1.25, seriestype=:steppost)
        plot!(plt, subplot=1, tv, yv, label="", color=:deeppink,
              linewidth=1.0)
        plot!(plt, subplot=2, tv, uv, label="", color=:royalblue,
              linewidth=1.0)

        redraw!(plt)
  
    end

    try
        receive_frames!(sys, frames, on_frame; timeout_factor=10.0)
    catch e
        println("Error: ", e)    

    end
    set_reference(sys, 0)   

    save_experiment([tv, rv, yv, uv], "DCmotor_step_closed_exp.csv", "t,r,y,u")
    return tv, rv, yv, uv
end


# ═══════════════════════════════════════════════════════════════════════════════
#  stairs_closed
# ═══════════════════════════════════════════════════════════════════════════════

"""
    stairs_closed(sys::MotorSystem; stairs=(90, 180, 270), duration=1.5)

Ejecuta en lazo cerrado una señal de referencia tipo "escalera": la
referencia toma sucesivamente cada uno de los valores de `stairs`,
permaneciendo `duration` segundos en cada nivel, mientras el controlador
previamente cargado (con [`set_pid`](@ref) o [`set_controller`](@ref)) actúa
sobre la planta. Los datos se grafican en tiempo real y, al finalizar, se
guardan en `datafiles/DCmotor_stairs_closed_exp.csv` y la referencia se
vuelve a fijar en `0`.

# Argumentos
- `sys::MotorSystem`: objeto de la plataforma, previamente configurado con
  un controlador.

# Argumentos de palabra clave
- `stairs`: colección (`Tuple` o `Vector`) con los niveles sucesivos de
  referencia.
- `duration::Real=1.5`: duración en segundos de cada escalón de la escalera.

# Retorna
- `(t, r, y, u)`: tupla de vectores `Vector{Float64}` con el tiempo (s), la
  referencia, la salida medida y la señal de control (voltios).

# Ejemplos
```julia
using DCMotor
sys = MotorSystem();
set_pid(sys; kp=2.0, ki=0.5, output=:angle);
t, r, y, u = stairs_closed(sys; stairs=(90, 180, 270, 90), duration=2.0);
```
"""
function stairs_closed(sys::MotorSystem;
                       stairs = (90, 180, 270),
                       duration::Real = 1.5)
    h  = SAMPLING_TIME
    bs = BUFFER_SIZE
    stairs_v = collect(Float64, stairs)
    dur_pts  = ceil(Int, duration / h)
    total    = length(stairs_v) * dur_pts - 1
    frames   = ceil(Int, total / bs)
    mn, mx   = minimum(stairs_v), maximum(stairs_v)

    payload = Dict(
        "signal"        => signal2hex(stairs_v),
        "duration"      => long2hex(dur_pts),
        "points_stairs" => long2hex(length(stairs_v)),
        "min_val"       => float2hex(mn),
        "max_val"       => float2hex(mx),
    )
    
   
    tv, rv, yv, uv = Float64[], Float64[], Float64[], Float64[]
    span = mx - mn

    plt = plot(layout=(2, 1), size=(900, 500),
        title=["Escaleras  $(length(stairs_v)) niveles, duración $(@sprintf("%.1f",total*h)) s" ""],
        ylabel=["Grados (o °/s)" "Volts"],
        xlabel=["Tiempo (s)" "Tiempo (s)"],
        xlims=[(0, total * h) (0, total * h)],
        ylims=[(min(0, mn - 0.1*abs(span)), mx + 0.1*span) (-5.5, 5.5)],
        background_color_subplot=[:ivory :mintcream],
        legend=:topright, grid=true, gridalpha=0.15, margin=5Plots.mm)
    display(plt)
   
    connect!(sys)
    
    send_command!(sys, "stairs_closed", payload)

    on_frame = function(msg, frame_no)
        rf = hexframe_to_array(string(msg["r"]))
        uf = hexframe_to_array(string(msg["u"]))
        yf = hexframe_to_array(string(msg["y"]))
        tf = h .* (collect(0:length(rf)-1) .+ (frame_no - 1) * bs)
        append!(rv, rf); append!(yv, yf); append!(uv, uf); append!(tv, tf)

        plt = plot(layout=(2, 1), size=(900, 500),
            title=["Escaleras con $(length(stairs_v)) niveles, duración $(@sprintf("%.1f",total*h)) s" ""],
            ylabel=["Grados (o °/s)" "Volts"],
            xlabel=["Tiempo (s)" "Tiempo (s)"],
            xlims=[(0, total * h) (0, total * h)],
            ylims=[(min(0, mn - 0.1*abs(span)), mx + 0.1*span) (-5.5, 5.5)],
            background_color_subplot=[:ivory :mintcream],
            legend=:topright, grid=true, gridalpha=0.15, margin=5Plots.mm)
        plot!(plt, subplot=1, tv, rv, label="r(t)", color=:darkgreen,
              linewidth=1.1)
        plot!(plt, subplot=1, tv, yv, label="y(t)", color=:deeppink,
              linewidth=1.25)
        plot!(plt, subplot=2, tv, uv, label="u(t)", color=:royalblue)
        redraw!(plt)        
    end

    try
        receive_frames!(sys, frames, on_frame;  timeout_factor=20.0)
    catch e
        println("Error: ", e)
    end
    set_reference(sys, 0)
   
    save_experiment([tv, rv, yv, uv], "DCmotor_stairs_closed_exp.csv", "t,r,y,u")
    return tv, rv, yv, uv
end


# ═══════════════════════════════════════════════════════════════════════════════
#  profile_closed
# ═══════════════════════════════════════════════════════════════════════════════

"""
    profile_closed(sys::MotorSystem; timevalues=(0,1,2,3), refvalues=(0,360,360,0))

Ejecuta en lazo cerrado un perfil de referencia arbitrario, definido por
pares `(timevalues[i], refvalues[i])` que el firmware interpola linealmente
entre los puntos indicados, mientras el controlador previamente cargado (con
[`set_pid`](@ref) o [`set_controller`](@ref)) actúa sobre la planta. Los
datos se grafican en tiempo real y, al finalizar, se guardan en
`datafiles/DCmotor_profile_closed_exp.csv` y la referencia se vuelve a fijar
en `0`.

# Argumentos
- `sys::MotorSystem`: objeto de la plataforma, previamente configurado con
  un controlador.

# Argumentos de palabra clave
- `timevalues`: colección con los instantes de tiempo (en segundos,
  crecientes, idealmente comenzando en `0`) que definen el perfil.
- `refvalues`: colección, del mismo largo que `timevalues`, con los valores
  de referencia en cada instante.

# Retorna
- `(t, r, y, u)`: tupla de vectores `Vector{Float64}` con el tiempo (s), la
  referencia interpolada, la salida medida y la señal de control (voltios).

# Ejemplos
```julia
using DCMotor
sys = MotorSystem();
set_pid(sys; kp=2.0, ki=0.5, output=:angle);
t, r, y, u = profile_closed(sys; timevalues=(0, 1, 2, 3), refvalues=(0, 360, 360, 0));
```
"""
function profile_closed(sys::MotorSystem;
                        timevalues = (0, 1, 2, 3),
                        refvalues  = (0, 360, 360, 0))
    h  = SAMPLING_TIME
    bs = BUFFER_SIZE
    tv_cmd = collect(Float64, timevalues)
    rv_cmd = collect(Float64, refvalues)

    int_tv = [round(Int, p / h) for p in tv_cmd]
    if int_tv[1] != 0
        int_tv  = vcat([0, int_tv[1] - 1], int_tv)
        rv_cmd  = vcat([0.0, 0.0], rv_cmd)
    end

    mn, mx = minimum(rv_cmd), maximum(rv_cmd)
    total  = int_tv[end] + 1
    frames = ceil(Int, total / bs)

    payload = Dict(
        "timevalues" => time2hex(int_tv),
        "refvalues"  => signal2hex(rv_cmd),
        "points"     => long2hex(length(int_tv)),
        "min_val"    => float2hex(mn),
        "max_val"    => float2hex(mx),
    )

    tv, rv, yv, uv = Float64[], Float64[], Float64[], Float64[]
    span = mx - mn

    plt = plot(layout=(2, 1), size=(900, 500),
        title=["Perfil  duración=$(@sprintf("%.1f",tv_cmd[end])) s, $(length(tv_cmd)) puntos" ""],
        ylabel=["Grados (o °/s)" "Volts"],
        xlabel=["Tiempo (s)" "Tiempo (s)"],
        xlims=[(0, (total - 1) * h) (0, (total - 1) * h)],
        ylims=[(mn - 0.1*abs(span), mx + 0.1*span) (-5.5, 5.5)],
        background_color_subplot=[:ivory :mintcream],
        legend=:topright, grid=true, gridalpha=0.15, margin=5Plots.mm)
    display(plt)

    connect!(sys)
    send_command!(sys, "prof_closed", payload)

    on_frame = function(msg, frame_no)
        rf = hexframe_to_array(string(msg["r"]))
        uf = hexframe_to_array(string(msg["u"]))
        yf = hexframe_to_array(string(msg["y"]))
        tf = h .* (collect(0:length(rf)-1) .+ (frame_no - 1) * bs)
        append!(rv, rf); append!(yv, yf); append!(uv, uf); append!(tv, tf)

        plt = plot(layout=(2, 1), size=(900, 500),
            title=["Perfil  duración=$(@sprintf("%.1f",tv_cmd[end])) s, $(length(tv_cmd)) puntos" ""],
            ylabel=["Grados (o °/s)" "Volts"],
            xlabel=["Tiempo (s)" "Tiempo (s)"],
            xlims=[(0, (total - 1) * h) (0, (total - 1) * h)],
            ylims=[(mn - 0.1*abs(span), mx + 0.1*span) (-5.5, 5.5)],
            background_color_subplot=[:ivory :mintcream],
            legend=:topright, grid=true, gridalpha=0.15, margin=5Plots.mm)
        plot!(plt, subplot=1, tv, rv, label="r(t)", color=:darkgreen,
              linewidth=1.25)
        plot!(plt, subplot=1, tv, yv, label="y(t)", color=:deeppink,
              linewidth=1.25)
        plot!(plt, subplot=2, tv, uv, label="u(t)", color=:royalblue)
        redraw!(plt)
    end

    try
        receive_frames!(sys, frames, on_frame)
    catch e
        println("Error: ", e)
    end
    set_reference(sys, 0)
    save_experiment([tv, rv, yv, uv], "DCmotor_profile_closed_exp.csv", "t,r,y,u")
    return tv, rv, yv, uv
end


# ═══════════════════════════════════════════════════════════════════════════════
#  stepinfo_real
# ═══════════════════════════════════════════════════════════════════════════════


"""
Estructura con los resultados de stepinfo para datos experimentales reales.
"""
struct StepInfoSR
    y0::Float64
    yf::Float64    
    stepsize::Float64
    peak::Float64
    peaktime::Float64        # relativo al instante del escalón
    overshoot::Float64
    lowerpeak::Float64
    undershoot::Float64
    settlingtime::Float64    # relativo al instante del escalón
    risetime::Float64
    settling_th::Float64
    risetime_th::Tuple{Float64, Float64}
    u_max::Float64          #  maximo de  la señal de control 
end






"""
    stepinfo_exp(res; T=nothing, settling_th=0.02, risetime_th=(0.1, 0.9))

Calcula métricas de la respuesta al escalón a partir de datos experimentales
en lazo cerrado, y grafica la respuesta medida (con las métricas anotadas)
junto con, opcionalmente, la respuesta simulada de un modelo prototipo `T`.

Antes de calcular el sobrepico, el tiempo de estabilización y el tiempo de
subida, la salida `y` se filtra con un pasabajos (mediante `prefilter`,
frecuencia de corte 12.5 Hz) para reducir el efecto del ruido de medición.

# Argumentos
- `res`: tupla `(t, r, y, u)` con los vectores de tiempo, referencia, salida
  medida y señal de control, tal como los retorna [`step_closed`](@ref).

# Argumentos de palabra clave
- `T=nothing`: función de transferencia (normalizada, de ganancia estática
  unitaria) de un modelo prototipo en lazo cerrado; si se suministra, su
  respuesta al escalón (escalada según el tamaño del escalón experimental)
  se superpone en la gráfica para comparar con la respuesta medida.
- `settling_th::Real=0.02`: umbral, como fracción del tamaño del escalón,
  de la banda usada para el tiempo de estabilización (por defecto, 2 %).
- `risetime_th::Tuple{Real,Real}=(0.1, 0.9)`: niveles, como fracción del
  tamaño del escalón, usados para el tiempo de subida (por defecto, entre
  10 % y 90 %).

# Retorna
- `info::StepInfoSR`: estructura con las métricas calculadas: valores
  inicial y final (`y0`, `yf`), tamaño del escalón (`stepsize`), pico y
  tiempo de pico (`peak`, `peaktime`), sobrepico en % (`overshoot`),
  subpico y subpico en % (`lowerpeak`, `undershoot`), tiempo de
  estabilización (`settlingtime`), tiempo de subida (`risetime`), los
  umbrales usados (`settling_th`, `risetime_th`) y el máximo de la señal de
  control (`u_max`).

# Ejemplos
```julia
using DCMotor
sys = MotorSystem();
set_pid(sys; kp=2.0, ki=0.5, output=:angle);
result = step_closed(sys; r0=0, r1=100, t0=0.5, t1=2.0);
info = stepinfo_exp(result)
```
"""
function stepinfo_exp(res; T = nothing,  settling_th = 0.02, risetime_th = (0.1, 0.9))
    
    t1 = res[1] 
    r1 = res[2]
    y1 = res[3]
    u1 = res[4]

    u_max = maximum(abs.(u1))

    n  = length(y1)
    Ts = SAMPLING_TIME
    fs = 1.0 / Ts

    # ── Detectar instante del escalón en u ────────────────────────────
    step_idx = argmax(abs.(diff(r1))) 
    

    if step_idx != 1
        step_idx +=1
        stepsize = r1[end] - r1[1]
    else 
       stepsize = r1[end] 
    end

    t_step   = t1[step_idx] 

    # ── Valores de estado estacionario ────────────────────────────────
    #  y0: promedio de hasta 50 puntos ANTES del escalón
    #  yf: promedio de los últimos 50 puntos
    n_pre = min(step_idx, 50)
    y0    = mean(y1[step_idx - n_pre + 1 : step_idx])
    n_end = min(50, n)
    yf    = mean(y1[n - n_end + 1 : n])

    # ── Dirección y magnitud (desde u) ────────────────────────────────
    
   

    # ── Subpico (undershoot) — solo post-escalón ──────────────────────
    lowerpeak, _ =  findmin(y1) 
    undershoot   = max(100.0 * (y0 - lowerpeak) / stepsize, 0.0)

    

    
    
    data = iddata(y1, u1, SAMPLING_TIME)
    data = prefilter(data, 0, 12.5)
   
    y_filtered = data.y
    y_filtered = y_filtered[step_idx: end]
    
    
    y = y_filtered #[step_idx: end]
    t = t1[step_idx: end] .- t_step
    r = r1[step_idx: end]

    band           = settling_th * stepsize
    y_rev = reverse(y_filtered)
    idx_rev        = findfirst(abs.(y_rev[10:end] .- yf) .> band)
    t_rev    = reverse(t)[10:end]
     
    if idx_rev === nothing
        settlingtime = NaN
    else
        settlingtime = ( t_rev[idx_rev] + t_rev[idx_rev-1] ) / 2 
    end



       # ── Sobrepico (overshoot) — solo post-escalón ─────────────────────
    peak, pidx = findmax(y) 
    peaktime   = t[pidx] 
    overshoot  = 100.0 * (peak - yf) / stepsize

 

    # ── Tiempo de subida — interpolación suave post-escalón ───────────
    #  filtfilt (cero fase) Butterworth orden 2, fc=5 Hz.
    #  Solo primer cruce para evitar efecto del ruido.
  
    
   

    lv10 = y0 + risetime_th[1] * stepsize 
    lv90 = y0 + risetime_th[2] * stepsize 
    
    i10  = findfirst(y .> lv10)
    i90  = findfirst(y .> lv90)

    if i10 === nothing || i90 === nothing
        risetime = NaN
    else
        t10      = _interp_cross(t, y, lv10, i10)
        t90      = _interp_cross(t, y, lv90, i90)
        risetime = t90- t10
    end

    info = StepInfoSR(y0, yf, stepsize, peak, peaktime, overshoot,
                      lowerpeak, undershoot, settlingtime, risetime,
                      settling_th, risetime_th,  u_max)

    _plot_stepinfo(t, r, y, t10, t90,i10, i90,  info, T)
    return info
end

# ── Interpolación lineal para cruce preciso ───────────────────────────
function _interp_cross(t, y, level, idx)
    idx == 1 && return t[1]
    α = (level - y[idx-1]) / (y[idx] - y[idx-1])
    return t[idx-1] + α * (t[idx] - t[idx-1])
end

# ── Gráfica con anotaciones ──────────────────────────────────────────
function _plot_stepinfo(t, r, y, t10, t90, i10, i90,  si, T)    
    
    leg_size =10
    p =    plot( t, r;
        label = "r(t)", legendfontsize = leg_size, lw = 1.5, color = :green, 
        alpha=0.7, background_color=:mintcream, seriestype=:steppre,  margin=5Plots.mm)
    plot!( [0, 0], [r[end]-si.stepsize, r[end]];
        label = "", lw = 1.5, color = :green, 
        alpha=0.7, background_color=:mintcream)


        

    if  T !== nothing
        ref0 = si.stepsize
        tsim = 0: 1/500: t[end]
        res = step(T*ref0, tsim)
        plot!(p, res.t, res.y'.+ (r[end]-si.stepsize), label="y(t)  (simulada)", 
        legendfontsize = leg_size, color = "#00aad4", alpha=0.75)
    
    end      
    plot!(t, y;
        label  = "y(t) medida (yf = $(@sprintf("%.2f",si.yf)))",
        lw     = 2, color = "#ff2a7f", alpha = 1,
        xlabel = "Tiempo (s)", ylabel = "Amplitud",
        title  = "Respuesta experimental al Escalón",
        legend = :right, size = (900, 500),     
        )

    
    # Banda de estabilización
    hline!(p, [si.yf + si.settling_th * si.stepsize,
               si.yf - si.settling_th * si.stepsize];
        ls = :dot, color = :orange, alpha = 0.5, label ="" )

    # Pico (sobrepico) — posición absoluta
    scatter!(p, [si.peaktime], [si.peak];
        ms = 5, color = :red, markershape = :circle,
        label = "Pico = $(@sprintf("%.2f",si.peak)) (SP = $(@sprintf("%.2f",si.overshoot)))")

    # Tiempo de estabilización — posición absoluta
    vline!(p, [si.settlingtime];
        ls = :dashdot, color = :orange, lw = 1.25,
        label = "Ts = $(@sprintf("%.2f",si.settlingtime)) s")

    # Niveles 10% y 90% para tiempo de subida
    if !isnan(si.risetime)
        lv10 = si.y0 + si.risetime_th[1] * si.stepsize
        lv90 = si.y0 + si.risetime_th[2] * si.stepsize
        vline!(p, [t10, t90];
            ls = :dashdot, color = :purple, alpha = 0.4, lw=1.25, label = "")
        annotate!(p, t10 + 0.4*si.risetime , lv10, text("10%", 7, :purple))
        annotate!(p, t90 - 0.4*si.risetime, lv90, text("90%", 7, :purple))
        #plot!(t[i10:i90], y[i10:i90], color = "#800033", label ="")
    end

    # Leyenda informativa
    plot!(p, Float64[], Float64[];
        label = "Tr = $(@sprintf("%.2f",si.risetime)) s", lw = 1,
        ls = :dashdot, color = :purple, alpha = 0.6)    
             

    redraw!(p)
end

# ── Mostrar resumen en consola ────────────────────────────────────────
function Base.show(io::IO, si::StepInfoSR)
    println(io, "Info Resp. Escalón:")
    @printf(io, "  %-18s %8.3f\n",   "Valor inicial:",    si.y0)
    @printf(io, "  %-18s %8.3f\n",   "Valor final:",      si.yf)
    @printf(io, "  %-18s %8.3f\n",   "Cambio escalón:",   si.stepsize)
    @printf(io, "  %-18s %8.3f\n",   "Pico:",             si.peak)
    @printf(io, "  %-18s %8.3f s\n", "Tiempo pico:",      si.peaktime)
    @printf(io, "  %-18s %8.2f %%\n","Sobrepico:",        si.overshoot)
    @printf(io, "  %-18s %8.3f s\n", "T. estabilización:",    si.settlingtime)
    @printf(io, "  %-18s %8.3f s\n", "T. subida:",        si.risetime)
    @printf(io, "  %-18s %8.3f V\n", "Max. |u(t)|:",        si.u_max)
end



