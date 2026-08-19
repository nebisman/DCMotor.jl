# ═══════════════════════════════════════════════════════════════════════════════
#  identsys.jl – Funciones de identificación para el sistema UNDCMotor.
#  Equivalente a identsys.py.   LB 2026 – MIT License
# ═══════════════════════════════════════════════════════════════════════════════

using Plots
using ControlSystemsBase
using LinearAlgebra
using Statistics
using Printf
using ControlSystemIdentification
using LaTeXStrings


# ═══════════════════════════════════════════════════════════════════════════════
#  step_open  (respuesta escalón en lazo abierto – velocidad)
# ═══════════════════════════════════════════════════════════════════════════════

"""
    step_open(sys::MotorSystem; u0=1.5, u1=3.5, t0=1.0, t1=1.0)

Ejecuta un experimento de respuesta al escalón en lazo abierto: el voltaje
aplicado al motor permanece en `u0` durante `t0` segundos y luego salta a
`u1` durante `t1` segundos (sin ningún controlador activo). Se usa
típicamente para identificar el modelo dinámico de la velocidad angular. Los
datos se grafican en tiempo real y, al finalizar, se guardan en
`datafiles/DCmotor_step_open_exp.csv` y la referencia se vuelve a fijar en
`0`.

# Argumentos
- `sys::MotorSystem`: objeto de la plataforma.

# Argumentos de palabra clave
- `u0::Real=1.5`: voltaje inicial (bajo), en voltios.
- `u1::Real=3.5`: voltaje final (alto), en voltios, tras el escalón.
- `t0::Real=1.0`: duración en segundos con el voltaje en `u0`.
- `t1::Real=1.0`: duración en segundos con el voltaje en `u1`.

# Retorna
- `(t, u, y)`: tupla de vectores `Vector{Float64}` con el tiempo (s), el
  voltaje aplicado (V) y la velocidad angular medida (°/s).

# Ejemplos
```julia
using DCMotor
sys = MotorSystem();
t, u, y = step_open(sys; u0=1.0, u1=4.0, t0=1.0, t1=2.0);
```
"""
function step_open(sys::MotorSystem;
                   u0::Real = 1.5, u1::Real = 3.5,
                   t0::Real = 1.0, t1::Real = 1.0)
    h  = SAMPLING_TIME
    bs = BUFFER_SIZE
    pts_low  = round(Int, t0 / h)
    pts_high = round(Int, t1 / h) + 1
    total    = pts_low + pts_high
    frames   = ceil(Int, total / bs)

    payload = Dict(
        "low_val"     => float2hex(u0),
        "high_val"    => float2hex(u1),
        "points_low"  => long2hex(pts_low),
        "points_high" => long2hex(pts_high),
    )

    tv, uv, yv = Float64[], Float64[], Float64[]

    ulim = sort([u0, u1])
    ylim_v = sort([Float64(speed_from_volts(sys, v)) for v in ulim])
    du = ulim[2] - ulim[1]

    plt = plot(layout=(2, 1), size=(900, 500),
        title=["Step lazo abierto  duración=$(@sprintf("%.2f",total*h)) s" ""],
        ylabel=["Grados/s" "Volts"],
        xlabel=["Tiempo (s)" "Tiempo (s)"],
        xlims=[(0, (total - 1) * h) (0, (total - 1) * h)],
        ylims=[(ylim_v[1] - 50, ylim_v[2] + 50) (ulim[1] - 0.1*du, ulim[2] + 0.1*du)],
        background_color_subplot=[:ivory :mintcream],
        legend=:bottomright, grid=true, gridalpha=0.15, margin=5Plots.mm)
    display(plt)

    connect!(sys)
    send_command!(sys, "step_open", payload)

    on_frame = function(msg, frame_no)
        uf = hexframe_to_array(string(msg["u"]))
        yf = hexframe_to_array(string(msg["y"]))
        tf = h .* (collect(0:length(yf)-1) .+ (frame_no - 1) * bs)
        append!(tv, tf); append!(uv, uf); append!(yv, yf)

        plt = plot(layout=(2, 1), size=(900, 500),
            title=["Step lazo abierto  duración=$(@sprintf("%.2f",total*h)) s" ""],
            ylabel=["Grados/s" "Volts"],
            xlabel=["Tiempo (s)" "Tiempo (s)"],
            xlims=[(0, (total - 1) * h) (0, (total - 1) * h)],
            ylims=[(ylim_v[1] - 50, ylim_v[2] + 50) (ulim[1] - 0.1*du, ulim[2] + 0.1*du)],
            background_color_subplot=[:ivory :mintcream],
            legend=:bottomright, grid=true, gridalpha=0.15, margin=5Plots.mm)
        plot!(plt, subplot=1, tv, yv,
              label="y(t) velocidad", color=:deeppink, linewidth=1.5)
        plot!(plt, subplot=2, tv, uv,
              label="u(t) voltaje", color=:green, linewidth=1.0,
              seriestype=:steppre)
        redraw!(plt)
    end

    try
        receive_frames!(sys, frames, on_frame; timeout_factor=10.0)
    catch e
        println("Error: ", e)


    end
    set_reference(sys, 0)
  # Volver a referencia cero al finalizar

    save_experiment([tv, uv, yv], "DCmotor_step_open_exp.csv", "t,u,y")
    return tv, uv, yv
end


# ═══════════════════════════════════════════════════════════════════════════════
#  prbs_open  (identificación con señal PRBS en lazo abierto)
# ═══════════════════════════════════════════════════════════════════════════════

"""
    prbs_open(sys::MotorSystem; low_val=2.0, high_val=4.0, divider=2)

Ejecuta un experimento de identificación en lazo abierto excitando el motor
con una señal binaria pseudoaleatoria (PRBS) que alterna entre `low_val` y
`high_val` voltios. La secuencia tiene una longitud fija `PRBS_LENGTH`
(1023 bits, definida por el firmware), y cada bit se mantiene durante
`divider` periodos de muestreo (`SAMPLING_TIME`). Los datos se grafican en
tiempo real (con una ventana deslizante) y, al finalizar, se guardan en
`datafiles/DCmotor_prbs_open_exp.csv` y la referencia se vuelve a fijar en
`0`.

# Argumentos
- `sys::MotorSystem`: objeto de la plataforma.

# Argumentos de palabra clave
- `low_val::Real=2.0`: nivel bajo de la señal PRBS, en voltios.
- `high_val::Real=4.0`: nivel alto de la señal PRBS, en voltios.
- `divider::Int=2`: número de periodos de muestreo durante los cuales se
  mantiene cada bit de la secuencia PRBS (a mayor `divider`, señal más
  "lenta" y de mayor duración total).

# Retorna
- `(t, u, y)`: tupla de vectores `Vector{Float64}` con el tiempo (s), el
  voltaje aplicado (V) y la velocidad angular medida (°/s).

# Ejemplos
```julia
using DCMotor
sys = MotorSystem();
t, u, y = prbs_open(sys; low_val=1.5, high_val=4.5, divider=3);
```
"""
function prbs_open(sys::MotorSystem;
                   low_val::Real = 2.0, high_val::Real = 4.0,
                   divider::Int = 2)
    h  = SAMPLING_TIME
    bs = BUFFER_SIZE
    total  = PRBS_LENGTH * divider
    frames = ceil(Int, total / bs)

    payload = Dict(
        "low_val"  => float2hex(low_val),
        "high_val" => float2hex(high_val),
        "divider"  => long2hex(divider),
    )

    tv, uv, yv = Float64[], Float64[], Float64[]

    ulim = sort([low_val, high_val])
    ylim_v = sort([Float64(speed_from_volts(sys, v)) for v in ulim])
    du = ulim[2] - ulim[1]

    plt = plot(layout=(2, 1), size=(900, 500),
        title=["PRBS  $total muestras, duración=$(@sprintf("%.2f",total*h)) s" ""],
        ylabel=["Velocidad (°/s)" "Voltaje (V)"],
        xlabel=["" "Tiempo (s)"],
        xlims=[(0, 5*bs*h) (0, 5*bs*h)],
        ylims=[(ylim_v[1] - 25, ylim_v[2] + 25) (ulim[1] - 0.1*du, ulim[2] + 0.1*du)],
        background_color_subplot=[:ivory :mintcream],
        legend=:bottomright, grid=true, gridalpha=0.15, margin=5Plots.mm)
    display(plt)

    connect!(sys)
    send_command!(sys, "prbs_open", payload)
    

    on_frame = function(msg, frame_no)
        uf = hexframe_to_array(string(msg["u"]))
        yf = hexframe_to_array(string(msg["y"]))
        tf = h .* (collect(0:length(uf)-1) .+ (frame_no - 1) * bs)
        append!(tv, tf); append!(uv, uf); append!(yv, yf)

        # Ventana deslizante después de 6 tramas
        if frame_no > 6
            x_lo = tv[max(1, end - 6*bs + 1)]
            x_hi = tv[end]
            xl = (x_lo, x_hi)
        else
            xl = (0, 5*bs*h)
        end     

        plt = plot(layout=(2, 1), size=(900, 500),
            title=["PRBS  $total muestras, duración=$(@sprintf("%.2f",total*h)) s" ""],
            ylabel=["Velocidad (°/s)" "Voltaje (V)"],
            xlabel=["" "Tiempo (s)"],
            xlims=[xl xl],
            ylims=[(ylim_v[1] - 25, ylim_v[2] + 25) (ulim[1] - 0.1*du, ulim[2] + 0.1*du)],
            background_color_subplot=[:ivory :mintcream],
            legend=:bottomright, grid=true, gridalpha=0.15, margin=5Plots.mm)
        plot!(plt, subplot=1, tv, yv, label="y(t)", color=:deeppink,
              seriestype=:steppre)
        plot!(plt, subplot=2, tv, uv, label="PRBS", color=:green,
              seriestype=:steppre)
        redraw!(plt)
    end

    try
        receive_frames!(sys, frames, on_frame; timeout_factor=20.0)
    catch e
        println("Error: ", e)

    end
    set_reference(sys, 0)
    # Volver a referencia cero al finalizar        

    save_experiment([tv, uv, yv], "DCmotor_prbs_open_exp.csv", "t,u,y")
    return tv, uv, yv
end


# ═══════════════════════════════════════════════════════════════════════════════
#  _step_open_static  (step simplificado para curva estática)
# ═══════════════════════════════════════════════════════════════════════════════

"""Step open para curva estática. `sys` ya debe estar conectado."""

function _step_open_static(sys::MotorSystem, u1::Real,  t1::Real, uee::Vector{Float64}, yee::Vector{Float64})
  
    
    h  = SAMPLING_TIME
    bs = BUFFER_SIZE
    leg_size = 8
    pts_high = round(Int, t1 / h) + 1
    frames   = ceil(Int, pts_high / bs)

    payload = Dict(
        "low_val"     => float2hex(0.0),
        "high_val"    => float2hex(u1),
        "points_low"  => long2hex(0),
        "points_high" => long2hex(pts_high),
    )

    tv, uv, yv = Float64[], Float64[], Float64[]
   
   
                 
    plt = plot( layout=(1, 2), size=(900, 500),
    title=["Experimento actual" "Curva estática – UNDCMotor"],
    xlabel=["Tiempo (s)" "Voltaje (V)"],
    ylabel=["Velocidad (°/s)" "Velocidad estacionaria (°/s)"], 
    xlims=[(0, t1) (0,5)],
    ylims=[(-30, 830) (-30, 830)],
    background_color_subplot=[:mintcream :ivory ],
    legend=[:bottomright :topleft], 
    grid=true, gridalpha=0.2,
    xticks=0:1:10, yticks=0:100:800,
    margin=5Plots.mm)

    if !isempty(yee)
         scatter!(subplot=2, uee, yee, color=:green, marker=:circle, label="",
               markersize=3, linewidth=1)
         scatter!(subplot=2, [uee[end]], [yee[end]], color=:orange, marker=:circle,
          legendfontsize = leg_size, fg_legend = "#d4aa00", markersize=3, linewidth=1)
     end


    redraw!(plt)

    connect!(sys)
    send_command!(sys, "step_open", payload)

    on_frame = function(msg, frame_no)
       
        uf = hexframe_to_array(string(msg["u"]))
        yf = hexframe_to_array(string(msg["y"]))
        tf = h .* (collect(0:length(yf)-1) .+ (frame_no - 1) * bs)
        append!(tv, tf); append!(uv, uf); append!(yv, yf)
        
        pl_ee = scatter(uee, yee, color=:green, marker=:circle, label="",
               markersize=3, linewidth=1)
                 

        pl_ins = plot(tv, yv, color="#00aad4",
                legendfontsize = leg_size, fg_legend= "#5fbcd3", linewidth=1.5)        
          
            
        plt = plot(pl_ins, pl_ee, layout=(1, 2), size=(900, 500),
        title=["Experimento actual" "Curva estática – UNDCMotor"],
        xlabel=["Tiempo (s)" "Voltaje (V)"],
        ylabel=["Velocidad (°/s)" "Velocidad estacionaria (°/s)"], 
        xlims=[(0, t1) (0,5)],
        ylims=[(-30, 830) (-30, 830)],
        background_color_subplot=[:mintcream :ivory ],
        legend=[:bottomright :topleft], 
        grid=true, gridalpha=0.2,
        xticks=0:1:10, yticks=0:100:800,
        margin=5Plots.mm)
        redraw!(plt)
  
    end

    try
        receive_frames!(sys, frames, on_frame; timeout_factor=10.0)
    catch e
        println("Error: ", e)
    end
    yss = Float64(mean(yv[end-49:end]))

    return yss
end


# ═══════════════════════════════════════════════════════════════════════════════
#  get_static_model  (curva estática del motor)
# ═══════════════════════════════════════════════════════════════════════════════

"""
    get_static_model(sys::MotorSystem; points=20)

Obtiene experimentalmente la curva estática (voltaje vs. velocidad angular
estacionaria) del motor DC. Aplica sucesivamente `points` niveles de voltaje
(5 puntos finos dentro de la zona muerta, entre 0.15 y 0.25 V, y el resto
espaciados uniformemente entre 0.3 y 5 V), midiendo en cada uno la velocidad
estacionaria como el promedio de las últimas muestras de un escalón de 3
segundos ([`step_open`](@ref) simplificado). Ajusta por mínimos cuadrados un
modelo lineal `y = K*u + b` sobre los puntos fuera de la zona muerta, y
estima el voltaje de zona muerta `zm`. Muestra una gráfica con los datos, la
recta ajustada y la zona muerta, y guarda los resultados en
`datafiles/DCmotor_static_gain_response.csv` y
`datafiles/DCmotor_static_pars.csv` (usados luego por
[`speed_from_volts`](@ref) y [`volts_from_speed`](@ref)). Al finalizar, fija
la referencia en `0`.

# Argumentos
- `sys::MotorSystem`: objeto de la plataforma.

# Argumentos de palabra clave
- `points::Int=20`: número total de niveles de voltaje a barrer.

# Retorna
- `(uee, yee)`: tupla de vectores `Vector{Float64}` con los voltajes
  aplicados (V) y las velocidades estacionarias medidas (°/s) en cada uno.

# Ejemplos
```julia
using DCMotor
sys = MotorSystem();
uee, yee = get_static_model(sys; points=25);
```
"""
function get_static_model(sys::MotorSystem; points::Int = 20)
    timestep = 3.0
    dz_points = 5
    u_dz = range(0.15, 0.25,  length=dz_points)
    u_pos = range(0.3,5, length = points-dz_points)
    u_all = vcat(u_dz  , u_pos )
  


    uee = Vector{Float64}()
    yee = Vector{Float64}()

    connect!(sys)
  
    for ui in u_all
        yss = _step_open_static(sys, ui, timestep, uee, yee)  

        uss = Float64(ui)        
        push!(uee, uss)
        push!(yee, yss)

    end

     # ── Regresión lineal: yee = K*uee + b 
    
    indices = findall(yee .> 2)
    yee1 = yee[indices]
    uee1 = uee[indices]
    
    A      = hcat(uee1, ones(length(uee1)))   # matriz de diseño [u 1]
    coeffs = A \ yee1                        # mínimos cuadrados
    K, b   = coeffs[1], coeffs[2]

    u_fit  = range(minimum(uee), maximum(uee), length=100)
    y_fit  = K .* u_fit .+ b

    R²     = 1 - sum((yee1 .- (K .* uee1 .+ b)).^2) /
                    sum((yee1 .- mean(yee1)).^2)

    zm = uee1[1]

     # Datos experimentales
    datos = scatter(uee, yee,
        label="Datos experimentales",
         color=:green, marker=:circle, markersize=4, linewidth=1.5, opacity=0.7)

    # Modelo lineal
    leg_size = 10
    mod_str = latexstring(@sprintf("\\omega_{ee} = %.2f\\,u -  \\,%0.2f\\, \\, (R^2 = %0.3f)", K, abs(b), R²))
    plot!(datos, collect(u_fit), y_fit,
        label = mod_str,
        color="#0055d4", linewidth=1,  legendfontsize = leg_size,  linestyle=:dash)
    
    scatter!(datos,  [zm], [0],
        label="Zona muerta: $(@sprintf("%.2f",zm)) V",
        color=:red, marker=:square, markersize=4, linewidth=1.5)
    
    plt = plot(datos,  size=(600,600), 
        title="Curva estática – UNDCMotor",
        xlabel="Tensión  (V)",
        ylabel="Velocidad estacionaria (°/s)",
        xlims=(0, 5.1), ylims=(0, maximum(yee)+30),
        background_color_subplot=:ivory,
        xticks=0:1:5, yticks=0:100:800,
        legend=:topleft, grid=true, gridalpha=0.2,  
        margin=2Plots.mm)
    display(plt)


    exp_data = hcat(uee1, yee1)
    open(_datafile("DCmotor_static_gain_response.csv"), "w") do io
            println(io, "u,y")
            for i in axes(exp_data, 1)
                @printf(io, "%.8f,%.8f\n", exp_data[i, 1], exp_data[i, 2])
            end
        end
    open(_datafile("DCmotor_static_pars.csv"), "w") do io
            println(io, "K, b ,zm")
            @printf(io, "%.8f,%.8f,%.8f\n", K, b, zm)
    end

    
    
    set_reference(sys, 0)
    # Volver a referencia cero al finalizar
    println("Modelo estático completado")
    return uee, yee
end


# ═══════════════════════════════════════════════════════════════════════════════
#  get_fomodel_step  (modelo de primer orden desde step)
# ═══════════════════════════════════════════════════════════════════════════════

"""
    get_fomodel_step(sys::MotorSystem; yop=400, sigma=100, usefile=false)

Estima un modelo de primer orden con retardo (FOTD, *First Order plus Time
Delay*) de la velocidad angular del motor DC, linealizado alrededor de un
punto de operación `yop` (en °/s). Calcula los voltajes de excitación `ua` y
`ub` (mediante [`volts_from_speed`](@ref)) de modo que el escalón cubra
aproximadamente el rango `[yop - sigma, yop + sigma]`, ejecuta (o reutiliza,
si `usefile=true`) un experimento de [`step_open`](@ref), detecta el
instante del escalón y ajusta por mínimos cuadrados el retardo `L` y la
constante de tiempo `τ` a partir de los tiempos de cruce de la respuesta
normalizada en los niveles 10 %, 15 %, 20 %, 30 % y 50 %. La ganancia
estática `α = Δy/Δu` se calcula directamente de los valores estacionarios
antes y después del escalón. Se muestra una gráfica comparando los datos
experimentales con el modelo simulado, y se guardan los parámetros en
`datafiles/DCmotor_fo_model.csv`.

El modelo retornado tiene la forma `G(s) = b/(s+a)`, equivalente a
`α/(τs+1)` con `b = α/τ` y `a = 1/τ` (el retardo `L` no se incluye como
retardo puro en la función de transferencia, pero se estima, se guarda en el
archivo de parámetros y se retorna aparte).

# Argumentos
- `sys::MotorSystem`: objeto de la plataforma.

# Argumentos de palabra clave
- `yop::Real=400`: punto de operación (velocidad, en °/s) alrededor del cual
  se linealiza el modelo.
- `sigma::Real=100`: semiancho (en °/s) de la banda de excitación alrededor
  de `yop`.
- `usefile::Bool=false`: si es `true`, reutiliza el último experimento
  guardado en `datafiles/DCmotor_step_open_exp.csv` en lugar de ejecutar uno
  nuevo.

# Retorna
- `G::ControlSystems.TransferFunction`: modelo continuo de primer orden
  `b/(s+a)` de la velocidad angular.
- `L::Float64`: retardo estimado, en segundos.

# Ejemplos
```julia
using DCMotor
sys = MotorSystem();
G, L = get_fomodel_step(sys; yop=300, sigma=80);
```
"""
function get_fomodel_step(sys::MotorSystem;
                          yop::Real = 400, sigma::Real = 100, usefile::Bool = false)

    ymax = speed_from_volts(sys, 5)
    timestep = 3.0

    
    # Punto de operación → voltajes de excitación

    if 0 <= yop < 2*sigma
        ua = volts_from_speed(sys, 50)
        ub = volts_from_speed(sys, 100)

    elseif ymax - sigma <= yop <= ymax
        ua = volts_from_speed(sys, yop - sigma);
        ub = 5
    elseif 2*sigma <= yop <  ymax - sigma
        ua = volts_from_speed(sys, yop - sigma)
        ub = volts_from_speed(sys, yop + sigma)
    else
        error("Velocidad fuera de rango [$(@sprintf("%.1f",ymin)), $(@sprintf("%.1f",ymax))]")
    end

    if !usefile
        t, u, y = step_open(sys; u0=ua, u1=ub, t0=timestep, t1=timestep)
    else
        t, u, y = read_csv_file3(_datafile("DCmotor_step_open_exp.csv"))
    end

    # Detectar el escalón
    ind_step = argmax(diff(u)) +1


    y = y[max(1, ind_step - 50):end]
    u = u[max(1, ind_step - 50):end]
    t = t[max(1, ind_step - 50):end]
    t = t .- t[min(51, length(t))]  # t=0 en el escalón

    ua_d = u[1]
    ub_d = u[end]
    ya = Float64(mean(y[1:min(50, length(y))]))
    yb = Float64(mean(y[max(1, end-49):end]))
    delta_u = ub_d - ua_d
    delta_y = yb - ya

    # Normalizar la respuesta
    y_norm = (y .- ya) ./ delta_y

    # LSQ para estimar tau y L
    yi_pts = [0.1, 0.15, 0.2, 0.3, 0.5]
    # Interpolación lineal para encontrar los tiempos correspondientes
    ti_pts = Float64[]
    for yp in yi_pts
        idx = findfirst(>=(yp), y_norm)
        if idx === nothing || idx <= 1
            push!(ti_pts, t[1])
        else
            # Interpolación lineal entre idx-1 e idx
            frac = (yp - y_norm[idx-1]) / (y_norm[idx] - y_norm[idx-1])
            push!(ti_pts, t[idx-1] + frac * (t[idx] - t[idx-1]))
        end
    end
    print(ti_pts)
    # Sistema lineal: ti = L + tau * (-ln(1 - yi))
    A_lsq = hcat(ones(length(yi_pts)), [-log(1 - p) for p in yi_pts])
    # Resolver con restricciones L >= 0, tau >= 0.1
    # Usando mínimos cuadrados simples
    x = A_lsq \ ti_pts
    
    L_val = max(0.01, x[1])     
    tau =  x[2]
    alpha = delta_y / delta_u

    # Modelo simulado
    tsim = range(t[1], t[end] , length=500)
    ymodel = [alpha * delta_u * (1 - exp(-max(0, ti - L_val) / tau)) + ya for ti in tsim]
    

    # Gráfica
    model_str = @sprintf("G(s) = %.3f/(%.3fs+1) * exp(%.3f)%%", alpha, tau, L_val)

    plt = plot(layout=(2, 1), size=(900, 550),
        title=["Modelo FOTD estimado para UNDCMotor" ""],
        ylabel=["Velocidad (°/s)" "Volts"],
        xlabel=["Tiempo (s)" "Tiempo (s)"],
        xlims=[(-1, timestep) (-1, timestep)],
        background_color_subplot=[:ivory :mintcream],
        legend=:bottomright, grid=true, gridalpha=0.15, margin=5Plots.mm)
    plot!(plt, subplot=1, t, y, label="Datos", color=:teal,
          linewidth=1.5, linestyle=:dot)
    plot!(plt, subplot=1, tsim, ymodel, label=model_str, color=:deeppink,
          linewidth=1.5)
    plot!(plt, subplot=2, t, u, label="Entrada", color=:green)
    display(plt)


    s = tf("s")
    b = alpha / tau
    a = 1 / tau
    G = b /(s+a) 

    open(_datafile("DCmotor_fo_model.csv"), "w") do io
            println(io, "b,a,L")
            @printf(io, "%.8f,%.8f,%.3f\n", b, a, L_val)
    end    
    return G, L_val
end

# ═══════════════════════════════════════════════════════════════════════════════
#  get_models_prbs  (modelo de primer orden desde PRBS)
# ═══════════════════════════════════════════════════════════════════════════════

"""
    get_model_prbs(sys::MotorSystem; yop=400, sigma=100, usefile=false)

Estima un modelo de primer orden de la velocidad angular del motor DC a
partir de un experimento de identificación con señal PRBS, linealizado
alrededor de un punto de operación `yop` (en °/s). Calcula los voltajes
`low_val`/`high_val` de la misma forma que [`get_fomodel_step`](@ref),
ejecuta (o reutiliza, si `usefile=true`) un experimento con
[`prbs_open`](@ref) (`divider=4`), remueve las medias de entrada y salida, y
ajusta un modelo ARX discreto de orden (1,1) con retardo de entrada de una
muestra, previo filtrado paso banda de los datos (`prefilter`, entre 0 y
12.5 rad/s, para eliminar la tendencia constante), usando el estimador de
variables instrumentales de `ControlSystemIdentification.jl`. El modelo
discreto se convierte a tiempo continuo. Se muestra una gráfica comparando
la salida medida con la simulada por el modelo (indicando el porcentaje de
ajuste, *FIT*), y se guardan los parámetros en
`datafiles/DCmotor_fo_model.csv`.

# Argumentos
- `sys::MotorSystem`: objeto de la plataforma.

# Argumentos de palabra clave
- `yop::Real=400`: punto de operación (velocidad, en °/s) alrededor del cual
  se linealiza el modelo.
- `sigma::Real=100`: semiancho (en °/s) de la banda de excitación alrededor
  de `yop`.
- `usefile::Bool=false`: si es `true`, reutiliza el último experimento
  guardado en `datafiles/DCmotor_prbs_open_exp.csv` en lugar de ejecutar uno
  nuevo.

# Retorna
- `G1::ControlSystems.TransferFunction`: modelo continuo de primer orden de
  la velocidad angular, identificado a partir de los datos PRBS.
- `L::Float64`: retardo, fijado en un periodo de muestreo.

# Ejemplos
```julia
using DCMotor
sys = MotorSystem();
G1, L = get_model_prbs(sys; yop=350, sigma=60);
```
"""
function get_model_prbs(sys::MotorSystem;
                         yop::Real = 400, sigma::Real = 100, usefile::Bool = false)
    ymax = speed_from_volts(sys, 5)
    

    # Punto de operación → voltajes de excitación

    if 0 <= yop < 2*sigma
        ua = volts_from_speed(sys, 50)
        ub = volts_from_speed(sys, 100)

    elseif ymax - sigma <= yop <= ymax
        ua = volts_from_speed(sys, yop - sigma);
        ub = 5
    elseif 2*sigma <= yop <  ymax - sigma
        ua = volts_from_speed(sys, yop - sigma)
        ub = volts_from_speed(sys, yop + sigma)
    else
        error("Velocidad fuera de rango [$(@sprintf("%.1f",ymin)), $(@sprintf("%.1f",ymax))]")
    end

    if !usefile
        prbs_open(sys; low_val=ua, high_val=ub, divider=4)
    end

    t, u, y = read_csv_file3(_datafile("DCmotor_prbs_open_exp.csv"))
    ymean_val = Float64(mean(y))
    
    # removing means
    um = u .- mean(u) 
    ym = y .- ymean_val


    # Formatear entrada como matriz (1 × N) para lsim
  

    na, nb = 1, 1 
    data = iddata(ym, um, SAMPLING_TIME)
    data = prefilter(data,0, 12.5)  # Eliminar tendencia constante
    Gh = arx(data, na, nb, inputdelay=1, estimator = wtls_estimator(data.y, na, nb)) 
    G1 = d2c(Gh)
    u_matrix = reshape(um, 1, :)
    res = lsim(G1, u_matrix, t)
    ysim =  vec(res.y)
    r1 = modelfit(ym, ysim)
   


    # ── Gráfica comparativa ──────────────────────────────────────────
    
    b = numvec(G1)[1][1];
    a = denvec(G1)[1][2];


    modelstr1 = latexstring(@sprintf("G(s) = \\frac{%.4f}{s + %.3f} \\quad (FIT=%.1f\\,\\%%)", b, a, r1))

  

    xlims_v = (t[end] - 20, t[end])
    plt = plot(layout=(2, 1), size=(900, 550),
        title=["Modelos PRBS  yOP=$(@sprintf("%.0f",yop)) °/s" ""],
        ylabel=["Velocidad (°/s)" "Voltaje (V)"],
        xlabel=["Tiempo (s)" "Tiempo (s)"],
        xlims=[xlims_v xlims_v],
        background_color_subplot=[:ivory :mintcream],
        legend=:bottomleft, grid=true, gridalpha=0.15, margin=5Plots.mm)
    plot!(plt, subplot=1, t, ym .+ ymean_val,
          label="Datos", color=:teal, linewidth=1.5, linestyle=:dot)
    plot!(plt, subplot=1, t, ysim .+ ymean_val,
          label=modelstr1, color=:deeppink, linewidth=1.5)
    plot!(plt, subplot=2, t, u,
          label="PRBS", color=:green)
    redraw!(plt)
   

    L = SAMPLING_TIME  # Retardo de una muestra
    open(_datafile("DCmotor_fo_model.csv"), "w") do io
            println(io, "b,a,L")
            @printf(io, "%.8f,%.8f,%.3f\n", b, a, SAMPLING_TIME)
    end

    return G1, L
end



