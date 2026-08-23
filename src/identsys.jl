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

Configura y ejecuta un experimento para observar la respuesta de la
velocidad angular de la plataforma **DCMotor** ante un escalón de voltaje aplicado en
lazo abierto (es decir, sin ningún controlador activo). 

La siguiente figura muestra los parámetros de esta función:


![Respuesta al escalón en lazo abierto de la plataforma DCMotor](../../assets/step_open.png)

Los parámetros `u0`, `u1`, `t0` y `t1` mostrados en la figura se describen 
a continuación.

# Argumentos
- `sys::MotorSystem`: objeto de la plataforma.

# Argumentos de palabra clave
- `u0::Real=1.5`: voltaje inicial [V] de la señal de entrada.
- `u1::Real=3.5`: voltaje final [V] de la señal de entrada, tras el escalón.
- `t0::Real=1.0`: duración en segundos con el voltaje en `u0`.
- `t1::Real=1.0`: duración en segundos con el voltaje en `u1`.

# Retorna
- `(t, u, y)`: tupla de vectores `Vector{Float64}` con el tiempo (s), el
  voltaje aplicado [V] y la velocidad angular medida [°/s].

# Notas
- Se usa típicamente para identificar el modelo dinámico de la velocidad
  angular de la plataforma **DCMoptor**. La respuesta se grafica en tiempo real.
- Los datos del experimento se guardan en un archivo CSV en
  `datafiles/DCmotor_step_open_exp.csv`.

# Ejemplo
Primero, asegúrese de haber importado el paquete DCMotor y de haber
definido el sistema, así:

```julia
using DCMotor
sys = MotorSystem();
```

Luego, obtenga la respuesta al escalón en lazo abierto así:

```julia
t, u, y = step_open(sys; u0=1.0, u1=3.0, t0=1.0, t1=2.0);
```
"""
function step_open(sys::MotorSystem;
                   u0::Real = 1.5, u1::Real = 3.5,
                   t0::Real = 1.0, t1::Real = 1.0)


    if !(-5.0 <= u0 <=5.0)
        error("u0 debe estar entre -5 y 5")
    end

    if !(-5.0 <= u1 <= 5.0)
        error("u1 debe estar entre -5 y 5")
    end   

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

Configura y ejecuta un experimento de identificación en lazo abierto,
excitando la plataforma **DCMotor** con una señal binaria pseudoaleatoria (PRBS) que
alterna entre los valores `low_val` y `high_val` voltios, como se ilustra en la figura:

![Señal PRBS aplicada en lazo abierto a la plataforma DCMotor](../../assets/prbs_open.png)

Los parámetros de esta función son:

# Argumentos
- `sys::MotorSystem`: objeto que representa la plataforma.

# Argumentos de palabra clave
- `low_val::Real=2.0`: nivel bajo de la señal PRBS, en voltios.
- `high_val::Real=4.0`: nivel alto de la señal PRBS, en voltios.
- `divider::Int=2`: número de periodos de muestreo durante los cuales se
  mantiene cada bit de la secuencia PRBS (a mayor valor en `divider`, se aplica una señal más
  "lenta" y de mayor duración total). 
  
  Note que el tamaño de la secuencia PRBS es fijo
   (tiene 1023 valores ya grabados en el firware de la plataforma) y cada uno de esos
   valores se repiten un número `divider` de veces.

# Retorna
- `(t, u, y)`: tupla de vectores `Vector{Float64}` con el tiempo (s), el
  voltaje aplicado (V) y la velocidad angular medida (°/s).

# Notas

- Esta función se usa para identificar el modelo dinámico de la velocidad
  angular del motor DC por medio de  ([`get_model_prbs`](@ref)). El usuario también puede
  disponer de los datos para usar otros algoritmos de identificación de sistemas como los
  del paquete [ControlSystemIdentification.jl](https://github.com/baggepinnen/ControlSystemIdentification.jl).
- Los datos del experimento se guardan en un archivo CSV en
  `datafiles/DCmotor_prbs_open_exp.csv`.

# Ejemplo
Primero, asegúrese de haber importado el paquete DCMotor y de haber
definido el sistema, así:

```julia
using DCMotor
sys = MotorSystem();
```

Luego, obtenga la respuesta a la señal PRBS en lazo abierto así:

```julia
t, u, y = prbs_open(sys; low_val=1.5, high_val=4.5, divider=2);
```
"""
function prbs_open(sys::MotorSystem;
                   low_val::Real = 2.0, high_val::Real = 4.0,
                   divider::Int = 2)
    h  = SAMPLING_TIME
    bs = BUFFER_SIZE
    total  = PRBS_LENGTH * divider
    frames = ceil(Int, total / bs)

    if !(-5 <= low_val <=5)
        error("low_val debe estar entre -5 y 5")
    end

    if !(-5 <= high_val <=5)
        error("high_val debe estar entre -5 y 5")
    end

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
    leg_size = 9
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
    title=["Experimento actual" "Curva estática – DCMotor"],
    xlabel=["Tiempo (s)" "Voltaje (V)"],
    ylabel=["Velocidad (°/s)" "Velocidad estacionaria (°/s)"], 
    xlims=[(0, t1) (0,5)],
    ylims=[(-30, 830) (-30, 830)],
    background_color_subplot=[:mintcream :ivory ],
    legend=[:topleft :topleft], 
    grid=true, gridalpha=0.2,
    xticks=0:1:10, yticks=0:100:800,
    margin=5Plots.mm)

    if !isempty(yee)
         ulast, ylast = uee[end], yee[end]
         label_2 = "ultimo punto: " * latexstring(@sprintf("u = %0.2f\\,\\, V \\to  \\omega_{ee} = %.2f\\,^o/s", ulast, ylast))
         scatter!(subplot=2, uee, yee, color=:green, marker=:circle, label="",
               markersize=3, linewidth=1)
         scatter!(subplot=2, [ulast], [ylast], color=:orange, marker=:square,
          label= label_2, 
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
        
        if !isempty(yee)    
            scatter!([ulast], [ylast], color=:orange, marker=:square,
            label= label_2, 
            legendfontsize = leg_size, fg_legend = "#d4aa00", markersize=3, linewidth=1)   
        end

        label_1 = latexstring(@sprintf("u = %0.2f\\,\\,V", u1))
        pl_ins = plot(tv, yv, color="#00aad4", label="",
                legendfontsize = leg_size, fg_legend= "#5fbcd3", linewidth=1.5)  
        annotate!(pl_ins, 0.5, 750, text(label_1, 10, :green))
                
                 
            
        plt = plot(pl_ins, pl_ee, layout=(1, 2), size=(900, 500),
        title=["Experimento actual" "Curva estática – DCMotor"],
        xlabel=["Tiempo (s)" "Voltaje (V)"],
        ylabel=["Velocidad (°/s)" "Velocidad estacionaria (°/s)"], 
        xlims=[(0, t1) (0,5)],
        ylims=[(-30, 830) (-30, 830)],
        background_color_subplot=[:mintcream :ivory ],
        legend=[:topleft :topleft],  
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

Realiza una secuencia de experimentos de respuesta al escalón en lazo abierto para
obtener la curva de ganancia estática (curva voltaje vs. velocidad angular
estacionaria) de la plataforma DCMotor.

Esta función aplica sucesivamente un número, definido por `points`, de niveles de voltaje (5 puntos
finos dentro de la zona muerta, entre 0.15 y 0.25 V, y el resto equiespaciados
entre 0.3 y 5 V). Para cada nivel, una vez que la salida
(velocidad angular) alcanza su valor estacionario, se registra el promedio
de las últimas 50 muestras del escalón aplicaso, confome se muestra en la figura siguiente: 

![Curva estática voltaje-velocidad del motor DC](../../assets/get_static_model.png)

A continuación se describen los parámetros mostrados en la figura:

# Argumentos
- `sys::MotorSystem`: objeto que representa la plataforma.

# Argumentos de palabra clave
- `points::Int=20`: número total de niveles de voltaje aplicados.

# Retorna
- `(uee, yee)`: tupla de vectores `Vector{Float64}` con los niveles de voltaje
  aplicado (V) y las velocidades estacionarias medidas (°/s) en cada nivel.

# Notas
- Al finalizar la secuencia se muestra un modelo lineal que relaciona la velocidad
angular en estado estacionario con la tensión de entrada:
 
``\\omega_{ee} = K\\,u + b``
 
Este modelo se  ajusta por mínimos cuadrados con los puntos fuera de la zona muerta. También se estima el voltaje de zona muerta
  `zm`. Se muestra una gráfica con los datos, la recta ajustada y la zona
  muerta.

- Los resultados se guardan en `datafiles/DCmotor_static_gain_response.csv`
  y `datafiles/DCmotor_static_pars.csv` (usados luego por
  [`speed_from_volts`](@ref) y [`volts_from_speed`](@ref)). 

# Ejemplo
Primero, asegúrese de haber importado el paquete DCMotor y de haber
definido el sistema, así:

```julia
using DCMotor
sys = MotorSystem();
```

Luego, obtenga el modelo de ganancia estática así:

```julia
uee, yee = get_static_model(sys);
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
    println("Modelo estático completo")
    return uee, yee
end


# ═══════════════════════════════════════════════════════════════════════════════
#  get_model_step  (modelo de primer orden desde step)
# ═══════════════════════════════════════════════════════════════════════════════

"""
    get_model_step(sys::MotorSystem; yop=400, sigma=100, usefile=false)

Estima, mediante la respuesta al escalón,  un modelo dinámico de primer orden con retardo
 **FOTD** (del inglés, *First Order plus Time Delay*), el cual relaciona la velocidad ángular
del motor y el voltaje de entrada. El modelo FOTD está representado por la siguiente función de transferencia:

``G(s) = \\dfrac{\\alpha}{\\tau s + 1} e^{-Ls}``

donde ``\\alpha`` es la ganancia del sistema, ``\\tau`` es la constante de
tiempo y ``L`` es el retardo.


Para estimar este modelo,  se aplica un escalón de voltaje cuyos valores se ajustan automaticamente para
que el punto de operación en velocidad angular especificado, `yop` (en °/s), quede aproximadamente centrado, como se muestra
en la siguiente figura: 

![Modelo de primer orden con retardo desde la respuesta al escalón](../../assets/get_model_step.png)

Los parámetros de esta función son los siguientes:

# Argumentos
- `sys::MotorSystem`: objeto que representa la plataforma.

# Argumentos de palabra clave
- `yop::Real=400`: punto de operación (velocidad angular, en °/s) alrededor del cual
  se obtiene el modelo FOTD. La respuesta al escalón queda aproximadamente centrada alrededor de `yop`.
- `sigma::Real=100`: desviación máxima (y minima) estimada de la salida  en relación al punto de operación `yop`. 
   La función asigna automáticamente (mediante de la función [`volts_from_speed`](@ref))
   los valores inicial y final del escalón de entrada para que la salida se desvie aproximadamente `sigma` del punto de operación `yop`.    
- `usefile::Bool=false`: si es `true`, se usan los datos del último experimento,
  guardados en `datafiles/DCmotor_step_open_exp.csv` en lugar de ejecutar uno
  nuevo.

# Retorna
- `G::ControlSystemsBase.TransferFunction`: modelo continuo de primer orden
  `b/(s+a)` para la velocidad angular, equivalente a `α/(τs+1)` con
  `b = α/τ` y `a = 1/τ`. Retorna así por compatibilidad con las otras funciones.  
- `L::Float64`: retardo estimado, en segundos.

# Notas

- El retardo `L` y la constante de tiempo `τ` se ajustan por mínimos
  cuadrados a partir de los tiempos de cruce de respuesta en
  los niveles 10 %, 15 %, 20 %, 30 % y 50 % del valor de estado estacionario. La ganancia estática
  `α = Δy/Δu` se calcula directamente de los valores estacionarios antes y
  después del escalón.
- Se muestra una gráfica comparando la salida experimental con el modelo
  simulado, y se guardan los parámetros en `datafiles/DCmotor_fo_model.csv`.

# Ejemplo
Primero, asegúrese de haber importado el paquete DCMotor y de haber
definido el sistema, así:

```julia
using DCMotor
sys = MotorSystem();
```

Es recomendable, al usar por primera vez la plataforma en una sesión, 
obtener el modelo estático para que haya una buena calibración del punto de operación, así:

```julia
G, L = get_static_model(sys);
```

Luego, obtenga el modelo FOTD a partir de la respuesta al escalón así:



```julia
G, L = get_model_step(sys; yop=360);
```
"""
function get_model_step(sys::MotorSystem;
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
    #modelstr1 = latexstring(@sprintf("G(s) = \\frac{%.4f}{s + %.3f} \\quad (FIT=%.1f\\,\\%%)", b, a, r1))

    model_str = latexstring(@sprintf("G(s) = \\frac{%.2f}{s + %.2f} e^{-%0.2f\\,s}", alpha, tau, L_val))

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

Estima los parámetros de un modelo dinámico lineal de primer orden de la forma

``\\qquad \\qquad G(s) =  \\dfrac{b}{s+a}``, 

el cual relaciona la velocidad angular del motor con la entrada de voltaje aplicada.  Para
obtener este modelo se realiza un experimento en lazo abierto, aplicando como entrada una onda binaria pseudoaleatoria
(PRBS) cuyos valores se ajusta automáticamente para que el punto de operación especificado,  `yop` (en °/s), quede aproximadamente
centrado, tal como se ilustra en la figura siguiente:


![Identificación con PRBS de la plataforma DCMotor](../../assets/get_model_prbs.png)

Los parámetros de esta función son los siguientes:

# Argumentos
- `sys::MotorSystem`: objeto que representa la plataforma.

# Argumentos de palabra clave
- `yop::Real=400`: punto de operación (velocidad, en °/s) alrededor del cual
  se obtiene el modelo lineal.
- `sigma::Real=100`: desviación máxima (y minima) estimada (en °/s) de la velocidad
   angular en relación al punto de operación `yop`.  La función asigna automáticamente (mediante de la función [`volts_from_speed`](@ref))
   los valores  mínimo y y máximo de la señal PRBS que producen una desviación `sigma` desde el punto `yop`.  
  
- `usefile::Bool=false`: si es `true`, usa los datos del último experimento
  guardado en `datafiles/DCmotor_prbs_open_exp.csv` para estimar los parámetros
  del modelo de primer orden ``G(s)=b/(s+a)``.



# Retorna
- `G1::ControlSystemsBase.TransferFunction`: modelo de primer
  orden de la velocidad angular, identificado a partir de los datos PRBS.
- `L::Float64`: retardo, fijado en un periodo de muestreo (0.02s).

# Notas
- Para obtener la identificación del modelo,  se filtran los datos  y se ajusta un modelo
  ARX discreto de orden (1,1) con retardo de una muestra, usando
  el estimador de
  [ControlSystemIdentification.jl](https://github.com/baggepinnen/ControlSystemIdentification.jl).
  El modelo discreto resultante se convierte a tiempo continuo con la función `c2d`.
- Al finalizar, se muestra una gráfica que compara la salida experimental con la simulada por el
  modelo (indicando el porcentaje de ajuste, o *FIT*), y se guardan los
  parámetros en `datafiles/DCmotor_fo_model.csv`.

# Ejemplo
Primero, asegúrese de haber importado el paquete DCMotor y de haber
definido el sistema, así:

```julia
using DCMotor
sys = MotorSystem();
```

Es recomendable, al usar por primera vez la plataforma en una sesión, 
obtener el modelo estático para que haya una buena calibración del punto de operación, así:

```julia
G, L = get_static_model(sys);
```

Luego, estime los parámetros del modelo de primer orden a partir de un experimento
PRBS, así:

```julia
G, L = get_model_prbs(sys; yop=350, sigma=60);
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
        t, u, y = prbs_open(sys; low_val=ua, high_val=ub, divider=2)
    else 
        t, u, y = read_csv_file3(_datafile("DCmotor_prbs_open_exp.csv"))
    end

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
    r1 = modelfit(data.y', ysim)
   


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



