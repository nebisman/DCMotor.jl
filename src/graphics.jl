# ═══════════════════════════════════════════════════════════════════════════════
#  Utilidades de plotting en tiempo real
# ═══════════════════════════════════════════════════════════════════════════════

#using Plots

"""
    setup_fig(nrows; figsize=(900,500))

Crea una figura con `nrows` subplots verticales usando Plots.jl.
Retorna el objeto plot y un vector de índices de subplot.
En Plots.jl, actualizamos los datos directamente y usamos `display()`.
"""
function setup_fig(nrows::Int; figsize=(900, 500))
    # Se usan layouts de Plots.jl
    plt = plot(layout=(nrows, 1), size=figsize, legend=:bottomright,
               background_color=:white, margin=5Plots.mm)
    display(plt)
    return plt
end


#= _ijulia() = isdefined(Main, :IJulia) ? getfield(Main, :IJulia) : nothing


"""Redibuja la figura para actualización en tiempo real."""
function redraw!(plt)
    if !isnothing(_ijulia())
        ij = getfield(Main, :IJulia)
        Base.invokelatest(getfield(ij, :clear_output), true)  # borra la salida anterior
    end
    display(plt)
    return nothing
end
 =#

# ═══════════════════════════════════════════════════════════════════════════════
#  pantalla.jl – Visualización en tiempo real unificada: REPL, Jupyter y Pluto.
#
#  Requiere que UNDCMotor declare dos dependencias livianas (una sola vez):
#
#      pkg> activate <ruta de UNDCMotor>
#      pkg> add HypertextLiteral AbstractPlutoDingetjes
#
#  y en UNDCMotor.jl:
#
#      include("motorsys.jl")      # (borrar allí redraw! y _ijulia)
#      include("pantalla.jl")
#      export redraw!, pantalla, entorno
#
#  Con eso, el notebook de Pluto solo necesita `using UNDCMotor`.
#
#  Los experimentos NO cambian: `on_frame` sigue armando la figura y llamando
#  `redraw!(plt)`; lo único que varía es a dónde va a parar la figura:
#
#      REPL     -> display(plt)                    (ventana GR, se redibuja sola)
#      Jupyter  -> clear_output(true) + display    (misma salida de la celda)
#      Pluto    -> put! a un canal interno; la celda `pantalla()` lo consume
#
#      # celda 1
#      using UNDCMotor
#
#      # celda 2 — la pantalla (una sola vez en todo el notebook)
#      pantalla()
#
#      # celda 3 — cualquier experimento, sin cambios
#      t, r, y, u = step_closed(sys, 100.0, 300.0, 1.0, 2.0)
#
#  LB 2026 – MIT License
# ═══════════════════════════════════════════════════════════════════════════════

using Base64
using HypertextLiteral: @htl
import AbstractPlutoDingetjes

# ── Detección del entorno ────────────────────────────────────────────────────

_ijulia() = isdefined(Main, :IJulia) ? getfield(Main, :IJulia) : nothing

"""
    entorno() -> :pluto | :jupyter | :script

Identifica dónde se está ejecutando para decidir cómo mostrar cada figura.
"""
entorno() = isdefined(Main, :PlutoRunner) ? :pluto :
            (_ijulia() !== nothing       ? :jupyter : :script)

# ── Canal de figuras (solo lo usa Pluto) ─────────────────────────────────────
#
#  Capacidad 1 con política "se conserva el más reciente": si el experimento
#  produce figuras más rápido de lo que el navegador las pide, se descartan las
#  intermedias en vez de encolar atraso. 

const _FIGURAS = Channel{Any}(1)

"Deposita la figura descartando la anterior si nadie la había recogido."
function _publicar_figura(plt)
    while isready(_FIGURAS)          # vaciar sin bloquear
        take!(_FIGURAS)
    end
    put!(_FIGURAS, plt)
    return nothing
end

# ── redraw!: el único punto de contacto con los experimentos ─────────────────

"""
    redraw!(plt)

Muestra la figura en el destino que corresponda al entorno. Es la misma llamada
que ya usan todos los `on_frame`; los experimentos no necesitan cambios.
"""
function redraw!(plt)
    ent = entorno()
    if ent === :pluto
        _publicar_figura(plt)
    elseif ent === :jupyter
        flush(stdout)                                        # texto pendiente
        ij = _ijulia()
        Base.invokelatest(getfield(ij, :clear_output), true) # borra la salida anterior
        display(plt)
    else
        display(plt)
    end
    return nothing
end

# ── pantalla(): el monitor para Pluto ────────────────────────────────────────

"Serializa una figura como data URL PNG para el atributo src de un <img>."
function _png_data_url(fig)
    io = IOBuffer()
    show(io, MIME"image/png"(), fig)
    return "data:image/png;base64," * base64encode(take!(io))
end

"¿Expone esta versión de AbstractPlutoDingetjes el puente JS <-> Julia?"
_hay_enlace_js() = isdefined(AbstractPlutoDingetjes, :Display) &&
                   isdefined(AbstractPlutoDingetjes.Display, :with_js_link)

"""
    pantalla(; alto = nothing)

Widget de monitoreo para Pluto: una celda que muestra cada figura que los
experimentos publican con `redraw!`. Créela UNA vez, en su propia celda, y
corra los experimentos en las celdas que quiera; la pantalla se actualiza sola.

Fuera de Pluto no hace falta: `redraw!` muestra directo en la ventana GR o en
la celda de Jupyter.
"""
function screen_pluto(; alto = nothing)
    entorno() === :pluto || return nothing   # fuera de Pluto la pantalla es implícita

    _hay_enlace_js() || error("""
        Esta versión de AbstractPlutoDingetjes no expone Display.with_js_link.
        Actualícela en el entorno del paquete:  pkg> up AbstractPlutoDingetjes""")

    enlace = AbstractPlutoDingetjes.Display.with_js_link() do _
        plt = take!(_FIGURAS)          # bloquea hasta que un experimento publique
        return _png_data_url(plt)
    end

    estilo = alto === nothing ? "max-width:100%; display:block" :
                                "max-width:100%; height:$(alto)px; display:block"

    return @htl("""
    <div style="font-family: sans-serif">
      <img style=$(estilo)>
      <span style="color:#666; font-size:0.85em">pantalla lista — esperando experimento…</span>
    </div>
    <script>
      const cont  = currentScript.parentElement
      const img   = cont.querySelector("img")
      const lbl   = cont.querySelector("span")
      const pedir = $(enlace)

      let vivo = true
      invalidation.then(() => { vivo = false })

      let k = 0
      try {
        while (vivo) {
          const src = await pedir("siguiente")   // espera sin reloj ni sondeo
          if (!vivo) break
          k += 1
          img.src = src
          lbl.textContent = "trama " + k
        }
      } catch (err) {
        lbl.textContent = "enlace cerrado: " + err
        console.error(err)
      }
    </script>
    """)
end