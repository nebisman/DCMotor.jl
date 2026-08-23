# ═══════════════════════════════════════════════════════════════════════════════
#  ejemplo_identificacion.jl – Identificación del sistema DCMotor
#
#  Muestra cómo:
#    1. Obtener la curva estática (velocidad vs voltaje)
#    2. Obtener un modelo FOTD desde un escalón
#    3. Obtener un modelo de primer orden desde PRBS
#    4. Obtener la función de transferencia nominal
# ═══════════════════════════════════════════════════════════════════════════════

import Pkg
Pkg.activate(joinpath(@__DIR__, ".."))

using DCMotor
using ControlSystems
using Plots

# ── 1. Crear el sistema ─────────────────────────────────────────────
sys = MotorSystem();
using Revise

# ── 2. Curva estática ───────────────────────────────────────────────
## Barre todos los voltajes y mide la velocidad estacionaria.
uee, yee = get_static_model(sys);
disconnect!(sys)
println("Curva estática: $(length(uee)) puntos")

# ── 3. Respuesta al escalón en lazo abierto ─────────────────────────
t, u, y = step_open(sys; u0=-0.1, u1=5, t0=1.0, t1=1.0)
println("Step open: $(length(t)) muestras")
using Plots

t, u, y = prbs_open(sys; low_val=2.0, high_val=4.0, divider=4)
using Plots
savefig("/home/leonardo/datos/share_desktop/proyecto_julia/DCMotor.jl/resources/graficos/entrada_prbs.svg")

plot(t,y, title="salida", label="y")
savefig("/home/leonardo/datos/share_desktop/proyecto_julia/DCMotor.jl/resources/graficos/salida_prbs.svg")

# ── 4. Modelo FOTD desde escalón ────────────────────────────────────
# Estima alpha, tau, L del modelo G(s) = alpha/(tau·s + 1) · exp(-L·s)
G = get_model_step(sys; yop=360)
println("Modelo FOTD: α=$(round(alpha,digits=3)), τ=$(round(tau,digits=3)), L=$(round(L,digits=3))")

# ── 5. Modelo de primer orden desde PRBS ────────────────────────────
G1 = get_model_prbs(sys; yop=360)
println("Modelo PRBS: $G1")

p