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


# ── 2. Curva estática ───────────────────────────────────────────────
## Barre todos los voltajes y mide la velocidad estacionaria.
uee, yee = get_static_model(sys; points=15);
disconnect!(sys)
println("Curva estática: $(length(uee)) puntos")

# ── 3. Respuesta al escalón en lazo abierto ─────────────────────────
t, u, y = step_open(sys; u0=0.5, u1=3.5, t0=1.0, t1=1.0)
println("Step open: $(length(t)) muestras")

# ── 4. Modelo FOTD desde escalón ────────────────────────────────────
# Estima alpha, tau, L del modelo G(s) = alpha/(tau·s + 1) · exp(-L·s)
G = get_fomodel_step(sys; yop=300)
println("Modelo FOTD: α=$(round(alpha,digits=3)), τ=$(round(tau,digits=3)), L=$(round(L,digits=3))")

# ── 5. Modelo de primer orden desde PRBS ────────────────────────────
G1 = get_model_prbs(sys; yop=300, usefile=true)
println("Modelo PRBS: $G1")

h = transfer_function(sys, :speed)