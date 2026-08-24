# ═══════════════════════════════════════════════════════════════════════════════
#  ejemplo_identificacion.jl – Identificación del sistema DCMotor
#
# ═══════════════════════════════════════════════════════════════════════════════




# 1. importamos el paquete DCMotor y creamos el sistema
using DCMotor
sys = MotorSystem();


## 2. Ahora obtenemos el modelo estático
uee, yee = get_static_model(sys);

## 3. Podemos obtener ña respuesta al escalón con: 
t, u, y = step_open(sys; u0=1, u1=3.5, t0=1.0, t1=1.0)

## 4. Podemos hacer una prueba con una señal de entrada PRBS
t, u, y = prbs_open(sys; low_val=2.0, high_val=3.5, divider=2)

## 5. Podemos estimar los parámetros de la planta por respuesta al escalón:
G, L = get_model_step(sys; yop=300)

##  6. Podemos estimar los parámetros de la planta por respuesta a una señal PRBS
G, L = get_model_prbs(sys; yop=300, usefile=true)