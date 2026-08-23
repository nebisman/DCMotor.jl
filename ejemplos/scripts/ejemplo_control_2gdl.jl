
# Ejemplo: controlador de dos grados de libertad (2-GDL)
# Supongamos que queremos diseñar e implementar un controlador de dos
# grados de libertad por métodos algebraicos.
# Para ello usamos el siguiente código:


## definiciones
using DCMotor
sys = MotorSystem();
G_ang = tf(sys,:angle)


## Vamos a implementar la siguiente funcion itae
ω₀ =20
s=tf("s")
T = ω₀^3/(s^3 + 1.75*s^2*ω₀ + 2.15*s*ω₀^2 + ω₀^3)
C = cont2dof(G_ang, T, 2)

# respuesta del controlador
set_controller(sys, C; output=:speed, deadzone=0.2)
result = step_closed(sys; r0 = 0, r1 = 400,  t0 = 0.5, t1 =2); 
stepinfo(result, T)
 




## Ahora vamos a implementar una función cuadrática óptima
Tlq, Gur = lqmodel(G_ang, .0015)
C = cont2dof(G_ang, Tlq, 2 , [-50,-55])
set_controller(sys, C; output=:angle, deadzone=0.2)
result = step_closed(sys; r0 = 0, r1 = 100,  t0 = 0.5, t1 =2); 
stepinfo(result, Tlq)   