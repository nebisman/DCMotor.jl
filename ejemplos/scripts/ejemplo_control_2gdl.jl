
## Ejemplo 1: controlador de dos grados de libertad (2-GDL)
# Supongamos que queremos diseñar e implementar un controlador de dos
# grados de libertad port métodos algebraicos
# Para ello usamos el siguiente código:

## definiciones
using DCMotor
sys = MotorSystem();
s = tf("s");
G_ang, L = transfer_function(sys)

##  Vamos a implementar la siguiente funcion itae
ω₀ = 18
T = ω₀^3/(s^3 + 1.75*s^2*ω₀ + 2.15*s*ω₀^2 + ω₀^3)
C = dise_2gdl(G_ang, T, 2 , [-80])

# respuesta del controlador
result = step_closed(sys; r0 = 0, r1 = 60,  t0 = 0.5, t1 =2); 
stepinfo_exp(result;T)   
