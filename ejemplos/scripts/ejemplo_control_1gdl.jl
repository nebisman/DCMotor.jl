
## Ejemplo 1: controlador de  un grado de libertad (1-GDL)
# Supongamos que queremos diseñar e implementar un controlador de 1-GDL
# usando un método algebraico
# Para ello usamos el siguiente código:

using DCMotor

## definiciones
sys = MotorSystem();
s = tf("s");
G, L = transfer_function(sys,:angle)

## diseño y carga del controlador
polos_lc = [-10+10im, -10-10im, -60]
C = dise_1gdl(G, polos_lc) 
set_controller(sys, C; output=:angle, deadzone=0.2)

# Aqui vemos la respuesta al escalon en lazo cerrado
T = feedback(C*G, 1)
result = step_closed(sys; r0 = 0, r1 = 90,  t0 = 0.5, t1 =2); 
stepinfo_exp(result;T)
