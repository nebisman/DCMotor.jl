
## Ejemplo 1: controlador de  un grado de libertad (1-GDL)
# Supongamos que queremos implementar un controlador definido por la función de transferencia:
#  ``C(s)=\\frac{0.1071s + 2.2463}{s + 35.5428}``
#
# Para ello usamos el siguiente código:

using DCMotor, ControlSystems
sys = MotorSystem();
s = tf("s");
C = (0.1071s + 2.2463)/(s + 35.5428);    
set_controller(sys, C; output=:angle, deadzone=0.2)

# Aqui vemos la repuesta del sistema cerrado al escalon
G, L = transfer_function(sys, :angle)
T = feedback(C*G, 1)
result = step_closed(sys; r0 = 0, r1 = 100,  t0 = 0.5, t1 =2); 
stepinfo_exp(result;T)
