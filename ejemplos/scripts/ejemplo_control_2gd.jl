
## Ejemplo 1: controlador de  un grado de libertad (1-GDL)
# Supongamos que queremos implementar un controlador definido por la función de transferencia:
# ``C(s)=\\begin{bmatrix} \\frac{0.0374s + 2.6207}{s + 69.5428} & -\\frac{0.0279s + 2.6207}{s + 69.5428} \\end{bmatrix}``
#
# Para ello usamos el siguiente código:

using DCMotor, ControlSystems
sys = MotorSystem();
s = tf("s");
C1 = (0.0374s + 2.6207)/(s + 69.5428)
C2 = -(0.0279s + 2.6207)/(s + 69.5428)
C = [C1 C2]
set_controller(sys, C; output=:angle, deadzone=0.2);

# respuesta del controlador
G, L = transfer_function(sys, :angle)
T = C1*feedback(G, -C2)
result = step_closed(sys; r0 = 0, r1 = 100,  t0 = 0.5, t1 =2); 
stepinfo_exp(result;T)   
