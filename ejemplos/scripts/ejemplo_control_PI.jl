
using DCMotor



## definicion del sistema y parametros
sys = MotorSystem();
G = tf(sys, :speed)
a = denvec(G)[1][2]
b = numvec(G)[1][1]

## Diseño por localización de polos
# Frecuencia natural y factos de amortiguamiento
ωn=15
ζ = 0.7
# calculo de las constantes
Kp = (2*ζ*ωn-a)/b
Ki = ωn^2/b

s = tf("s")
T= ωn^2/(s^2 + 2*ζ*ωn*s + ωn^2)
set_pid(sys;  kp=Kp, ki=Ki, kd=0, beta=0, output=:speed, Tf=0.005)
result = step_closed(sys; r0 = 00, r1 =400,  t0 = 1.5, t1 =2);
stepinfo(result,T)

## Ahora diseñamos un PI para velocidad con Loopshaping, 


ωgc = 15
# Note que incluimos el retardo del muestreo digital, por lo cual se requiere 
# el paquete completo ControlSystems
using ControlSystems
Gd = G*delay(0.02)
C, Kp, Ki, fig, CF = loopshapingPI(Gd, ωgc; rl=1,  phasemargin=45, form=:parallel)

T1 = feedback(C*G, 1)

# y lo probamos
set_pid(sys;  kp=Kp, ki=Ki, kd=0, beta=1, output=:speed, deadzone=0)
result = step_closed(sys; r0 = 0, r1 = 400,  t0 = .5, t1 =1.5); 
stepinfo(result,T1)

