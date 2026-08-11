


using DCMotor
include("ControlUN.jl")


# definicion del sistema
sys = MotorSystem(port="/dev/ttyUSB0", bauds=460800);

s=tf("s")

# identificacion del sistema
G = get_model_prbs(sys; yop=360)

# parametros del sistema
b = numvec(G)[1][1]
a = denvec(G)[1][2]


# iteraciones
ωn=15
ζ = 0.7
T  = ωn^2/(s^2 + 2*ζ*ωn*s +ωn^2)

# calculo de las constantes
Kp = (2*ζ*ωn-a)/b
Ki = ωn^2/b


# exoerimentos
set_pid(sys;  kp=Kp, ki=Ki, kd=0, beta=0, output=:speed, deadzone=0)
result = step_closed(sys; r0 = 0, r1 = 360,  t0 = 1, t1 =2); 





