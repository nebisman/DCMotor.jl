import Pkg
Pkg.activate(joinpath(@__DIR__, ".."))
using DCMotor

include("ControlUN.jl")




# definicion del sistema
sys = MotorSystem(port="/dev/ttyUSB0");

s=tf("s")

# identificacion del sistema
G, L = get_model_prbs(sys; yop=400, usefile=true)

#funcion del ángulo
using Alert
G_ang = G/s
C, T, Gur, S, ind_error=asigne_polos(G_ang, [-5+5im,-5-5im,-10.0])

# parametros del sistema
b = numvec(G_ang)[1][1]
a = denvec(G_ang)[1][2]




# especificaciones transitorias
tee = 0.5
tr = 0.2
SP = 0.1

# Valores iniciales
n = 3
ζ0 = abs(log(SP))/sqrt(pi^2+(log(SP))^2)
ωn1 = (1.197 - 0.43*ζ0 + 2.553*ζ0^2)/tr
ωn2 = 5/tee
ωn3 = a/(2*ζ0+n)

ωn0 = maximum([ωn1, ωn2, ωn3])


# Iteraciones



# Calculo de las constantes del PID Segundo orden
ωn = 15
ζ0 = 0.75
n=2

T  = n*ωn^3/((s+n*ωn)*(s^2 + 2*ζ0*ωn*s +ωn^2))
Kd = ((2ζ + n) * ωn - a) / b
Kp = (1 + 2n * ζ) * ωn^2 / b
Ki = n * ωn^3 / b
set_pid(sys;  kp=Kp, ki=Ki, kd=Kd, beta=0, N=5, output=:angle, deadzone=0.1)
result = step_closed(sys; r0 = 0, r1 = 100,  t0 = 1, t1 =3);
stepinfo(result, T)


#%% calculo de las constantes del PID con ITAE

ωn = 14
T = (ωn^3)/(s^3 + 1.75*s^2*ωn + 2.15*s*ωn^2 + ωn^3)
Kd = (1.78 * ωn - a) / b
Kp = 2.15 * ωn^2 / b
Ki = ωn^3/b
 

set_pid(sys;  kp=Kp, ki=Ki, kd=Kd, beta=0,  Tf= 0.01, output=:angle)
result = step_closed(sys; r0 = 0, r1 = 100,  t0 = 1, t1 =2);
stepinfo(result, T)


using Printf

numero = 3.141592653589793
@printf("%.8g\n", Kp)
@printf("%.8g\n", Ki)
@printf("%.8g\n", Kd)

