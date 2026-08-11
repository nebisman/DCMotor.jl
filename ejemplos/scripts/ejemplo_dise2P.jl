
# inclusion del paquete
import Pkg
#Pkg.activate(joinpath(@__DIR__, ".."))

using DCMotor
include("ControlUN.jl")


# definicion del sistema
sys = MotorSystem(port="/dev/ttyUSB0");

#funcion de transferencia
G_ang = transfer_function(sys)


# funcion de angulo

s=tf("s")

# funcion itae
w0 = 10
T = (w0^3+2.15*s*w0^2)/(s^3 + 1.75*s^2*w0 + 2.15*s*w0^2 + w0^3)
ωn=10
ζ = 0.7
T  = ωn^2/(s^2 + 2*ζ*ωn*s +ωn^2)



C2 = dise2p(G_ang, T, 2 ,[-80])
set_controller(sys, C2; output=:angle, deadzone=0.4);


# respuesta del controlador
result = step_closed(sys; r0 = 0, r1 = 100,  t0 = 0.5, t1 =2); stepinfo_exp(result;T)
profile_closed(sys; timevalues =[0,1,2,3,4], refvalues=[0,90,180,90, 0])
