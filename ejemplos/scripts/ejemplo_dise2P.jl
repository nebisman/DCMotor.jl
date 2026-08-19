
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



C2 = dise2p(G_ang, T, 1 ,[-70])


using Alert
G_ang = G/s
C, T, Gur, S, ind_error=asigne_polos(G_ang, [-10+10im,-10-10im,-30.0])

s = tf("s");
C = (0.1071s + 2.2463)/(1.0000s + 35.5428);    
set_controller(sys, C; output=:angle, deadzone=0.2)

set_controller(sys, C; output=:angle, deadzone=0.2);

Ca = (0.0374s + 2.6207)/(s + 69.5428)
Cb = -(0.0279s + 2.6207)/(s + 69.5428)
C2 = [Ca Cb]
set_controller(sys, C2; output=:angle, deadzone=0.2);
# respuesta del controlador
result = step_closed(sys; r0 = 0, r1 = 100,  t0 = 0.5, t1 =2); 
stepinfo_exp(result;T)
profile_closed(sys; timevalues =[0,1,2,3,4], refvalues=[0,90,180,90, 0])
