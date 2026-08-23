
#= 
import Pkg
Pkg.instantiate()
Pkg.activate(joinpath(@__DIR__, ".."))
Pkg.resolve()

 =#
import Pkg
Pkg.activate(joinpath(@__DIR__, ".."))
using DCMotor


#Pkg.add("InvertedIndices")
#Pkg.add("Symbolics")


include("ControlUN.jl")


# definicion del sistema
sys = MotorSystem();
using ControlSystems
s=tf("s")

# identificacion del sistema
G, L = get_model_prbs(sys; yop=360,sigma=100)
#G = get_fomodel_step(sys; yop=360)

# parametros del sistema
using ControlSystems
a = denvec(G)[1][2]
b = numvec(G)[1][1]

# especificaciones transitorias
SP = 0.05
tee = 0.5
tr = 0.2



# Valores iniciales
ζ0 = abs(log(SP))/sqrt(pi^2+(log(SP))^2)
ωn1 = (1.197 - 0.43*ζ0 + 2.553*ζ0^2)/tr
ωn2 = 5/tee
ωn3 = a/(2*ζ0)

ωn0 = maximum([ωn1, ωn2, ωn3])


using ControlSystems
ωn=15
ζ = 0.7
# calculo de las constantes
Kp = (2*ζ*ωn-a)/b
Ki = ωn^2/b

Gd=G#*delay(0.01)
G1  = feedback(Gd,Kp)
T= feedback(G1*Ki/s,1)
set_pid(sys;  kp=Kp, ki=Ki, kd=0, beta=0, output=:speed, deadzone=0)
result = step_closed(sys; r0 = 00, r1 =400,  t0 = 1.5, t1 =2); 
stepinfo(result, T)# %% iteraciones

# calculo de las constantes
set_reference(sys, 5)

ωgc = 15
Gd=G*delay(0.02)
C, Kp, Ki, fig, CF = loopshapingPI(Gd, ωgc; rl=1,  phasemargin=45, form=:parallel)


T1 = feedback(C*Gd, 1)
# exoerimentos
set_pid(sys;  kp=Kp, ki=Ki, kd=0, beta=1, output=:speed, deadzone=0)
result = step_closed(sys; r0 = 0, r1 = 400,  t0 = .5, t1 =1.5); stepinfo_exp(result;T=T1)
#stairs_closed(sys, stairs=[180, 270, 360, 450,540,620] )
