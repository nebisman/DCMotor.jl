import Pkg
Pkg.activate(joinpath(@__DIR__, ".."))
using DCMotor
include("ControlUN.jl")



# definicion del sistema
sys = MotorSystem();
s=tf("s")

# identificacion del sistema
G_ang, L = transfer_function(sys) 
#funcion del ángulo


# parametros del sistema
b = numvec(G_ang)[1][1]
a = denvec(G_ang)[1][2]


A = [- a 0; 1 0]
B = [b; 0]
C = [0 1]
D = [0]

Aest = [A [0, 0]; -C 0]
Best = [B; 0]

polos = 1* [-1+1im, -1-1im, -3]; 
Kest = place(Aest, Best, polos)
print("Kest: ", Kest)

Kd = Kest[1]
Kp = Kest[2]
Ki = -Kest[3]


sys_zpk = tf(zpk([], polos, 1))
den_T = denvec(sys_zpk)[1]
T = tf(den_T[4], den_T)


set_pid(sys;  kp=Kp, ki=Ki, kd=Kd, beta=0, output=:angle, deadzone=0.45)

result = step_closed(sys; r0 = 0, r1 = 100,  t0 = 1, t1 =10);stepinfo_exp(result;T)


# ahora con algortimo LQR
Q = diagm([.00100, 1.0, 100])
R = diagm([100])
K_lqr = lqr(Aest, Best, Q, R)

print("K_lqr: ", K_lqr)



Kd = K_lqr[1]
Kp = K_lqr[2]
Ki = -K_lqr[3]

polos = eigvals(Aest - Best*K_lqr)
sys_zpk = tf(zpk([], polos, 1))
den_T = denvec(sys_zpk)[1]
T = tf(den_T[4], den_T)



set_pid(sys;  kp=Kp, ki=Ki, kd=Kd, beta=0, output=:angle, deadzone=0.2)
result = step_closed(sys; r0 = 0, r1 = 100,  t0 = 1, t1 =2); stepinfo_exp(result;T)

