using ControlSystems, ControlSystemsBase
using LinearAlgebra
using Polynomials
using InvertedIndices
using Plots; gr(linewidth = 2, grid=:true)
using Printf
Base.show(io::IO, f::Float64) = @printf(io, "%.4f", f)
using Symbolics


function convol(h,u)
    n = length(h);
    m = length(u);
    l = n + m-1;
    w = zeros(l);
    for i in 1:l
        for j in max(1,i+1-m):min(i,n)
            w[i] = w[i] + h[j]*u[i-j+1];
        end
    end
    w;
end

#
function lq1(P,q)
    n, d = tfdata(P);
    Qd = convol(d, cambia_signo(d));
    Qn = q * convol(n, cambia_signo(n));
    Qa = zeros(length(Qd));
    l = length(Qd) - length(Qn);
    Qa[l+1:end] = Qn;
    Q = Qd .+ Qa;
    Q_poly  = Polynomial(reverse(Q));
    r = roots(Q_poly);
    inestables = real.(r).> 0;
    indices = findall(iszero,inestables);
    dT = fromroots(r[indices]);
    dT0 = dT(0);
    DT = reverse(dT.coeffs);
    T = tf(n,DT) / dcgain(tf(n,DT));
    Gur = minreal(T / P) ;
    T, Gur
end
     

function tfdata(G)
    G = tf(G)
    n = num(G)[1,1]
    d = den(G)[1,1]
    n, d
end

function cambia_signo(n)
    n1 = n
    long = length(n1) - 1
    n2 = ones(long+1)
     for j in long:-2:1
       n2[j] = -1
    end
    return n2 .*n
end
   
function poly_coeffs(eq, x, deg; max_power=20)
   @variables s z 
   ns = 1:deg
   eq = expand(eq)
   eq0 = substitute(eq, Dict(x => 0))
   eq = eq - eq0
   eq = substitute(expand(eq), Dict(x^j => 0 for j=last(ns)+1:max_power))
   eqs = [eq0]
   for i in ns
       powers = Dict(x^j => (i==j ? 1 : 0) for j=1:last(ns))
       push!(eqs, substitute(eq, powers))
   end
   reverse(eqs)
end

function tustin_c2d(F, Ts)
   @variables s z 
   num, den = tfdata(F)
   n = length(den) - 1
   num_c = crea_poly(num,s)
   den_c = crea_poly(den,s)
   P = num_c / den_c
   F_d = simplify(substitute(P, (Dict(s => (2/Ts)*(z-1)/(z + 1)))))
   num_f, den_f = Symbolics.arguments(Symbolics.value(simplify(F_d)))
   num_d = poly_coeffs(num_f, z, n)
   den_d = poly_coeffs(den_f, z, n)
   F_d = tf(num_d, den_d, Ts)
   F_d
end

function crea_poly(coef,var)
   @variables s z 
   n = length(coef)
   @variables F
   F = 0
   for k = 1:n 
       F = F + coef[k] * s^(n-k)
   end
   F
end



function step_plt(sys, t_max)
   #=
   Grafica la respuesta al escalón del syistema sys
   =#
   rey = step(sys,t_max)
   si = stepinfo(rey)
   plot(si)
end 

#=
Evalúa el máximo de la respuesta al escalón de sys
=#
function max_resp_step(sys)
   reu = step(sys)
   v_max = maximum(abs.(reu.y))
   v_max
end   

#=
Grafica respuesta al escalón de salida y de control
=#
function yu_stp_plt(T, Gur, tmax)
   re_y = step(T, tmax)
   sy = stepinfo(re_y)
   re_u = step(Gur, tmax)
   su = stepinfo(re_u)
   plot(plot(sy), plot(su))
end
