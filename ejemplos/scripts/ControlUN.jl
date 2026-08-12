using ControlSystems
using LinearAlgebra
using Polynomials
using InvertedIndices
using Plots; gr(linewidth = 2, grid=:true)
using Printf
Base.show(io::IO, f::Float64) = @printf(io, "%.4f", f)
using Symbolics

"""
    dise2p(G, T, m, PolosObs)

Calcula un controlador de dos parámetros.

# Argumentos
- `G`: Función de transferencia de la planta
- `T`: Función de transferencia de lazo cerrado deseada
- `m::Int`: Orden del controlador
- `PolosObs::Vector`: Polos del observador

# Retorna
- `C`: Función de transferencia 1×2  →  [L(s)/A(s)   -M(s)/A(s)]

La señal de control es:  u = (L/A)·r − (M/A)·y
"""
function dise2p(G, T, m, polosobs)

    # ─── Funciones auxiliares ───────────────────────────────────────────

    """Multiplicación de polinomios en coeficientes descendentes."""
    function polyconv(a::Vector, b::Vector)
        c = zeros(promote_type(eltype(a), eltype(b)), length(a) + length(b) - 1)
        for i in eachindex(a), j in eachindex(b)
            c[i + j - 1] += a[i] * b[j]
        end
        return c
    end

    """Extrae coeficientes descendentes (grado mayor primero) de un Polynomial."""
    function descvec(p)
        return Float64.(reverse(coeffs(p)))
    end
    
    npol = length(polosobs)
    if npol == 0
        Dhbar = [1]
    else
        Dhbar = descvec(fromroots(polosobs))
    end
    
    D = descvec(denpoly(G)[1, 1])
    N_raw = descvec(numpoly(G)[1, 1])
    n = length(D) - 1  
    
    # ─── H = minreal(T / N(s)) ──────────────────────────────────────────

    H = minreal(T * tf([1.0], N_raw))
    nh = descvec(numpoly(H)[1, 1])
    dh = descvec(denpoly(H)[1, 1])
    
    grad_DH = length(dh) - 1   

    gr_Dpbar = n + m - grad_DH 
    
    if npol != gr_Dpbar
        error("DH_bar debe tener $gr_Dpbar polos")
    end


    
 

    # Rellenar N con ceros a la izquierda (como hace tfdata de MATLAB)
    N = [zeros(length(D) - length(N_raw)); N_raw]



    # ─── Prefiltro L y lado derecho F ───────────────────────────────────

    L = polyconv(nh, Dhbar)
    F = polyconv(dh, Dhbar)

    # ─── Matriz de Sylvester  (n+m+1) × (2m+2) ─────────────────────────

    Sm = zeros(n + m + 1, 2m + 2)
    for k in 1:m+1
        Sm[k:k+n, k]       = D
        Sm[k:k+n, k+m+1]   = N
    end

    # ─── Resolución del sistema lineal ──────────────────────────────────

    local A_vec::Vector{Float64}, M_vec::Vector{Float64}

    if m >= n
        # Intenta diseñar controlador que rechaza perturbaciones
        Ind = setdiff(1:2m+2, m + 1)           # todas las columnas excepto m+1
        Sm_red = Sm[:, Ind]
   
        if rank(Sm_red) >= n + m + 1
            x     = Sm_red \ F
            A_vec = [x[1:m]; 0.0]               # acción integral (cero en s=0)
            M_vec = x[m+1:2m+1]
        else
            x     = Sm \ F
            A_vec = x[1:m+1]
            M_vec = x[m+2:2m+2]
        end

    elseif m == n - 1
        x     = Sm \ F
        A_vec = x[1:m+1]
        M_vec = x[m+2:2m+2]

    else
        error("El orden del controlador m=$m debe ser al menos n-1 = $(n-1)")
    end

    # ─── Controlador de dos parámetros ──────────────────────────────────
     C2P = [tf(L,A_vec) tf(-M_vec,A_vec)]
     return C2P
    
end




"""
    asigne_polos(P, pol)

Calcula un controlador 1-DOF de realimentación unitaria que asigna los
polos de lazo cerrado al vector `pol`.
Retorna `(K, T, Gur, S, ind_error)`.
"""
function asigne_polos(P, pol)
    P1     = tf(P)
    N, D   = _tfdata(P1)
    n      = length(D) - 1
    l      = length(D) - length(N)
    nMm    = length(pol)
    m      = nMm - n
    DT_p   = fromroots(pol)
    Na     = zeros(length(D))
    Na[l+1:end] = N
    DT     = reverse(DT_p.coeffs)

    Sm = zeros(n + m + 1, 2*(m+1))
    for k in 1:m+1
        Sm[k:k+n, k]     = D
        Sm[k:k+n, k+m+1] = Na
    end

    ind_error = 0
    local Y::Vector{Float64}, X::Vector{Float64}

    if m > n - 1
        Ind  = setdiff(1:2m+2, m+1)
        Smj  = Sm[:, Ind]
        if rank(Smj) >= n + m + 1
            XY = Smj \ DT
            Y  = [XY[1:m]; 0.0]
            X  = XY[m+1:end]
        else
            XY = Sm \ DT
            Y  = XY[1:m+1]
            X  = XY[m+2:end]
        end
    elseif m == n - 1
        XY = Sm \ DT
        Y  = XY[1:m+1]
        X  = XY[m+2:2m+2]
    else
        ind_error = 1
        @warn "asigne_polos: Número de polos insuficiente"
        return nothing, nothing, nothing, nothing, ind_error
    end

    K   = tf(X, Y)
    T   = feedback(K * P)
    Gur = minreal(T / P)
    S   = minreal(1 - T)
    return K, T, Gur, S, ind_error
end





function asigne_polos(P,pol)
    # CALCULA UN CONTROLADOR CON REALIMENTACI?N UNITARIA
    # Para la planta dada por una funcion de transferencia P
    # Los polos se asignan en el vector pol
    # m: orden del controlador
    P1 = tf(P);
    # Obtiene los datos de la planta
    P1 = tf(P);
    # Obtiene los datos de la planta
    N,D = tfdata(P1);
    # n =  orden de la planta
    n = length(D)-1;
    l = length(D)-length(N);
    # Calcula orden del sistema de lazo cerrado
    nMm = length(pol);
    m = nMm-n;
    # Forma polinomio con los polos
    DT = fromroots(pol);
    Na = zeros(length(D));
    Na[l+1:end]  =  N;
    DT = reverse(DT.coeffs);
    # Forma la Matriz de Sylvester
    Sm = zeros(n+m+1,2*(m+1));
    for k = 1:m+1
      Sm[k:k+n,k] = D;
      Sm[k:k+n,k+m+1] = Na;
   end
   ind_error = 0;
   if m > n - 1;
      # Si es posible, disena un controlador que rechaza perturbaciones 
      #Smj = Sm[:,1:end .! =  m+1]
      Smj = Sm[:,Not(m+1)];
      if rank(Smj)>=n+m+1;
         XY = Smj\DT;
         Y = [XY[1:m];0];
         X = XY[m+1:end] ;
      else
         XY = Sm\DT;
         Y = XY[1:m+1];
         X = XY[m+2:end] ; 
      end
   elseif m==n-1
      XY = Sm\DT;
      Y = XY[1:m+1];
      X = XY[m+2:2*m+2] ;
   else
      ind_error = 1;
      alert("asigne_polos: Número de polos insuficiente")
   end
   K = tf(X,Y);
   T = feedback(K * P);
   Gur = minreal(T/P);
   S = minreal(1 - T);
   K, T, Gur, S, ind_error
end

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
