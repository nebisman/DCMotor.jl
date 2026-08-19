# ═══════════════════════════════════════════════════════════════════════════════
#  control_design.jl – Diseño algebraico de controladores por asignación de
#  polos.   LB 2026 – MIT License
# ═══════════════════════════════════════════════════════════════════════════════

using ControlSystemsBase
using LinearAlgebra
using Polynomials

"""
    dise_2gdl(G, T, m, PolosObs)

Calcula un controlador de dos parámetros.

# Argumentos
- `G`: Función de transferencia de la planta
- `T`: Función de transferencia de lazo cerrado deseada
- `m::Int`: Orden del controlador
- `polosobs::Vector`: Polos del observador

# Retorna
- `C`: Función de transferencia 1×2  →  [L(s)/A(s)   -M(s)/A(s)]

La señal de control es:  u = (L/A)·r − (M/A)·y
"""
function dise_2gdl(G, T, m, polosobs)
    
    # -- normalizar las funciones de transferencia

    G = tf(G)
    T = tf(T)
    # ─── Funciones auxiliares ───────────────────────────────────────────

    """Multiplicación de polinomios en coeficientes descendentes."""
    function polyconv(a::Vector, b::Vector)
        c = zeros(promote_type(eltype(a), eltype(b)), length(a) + length(b) - 1)
        for i in eachindex(a), j in eachindex(b)
            c[i + j - 1] += a[i] * b[j]
        end
        return c
    end

    """Extrae coeficientes descendentes (grado mayor primero) de un Polinomio."""
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
        error("Debe asignar $gr_Dpbar polos para el observador")
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
    dise_1gdl(P, pol)

Calcula un controlador 1-GDL de realimentación unitaria que asigna los
polos de lazo cerrado al vector `pol`.

# Argumentos
- `P`: Función de transferencia de la planta.
- `pol::Vector`: Polos deseados de lazo cerrado.

# Retorna
- `K`: Función de transferencia del controlador.
- `T`: Función de transferencia de lazo cerrado (referencia → salida).
- `Gur`: Función de transferencia de referencia a control (`u = Gur·r`).
- `S`: Función de sensibilidad, `S = 1 - T`.
"""
function dise_1gdl(P, pol)

    # -- normalizar la función de transferencia

    P = tf(P)

    # ─── Función auxiliar ────────────────────────────────────────────────

    """Extrae coeficientes descendentes (grado mayor primero) de un Polinomio."""
    function descvec(p)
        return Float64.(reverse(coeffs(p)))
    end

    D = descvec(denpoly(P)[1, 1])
    N_raw = descvec(numpoly(P)[1, 1])
    n = length(D) - 1

    # Rellenar N con ceros a la izquierda (como hace tfdata de MATLAB)
    N = [zeros(length(D) - length(N_raw)); N_raw]

    # ─── Orden del controlador y polinomio deseado ─────────────────────

    m = length(pol) - n
    DT = descvec(fromroots(pol))

    # ─── Matriz de Sylvester  (n+m+1) × (2m+2) ─────────────────────────

    Sm = zeros(n + m + 1, 2m + 2)
    for k in 1:m+1
        Sm[k:k+n, k]       = D
        Sm[k:k+n, k+m+1]   = N
    end

    # ─── Resolución del sistema lineal ──────────────────────────────────

    local X::Vector{Float64}, Y::Vector{Float64}

    if m >= n
        # Intenta diseñar un controlador que rechaza perturbaciones
        Ind = setdiff(1:2m+2, m + 1)           # todas las columnas excepto m+1
        Sm_red = Sm[:, Ind]

        if rank(Sm_red) >= n + m + 1
            x = Sm_red \ DT
            Y = [x[1:m]; 0.0]                  # acción integral (cero en s=0)
            X = x[m+1:end]
        else
            x = Sm \ DT
            Y = x[1:m+1]
            X = x[m+2:end]
        end

    elseif m == n - 1
        x = Sm \ DT
        Y = x[1:m+1]
        X = x[m+2:2m+2]

    else
        error("El orden del controlador m=$m debe ser al menos n-1 = $(n-1)")
    end

    # ─── Controlador y funciones de lazo cerrado ─────────────────────────

    K   = tf(X, Y)
    T   = feedback(K * P)
    Gur = minreal(T / P)
    S   = minreal(1 - T)

    return K, T, Gur, S
end