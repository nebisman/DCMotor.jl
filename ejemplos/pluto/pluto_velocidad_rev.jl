### A Pluto.jl notebook ###
# v1.0.1

using Markdown
using InteractiveUtils

# This Pluto notebook uses @bind for interactivity. When running this notebook outside of Pluto, the following 'mock version' of @bind gives bound variables a default value (instead of an error).
macro bind(def, element)
    #! format: off
    return quote
        local iv = try Base.loaded_modules[Base.PkgId(Base.UUID("6e696c72-6542-2067-7265-42206c756150"), "AbstractPlutoDingetjes")].Bonds.initial_value catch; b -> missing; end
        local el = $(esc(element))
        global $(esc(def)) = Core.applicable(Base.get, el) ? Base.get(el) : iv(el)
        el
    end
    #! format: on
end

# ╔═╡ b47a387e-680f-11f1-928e-8b266619493f
begin
    import Pkg
    Pkg.activate()
    using Markdown
using InteractiveUtils
end

# ╔═╡ bfcf7ec0-f320-46bf-93a4-a362aa9a3ab4
begin	
	using PlutoUI
	using DCMotor
	using Plots
	using ControlSystemsBase
	include("ControlUN.jl")
end

# ╔═╡ c1a00001-0001-4001-8001-000000000001
md"""
# Control de velocidad de un motor DC

Diseño de un controlador **PI** mediante **asignación de polos** (*pole placement*), de modo que el lazo cerrado se comporte como un sistema prototipo de segundo orden.

El notebook permite ajustar de forma interactiva la frecuencia natural ``\omega_n`` y el factor de amortiguamiento ``\zeta``, y comparar la **respuesta real** del motor (comunicación por ESP32) con la **respuesta prototipo** deseada.
"""

# ╔═╡ c1a00002-0002-4002-8002-000000000002
md"""
## 1. Modelo de la planta

La dinámica de **velocidad** del motor DC se modela como un sistema de primer orden entre la tensión de entrada ``u`` y la velocidad angular ``\omega``:

```math
G(s) = \frac{\Omega(s)}{U(s)} = \frac{b}{s+a}, \qquad b = 2395.3841,\quad a = 3.4665.
```

Equivalentemente, en forma de **ganancia – constante de tiempo**:

```math
G(s) = \frac{K}{\tau s + 1}, \qquad K = \frac{b}{a} \approx 691, \qquad \tau = \frac{1}{a} \approx 0.29\ \text{s}.
```

donde ``K`` es la ganancia estática (velocidad final por unidad de entrada) y ``\tau`` la constante de tiempo. El único polo en lazo abierto se ubica en ``s = -a``.
"""

# ╔═╡ 9b61f289-d73d-43ed-89dd-0d1fbe22dcf7
begin
sys = MotorSystem(port="/dev/ttyUSB0");
s = tf("s")
b = 2395.3841;
a = 3.4665;
G = b/(s+a);
end

# ╔═╡ bb58e7a1-9b6f-466a-bb76-62390bcb9c6a
md"""
## 2. Diseño del controlador PI

Se emplea un controlador proporcional–integral:

```math
C(s) = K_p + \frac{K_i}{s} = \frac{K_p\,s + K_i}{s}.
```

La acción integral garantiza **error de estado estacionario nulo** ante una referencia escalón.

#### Lazo abierto

```math
L(s) = C(s)\,G(s) = \frac{b\,(K_p s + K_i)}{s\,(s+a)}.
```

#### Lazo cerrado (realimentación unitaria)

```math
T(s) = \frac{L(s)}{1+L(s)} = \frac{b\,(K_p s + K_i)}{s^{2} + (a + bK_p)\,s + bK_i}.
```

#### Asignación de polos

Se busca que el polinomio característico coincida con el prototipo de segundo orden ``s^{2} + 2\zeta\omega_n s + \omega_n^{2}``. Igualando coeficiente a coeficiente:

```math
a + bK_p = 2\zeta\omega_n \;\Longrightarrow\; \boxed{\,K_p = \dfrac{2\zeta\omega_n - a}{b}\,}
```

```math
bK_i = \omega_n^{2} \;\Longrightarrow\; \boxed{\,K_i = \dfrac{\omega_n^{2}}{b}\,}
```

que son exactamente las fórmulas usadas en el código.

#### Observación: el cero del controlador

El lazo cerrado **real** tiene un cero en

```math
s = -\frac{K_i}{K_p} = -\frac{\omega_n^{2}}{2\zeta\omega_n - a},
```

que **no** aparece en el prototipo ``T(s)=\dfrac{\omega_n^{2}}{s^{2}+2\zeta\omega_n s+\omega_n^{2}}`` usado como referencia. Este cero tiende a **aumentar el sobrepaso** respecto al segundo orden ideal. La ganancia en continua sí es unitaria, ``T(0)=1``, por lo que se sigue la referencia escalón sin error permanente.
"""

# ╔═╡ c1a00003-0003-4003-8003-000000000003
md"""
## 3. Parámetros de diseño interactivos

Ajusta ``\omega_n`` (rapidez) y ``\zeta`` (amortiguamiento). Para el prototipo de segundo orden:

- **Sobrepaso:** ``M_p = e^{-\pi\zeta/\sqrt{1-\zeta^{2}}}`` (depende sólo de ``\zeta``).
- **Tiempo de establecimiento (2 %):** ``t_s \approx \dfrac{4}{\zeta\omega_n}``.
- **Tiempo de pico:** ``t_p = \dfrac{\pi}{\omega_n\sqrt{1-\zeta^{2}}}``.
- **Frecuencia amortiguada:** ``\omega_d = \omega_n\sqrt{1-\zeta^{2}}``.

> ⚠️ Para que ``K_p \ge 0`` se requiere ``2\zeta\omega_n \ge a``, es decir ``\zeta\omega_n \ge a/2 \approx 1.73``. Con valores muy bajos de ``\zeta`` y ``\omega_n`` la ganancia proporcional resultaría negativa.
"""

# ╔═╡ c946efee-dfd6-414b-83cf-5fd4d591ab6a
@bind ωn Slider(5:0.5:15.0, default=10.0, show_value=true)

# ╔═╡ a419c077-63f7-4b70-8a8a-dc1408eb6716
@bind ζ Slider(.2:0.05:1, default=0.6, show_value=true)

# ╔═╡ c1a00004-0004-4004-8004-000000000004
md"""
## 4. Cálculo de ganancias y experimento

Con los valores actuales de los deslizadores se calculan ``K_p`` y ``K_i``, se cargan en el controlador del motor (`set_pid`) y se aplica un escalón de referencia de 0 a 400 durante 2 s. La función `stepinfo_exp` compara la respuesta **experimental** del motor con la respuesta **prototipo** ``T(s)``.
"""

# ╔═╡ d6c868db-98e3-403b-ad93-cf33ced403f4
begin
# funcion de lazo cerrado
T  = ωn^2/(s^2 + 2*ζ*ωn*s +ωn^2)	

# calculo de las constantes
Kp = (2*ζ*ωn-a)/b
Ki = ωn^2/b

# experimentos
set_pid(sys;  kp=Kp, ki=Ki, kd=0, beta=0, output=:speed, deadzone=0)
	
# respuesta del controlador
result = step_closed(sys; r0 = 0, r1 = 400,  t0 = 0, t1 =2);
stepinfo_exp(result;T);
plot!()
end

# ╔═╡ fdd92ef6-8561-4afa-8e8f-e9ccadbcd475
md"""
## 5. Análisis en frecuencia y mapa de polos y ceros

- **Diagrama de Bode** de ``T(s)``: muestra el ancho de banda (próximo a ``\omega_n``) y el pico de resonancia, que crece a medida que disminuye ``\zeta``.
- **Mapa de polos y ceros** de ``T(s)``: los polos del prototipo se ubican en

```math
s = -\zeta\omega_n \pm j\,\omega_n\sqrt{1-\zeta^{2}},
```

sobre un círculo de radio ``\omega_n``; el ángulo respecto al eje real negativo cumple ``\theta = \arccos\zeta``. Al acercarse ``\zeta`` a 1 los polos se vuelven reales (respuesta sin sobrepaso) y al disminuir ``\zeta`` se aproximan al eje imaginario (mayor oscilación).
"""

# ╔═╡ d7efac43-c45e-4fda-a682-95be82d183fb
begin
	p1=bodeplot(T)
	p2=pzmap(T)
	plot(p1,p2)
end

# ╔═╡ Cell order:
# ╠═b47a387e-680f-11f1-928e-8b266619493f
# ╠═bfcf7ec0-f320-46bf-93a4-a362aa9a3ab4
# ╟─c1a00001-0001-4001-8001-000000000001
# ╟─c1a00002-0002-4002-8002-000000000002
# ╠═9b61f289-d73d-43ed-89dd-0d1fbe22dcf7
# ╟─bb58e7a1-9b6f-466a-bb76-62390bcb9c6a
# ╟─c1a00003-0003-4003-8003-000000000003
# ╠═c946efee-dfd6-414b-83cf-5fd4d591ab6a
# ╠═a419c077-63f7-4b70-8a8a-dc1408eb6716
# ╟─c1a00004-0004-4004-8004-000000000004
# ╠═d6c868db-98e3-403b-ad93-cf33ced403f4
# ╟─fdd92ef6-8561-4afa-8e8f-e9ccadbcd475
# ╠═d7efac43-c45e-4fda-a682-95be82d183fb
