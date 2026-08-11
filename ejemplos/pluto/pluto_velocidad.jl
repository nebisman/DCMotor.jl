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

# ╔═╡ 9b61f289-d73d-43ed-89dd-0d1fbe22dcf7
begin
sys = MotorSystem(port="/dev/ttyUSB0");
s = tf("s")
b = 2395.3841;
a = 3.4665;
G = b/(s+a);
end

# ╔═╡ bb58e7a1-9b6f-466a-bb76-62390bcb9c6a


# ╔═╡ c946efee-dfd6-414b-83cf-5fd4d591ab6a
@bind ωn Slider(5:0.5:15.0, default=10.0, show_value=true)

# ╔═╡ a419c077-63f7-4b70-8a8a-dc1408eb6716
@bind ζ Slider(.2:0.05:1, default=0.6, show_value=true)

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


# ╔═╡ d7efac43-c45e-4fda-a682-95be82d183fb
begin
	p1=bodeplot(T)
	p2=pzmap(T)
	plot(p1,p2)
end

# ╔═╡ Cell order:
# ╠═b47a387e-680f-11f1-928e-8b266619493f
# ╠═bfcf7ec0-f320-46bf-93a4-a362aa9a3ab4
# ╠═9b61f289-d73d-43ed-89dd-0d1fbe22dcf7
# ╠═bb58e7a1-9b6f-466a-bb76-62390bcb9c6a
# ╠═c946efee-dfd6-414b-83cf-5fd4d591ab6a
# ╠═a419c077-63f7-4b70-8a8a-dc1408eb6716
# ╠═d6c868db-98e3-403b-ad93-cf33ced403f4
# ╠═fdd92ef6-8561-4afa-8e8f-e9ccadbcd475
# ╠═d7efac43-c45e-4fda-a682-95be82d183fb
