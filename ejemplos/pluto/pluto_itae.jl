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
	
#funcion de transferencia de angulo
G_ang = transfer_function(sys)
end

# ╔═╡ c946efee-dfd6-414b-83cf-5fd4d591ab6a
@bind w0 Slider(5:0.5:20.0, default=10.0, show_value=true)

# ╔═╡ 70b8a8dc-9a75-4531-b2f3-d43ec00a97e2


# ╔═╡ d6c868db-98e3-403b-ad93-cf33ced403f4
begin
T = (w0^3)/(s^3 + 1.75*s^2*w0 + 2.15*s*w0^2 + w0^3)
C2 = dise2p(G_ang, T, 2 ,[-100])
set_controller(sys, C2; output=:angle, deadzone=0.3);
result=step_closed(sys; r0 = 0, r1 = 90,  t0 = 0, t1 =2);
stepinfo_exp(result;T);
plot!()
end

# ╔═╡ 9d967262-8fae-4c4b-892f-1d621e1815dd
C2

# ╔═╡ b30812af-b632-47be-a3ae-7522f1ff7442
begin
plot(result[1], result[4], label="señal de control")
end

# ╔═╡ c4ccc4e6-7396-436b-bfd2-52710369d2eb


# ╔═╡ Cell order:
# ╠═b47a387e-680f-11f1-928e-8b266619493f
# ╠═bfcf7ec0-f320-46bf-93a4-a362aa9a3ab4
# ╠═9b61f289-d73d-43ed-89dd-0d1fbe22dcf7
# ╠═c946efee-dfd6-414b-83cf-5fd4d591ab6a
# ╠═70b8a8dc-9a75-4531-b2f3-d43ec00a97e2
# ╠═d6c868db-98e3-403b-ad93-cf33ced403f4
# ╠═9d967262-8fae-4c4b-892f-1d621e1815dd
# ╠═b30812af-b632-47be-a3ae-7522f1ff7442
# ╠═c4ccc4e6-7396-436b-bfd2-52710369d2eb
