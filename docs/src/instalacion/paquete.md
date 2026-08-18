# Instalación del paquete

Este paquete requiere Julia 1.12 o superior. Se instala directamente desde este
repositorio, así:

```julia
using Pkg
Pkg.add(url="https://github.com/nebisman/DCMotor.jl")
```

Luego, para empezar a usarlo, ejecute el siguiente código:

```julia
using DCMotor
sys = MotorSystem();
set_pid(sys;  kp=0.1886, ki=1, kd=0.0069)
set_reference(sys,360)
```
El sistema debe reaccionar y mostrar el cambio a una referencia de $$360^o$$ en ángulo.

## Notas
Cuando el usuario carga el paquete por primera vez en un directorio de trabajo, se crean automáticamente allí las dos siguientes carpetas:

- `ejemplos/`: scripts, notebooks Pluto y notebooks Jupyter con ejemplos de
  identificación y control (PI, PID, diseño de dos parámetros, variables de estado,
  entre otros). Se recomienda iniciar revisando los ejemplos en el directorio `scripts/`.
- `datafiles/`: archivos CSV con los datos generados por los experimentos
  (identificación, respuesta al escalón, PRBS, etc.), usados como entrada y salida  por las funciones del paquete.
