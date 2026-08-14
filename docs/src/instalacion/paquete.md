# Instalación del paquete

Este paquete requiere Julia 1.12 o superior. Se instala directamente desde este
repositorio:

```julia
using Pkg
Pkg.add(url="https://github.com/nebisman/DCMotor.jl")
```

Luego, para empezar a usarlo:

```julia
using DCMotor
```

Al cargar el paquete por primera vez en un directorio de trabajo elegido por el
usuario, este crea automáticamente allí las siguientes carpetas:

- `ejemplos/`: scripts, notebooks Pluto y notebooks Jupyter con ejemplos de
  identificación y control (PI, PID, diseño de dos parámetros, variables de estado,
  entre otros). Se recomienda iniciar con los ejemplos en el directorio `scripts/`.
- `datafiles/`: archivos CSV con los datos generados por los experimentos
  (identificación, respuesta al escalón, PRBS, etc.), usados como entrada y salida
  por las funciones del paquete.
