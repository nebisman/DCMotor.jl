# DCMotor -- Laboratorio portátil de control de ángulo y velocidad angular de un servomotor.

[![Build Status](https://github.com/nebisman/DCMotor.jl/actions/workflows/CI.yml/badge.svg?branch=main)](https://github.com/nebisman/DCMotor.jl/actions/workflows/CI.yml?query=branch%3Amain)

Este paquete de Julia permite hacer experimentos de control e identificación con la plataforma portátil de experimentación **DCMotor**. Esta plataforma integra los componentes para controlar un servomotor DC por medio de un microcontrolador ESP32.

Desde una perspectiva de control y sistemas dinámicos, un motor DC permite representar vívidamente varios conceptos, a saber:

- Representa la respuesta de sistemas de primer orden estable cuando se controla la velocidad angular.
- Representa la respuesta de sistema de segundo orden  inestable con un integrador cuando se controla él ángulo.
- El motor DC es un ejemplo clásico de sistema multidominio, pues incluye elementos del dominio eléctrico y mecánico.

Adicionalmente, las constantes de tiempo están muy sintonizadas con la percepción humana. Esto convierte al motor DC en un sistema muy ilustrativo y pedagógico.

## Componentes de la plataforma

La siguiente imagen muestra la plataforma **DCMotor** junto con su componentes descritos en la tabla.
<img src="docs/assets/motor.png" alt="Plataforma DCMotor" width="600">

| # | Componente |
|---|---|
| 1 | Micromotor DC uxcell GA12-N20 6V 150RPM |
| 2 | Encoder de efecto hall 2800 ppr |
| 3 | Driver DRV8833 |
| 4 | Encoder de tipo perilla para interacción del usuario |
| 5 | LED inteligente para visualizar la señal de control como mapa de calor |
| 6 | Botones capacitivos para interacción del usuario |
| 7 | Soporte plástico del servomotor |
| 8 | Rueda que muestra el ángulo y la velocidad del servomotor |
| 9 | ESP32-S3 con factor de forma Arduino (abajo) |

## Instalación

Este paquete requiere Julia 1.12 o superior. Se instala directamente desde este repositorio:

```julia
using Pkg
Pkg.add(url="https://github.com/nebisman/DCMotor.jl")
```

Luego, para empezar a usarlo:

```julia
using DCMotor
```

Al cargar el paquete por primera vez en un directorio de trabajo elegido por el usuario, este crea automáticamente allí las siguientes carpetas:

- `datafiles/`: archivos CSV con los datos generados por los experimentos (identificación, respuesta al escalón, PRBS, etc.), usados como entrada y salida por las funciones del paquete.
- `ejemplos/`: scripts, notebooks Pluto y notebooks Jupyter con ejemplos de identificación y control (PI, PID, diseño de dos parámetros, variables de estado, entre otros).
