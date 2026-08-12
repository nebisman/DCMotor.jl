# DCMotor -- Laboratorio portátil de control de ángulo y velocidad angular de un servomotor.

[![Build Status](https://github.com/nebisman/DCMotor.jl/actions/workflows/CI.yml/badge.svg?branch=main)](https://github.com/nebisman/DCMotor.jl/actions/workflows/CI.yml?query=branch%3Amain)

Este paquete de Julia permite el control e identificación con la plataforma portátil de experimentación **DCMotor**. Esta plataforma es para el control de un servomotor DC por medio de un microcontrolador ESP32.

Desde una perspectiva de control y sistemas dinámicos, el motor DC permite representar vívidamente varios conceptos, a saber:

- Representa la respuesta de sistemas de primer orden (o segundo, si se considera el polo eléctrico) estable cuando se controla la velocidad.
- Representa la respuesta de sistema de segundo orden (o tercero, si se considera el polo eléctrico) inestable con un integrador cuando se controla la posición.
- El motor DC es un ejemplo fundamental de sistema multidominio, pues incluye elementos del dominio eléctrico y mecánico.

Adicionalmente, las constantes de tiempo están muy sintonizadas con la percepción humana. Esto convierte al motor DC en un sistema muy ilustrativo y pedagógico.

## Componentes de la plataforma

<img src="docs/assets/motor.png" alt="Plataforma DCMotor" width="800">

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
