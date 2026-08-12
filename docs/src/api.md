```@meta
CurrentModule = DCMotor
```

# Referencia de la API

## Funciones de interacción

```@docs
MotorSystem
connect!
disconnect!
transfer_function
speed_from_volts
volts_from_speed
```

## Funciones de identificación

```@docs
step_open
prbs_open
get_static_model
get_fomodel_step
get_model_prbs
```

## Funciones de control

```@docs
set_reference
set_pid
set_controller
step_closed
stairs_closed
profile_closed
```
