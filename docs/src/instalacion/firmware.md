# Instalación del firmware

El código fuente del firmware del ESP32 usado por la plataforma DCMotor se
encuentra en [`firmware/esp32_DCMotor`](https://github.com/nebisman/DCMotor.jl/tree/main/firmware/esp32_DCMotor),
dentro del repositorio del paquete.

Para instalarlo (compilarlo y cargarlo en la placa) se provee el script
[`cargar_firmware.sh`](https://github.com/nebisman/DCMotor.jl/blob/main/firmware/instalador/cargar_firmware.sh),
disponible en `firmware/instalador/` dentro del repositorio. El script:

1. Verifica si Arduino CLI está instalado; si no lo está, lo instala.
2. Verifica si el core `esp32:esp32` (Espressif) está instalado; si no lo
   está, lo instala.
3. Si el directorio actual no contiene ya `esp32_DCMotor/` con
   `esp32_DCMotor.ino` y `definitions.h`, descarga el firmware desde
   `firmware/esp32_DCMotor` en el repositorio de GitHub.
4. Detecta automáticamente el puerto USB-serial donde está conectada la
   placa (o informa si no pudo detectarlo).
5. Compila y sube el firmware a la placa.

## Requisitos

El script requiere `bash`, `curl`, `git` y `python3`. Además, en Linux, el
usuario debe pertenecer al grupo `dialout` para tener permisos sobre el
puerto serial — ver [Permisos del puerto serial en Linux](@ref).

## Uso

Descargue y ejecute el script en un directorio de trabajo cualquiera (creará
allí una subcarpeta `esp32_DCMotor/` con el código fuente descargado):

```bash
curl -fsSLO https://raw.githubusercontent.com/nebisman/DCMotor.jl/main/firmware/instalador/cargar_firmware.sh
chmod +x cargar_firmware.sh
./cargar_firmware.sh
```

Con la placa conectada por USB, el script detecta el puerto, compila el
firmware y lo carga automáticamente. Si no se detecta ningún puerto
compatible, o se detecta más de uno, el script se detiene con un mensaje de
error indicando cómo proceder.
