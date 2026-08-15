

```@raw html
<div style="font-size: 1em; font-weight: bold; color: rgba(202, 60, 50, 1); background-color: #e9ddaf; border: 4px solid rgba(202, 60, 50, 1); border-radius: 8px; padding: 1em; margin: 1em 0;">
⚠️ La carga de firmware solo es necesaria si la placa está desconfigurada y el controlador por defecto no funciona. En el caso en que el firmware esté desactualizado, la plataforma transmitirá un mensaje indicando que debe hacerse una actualización de firmware al cargar el paquete <code>UNDCMotor</code>.
</div>
```
# Instalación del Firmware

El código fuente del firmware del ESP32 usado por la plataforma DCMotor se
encuentra en [`firmware/esp32_DCMotor`](https://github.com/nebisman/DCMotor.jl/tree/main/firmware/esp32_DCMotor), como parte del repositorio del paquete.

**Note que el proceso demora porque debe descargarse la libreria completa de programación del ESP32, la cual es muy estensa. Después de instalada por primera vez, el proceso es muy rápido.**

## Instalación en Linux

Para una instalación directa del firmware se provee un script de instalación que instala todas las dependencias. Este script requiere que `bash`, `curl`, `git` y `python3` estén actualmente en su instalación de Linux. Además. el
usuario debe pertenecer al grupo `dialout` para tener permisos sobre el
puerto serial (ver [Permisos del puerto serial en Linux](@ref)).

### Pasos de instalación 

#### Alternativa 1

Descargue y ejecute  el script de instalación desde la
terminal con:

```bash
curl -fsSLO https://raw.githubusercontent.com/nebisman/DCMotor.jl/main/firmware/instalador/cargar_firmware.sh
chmod +x cargar_firmware.sh
./cargar_firmware.sh
```


#### Alternativa 2
Descargue el script haciendo clic en el siguiente enlace de descarga directa:

[⬇ Descargar `cargar_firmware.sh`](https://raw.githubusercontent.com/nebisman/DCMotor.jl/main/firmware/instalador/cargar_firmware.sh)

Guárdelo en un directorio de trabajo cualquiera, abra una terminal en
ese mismo directorio. Otórguele permisos de ejecución a `cargar_firmware.sh` y ejecútelo, así:

```bash
chmod +x cargar_firmware.sh
./cargar_firmware.sh
```

Si la la placa está conectada por USB, el script detecta el puerto, compila el
firmware y lo carga automáticamente. Si no se detecta ningún puerto
compatible, o se detecta más de uno, el script se detiene con un mensaje de
error indicando cómo proceder.
