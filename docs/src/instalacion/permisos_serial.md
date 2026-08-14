# Permisos del puerto serial en Linux

En Linux, el acceso a los puertos seriales (por ejemplo, `/dev/ttyUSB0`) suele
estar restringido a los usuarios que pertenecen al grupo `dialout`. Si al
conectar la plataforma se obtiene un error de permisos (`Permission denied`),
añada su usuario a dicho grupo:

```bash
sudo usermod -aG dialout $USER
```

Luego cierre sesión y vuelva a iniciarla (o reinicie el equipo) para que el
cambio de grupo surta efecto. Puede verificar que el usuario quedó incluido
con:

```bash
groups $USER
```

Para identificar el nombre exacto del puerto asignado a la placa ESP32 al
conectarla por USB, puede usar:

```bash
dmesg | grep -i tty
```
