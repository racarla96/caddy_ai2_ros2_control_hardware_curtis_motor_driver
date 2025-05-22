README: Uso de interfaz virtual CAN y interactuar con candump y canplayer

Este documento explica cómo crear una interfaz virtual CAN llamada vcan_motor_drv, cómo eliminarla y cómo reproducir un log CAN usando canplayer.

1. Crear la interfaz virtual CAN

Guarda el siguiente script como create_vcan.sh y dale permisos de ejecución:

chmod +x create_vcan.sh


Contenido de create_vcan.sh o en la carpeta scripts/crear_interfaz_virtual_vcan_motor_drv.sh:

#!/bin/bash

# Cargar el módulo vcan si no está cargado
if ! lsmod | grep -q "^vcan"; then
    sudo modprobe vcan
fi

# Crear la interfaz virtual CAN
sudo ip link add dev vcan_motor_drv type vcan
sudo ip link set up vcan_motor_drv

echo "Interfaz virtual CAN 'vcan_motor_drv' creada y activada."


Ejecuta el script con:

sudo ./create_vcan.sh

2. Eliminar la interfaz virtual CAN

Guarda el siguiente script como delete_vcan.sh y dale permisos de ejecución:

chmod +x delete_vcan.sh


Contenido de delete_vcan.sh o en la carpeta scripts/eliminar_interfaz_virtual_vcan_motor_drv.sh:

#!/bin/bash

# Desactivar la interfaz
sudo ip link set down vcan_motor_drv

# Eliminar la interfaz
sudo ip link delete vcan_motor_drv

echo "Interfaz virtual CAN 'vcan_motor_drv' eliminada."


Ejecuta el script con:

sudo ./delete_vcan.sh

3. Reproducir un log CAN con canplayer

Asegúrate de tener instalado el paquete can-utils:

sudo apt-get update
sudo apt-get install can-utils


Para reproducir el log candump-2025_05_19_test2_conector_rojo.log en la interfaz virtual vcan_motor_drv, usa:

canplayer -I candump-2025-02-13_124653_conector_rojo.log vcan_motor_drv=can0


Esto enviará los mensajes del log a la interfaz virtual para pruebas o desarrollo.

Nota: Si necesitas establecer un bitrate, ten en cuenta que las interfaces vcan no requieren bitrate, ya que son virtuales y no transmiten físicamente. El parámetro de bitrate se usa en interfaces físicas (por ejemplo, can0).


4. Capturar tráfico CAN con candump

Para capturar el tráfico CAN en la interfaz can_motor_drv (o vcan_motor_drv si estás usando la interfaz virtual), usa el siguiente comando:

candump can_motor_drv


Esto mostrará en la terminal todo el tráfico CAN que pasa por la interfaz. Si quieres guardar el tráfico en un archivo, puedes usar:

candump can_motor_drv -l > candump_$(date +%Y_%m_%d_%H_%M_%S).log


Este comando guardará el tráfico en un archivo con un nombre que incluye la fecha y hora actuales.

Nota: Si necesitas establecer un bitrate, ten en cuenta que las interfaces vcan no requieren bitrate, ya que son virtuales y no transmiten físicamente. El parámetro de bitrate se usa en interfaces físicas (por ejemplo, can0).