#!/bin/bash

# Script para crear una regla de udev para un adaptador USB-CAN
echo "Este script creará una regla udev para asignar un nombre personalizado a tu adaptador USB-CAN."
echo "Primero, veamos los mensajes del sistema para identificar tu adaptador:"
echo "-------------------------------------------------------------------"
dmesg | grep -i "usb\|can"
echo "-------------------------------------------------------------------"
echo
echo "↑ Busca en la salida anterior una línea que contenga 'SerialNumber:' o similar"
echo
echo "Introduce el número de serie (por ejemplo, HW033197):"
read HWSERIAL

INTERFACE_NAME="can_steer_drv"

# Crear la regla udev
RULE1="SUBSYSTEM==\"net\", ACTION==\"add\", ATTRS{serial}==\"$HWSERIAL\", NAME=\"$INTERFACE_NAME\", RUN+=\"/bin/sh -c 'sleep 1 && /sbin/ip link set $INTERFACE_NAME type can bitrate 1000000 && /sbin/ip link set up $INTERFACE_NAME'\""

echo -e "\nLa siguiente regla será añadida a /etc/udev/rules.d/81-can-usb.rules:"
echo $RULE1

# Solicita confirmación
read -p "¿Deseas continuar? (s/n): " CONFIRM
if [[ "$CONFIRM" != "s" ]]; then
    echo "Cancelado."
    exit 1
fi

# Escribe la regla
echo $RULE1 | sudo tee /etc/udev/rules.d/81-can-usb.rules

# Recarga las reglas de udev
sudo udevadm control --reload-rules
echo -e "\nRegla añadida y udev recargado."
echo "Para aplicar los cambios:"
echo "1. Desconecta el adaptador USB-CAN"
echo "2. Vuelve a conectarlo"
echo "3. Verifica el nombre con: ip link show"
