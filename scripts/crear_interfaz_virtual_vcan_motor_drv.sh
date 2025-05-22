#!/bin/bash

# Cargar el módulo vcan si no está cargado
if ! lsmod | grep -q "^vcan"; then
    sudo modprobe vcan
fi

# Crear la interfaz virtual CAN
sudo ip link add dev vcan_motor_drv type vcan
sudo ip link set up vcan_motor_drv

echo "Interfaz virtual CAN 'vcan_motor_drv' creada y activada"