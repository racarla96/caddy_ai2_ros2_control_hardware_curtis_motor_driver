#!/bin/bash

# Desactivar la interfaz
sudo ip link set down vcan_motor_drv

# Eliminar la interfaz
sudo ip link delete vcan_motor_drv

echo "Interfaz virtual CAN 'vcan_motor_drv' eliminada."