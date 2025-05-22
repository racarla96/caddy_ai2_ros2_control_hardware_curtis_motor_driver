# SocketCAN Test

Este proyecto implementa una interfaz sencilla para comunicación CAN utilizando SocketCAN y epoll para lectura eficiente de mensajes. Está diseñado para probar la comunicación CAN en modo RAW, leyendo mensajes en lotes de hasta 4 frames a la vez.

## Características

- Interfaz SocketCAN en modo RAW
- Lectura no bloqueante con epoll
- Lectura en lotes de hasta 4 mensajes CAN
- Frecuencia de operación de 25Hz (reducida de 250Hz con un contador)
- Manejo de señales para salida limpia

## Requisitos

- Sistema Linux con soporte para SocketCAN
- CMake (versión 3.8 o superior)
- Compilador C++ con soporte para C++14
- libsocketcan-dev
- can-utils (opcional, para pruebas)

## Instalación

### 1. Instalar dependencias

```bash
sudo apt-get install libsocketcan-dev can-utils
```

### 2. Compilar el proyecto

```bash
mkdir build
cd build
cmake ..
make
```

## Configuración de la interfaz CAN

### Para interfaz CAN física

```bash
# Cargar módulos del kernel
sudo modprobe can
sudo modprobe can_raw

# Configurar interfaz (ajusta el bitrate según sea necesario)
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0
```

### Para interfaz CAN virtual (pruebas)

```bash
# Cargar módulos del kernel
sudo modprobe can
sudo modprobe can_raw
sudo modprobe vcan

# Crear y activar interfaz virtual
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

## Uso

### Ejecutar el programa

```bash
# Para interfaz física
./test_socket_can_interface_reader can0

# O para interfaz virtual
./test_socket_can_interface_reader vcan0
```

El programa mostrará los mensajes CAN recibidos en formato hexadecimal.

### Enviar mensajes de prueba (con interfaz virtual)

En otra terminal, puedes enviar mensajes de prueba:

```bash
# Formato: cansend <interfaz> <ID>#<DATOS>
cansend vcan0 123#DEADBEEF
```

### Enviar mensajes de prueba (con interfaz virtual) y un fichero dump

```bash
# Terminal 1
./test_socket_can_interface_reader vcan_motor_drv
```

```bash
# Terminal 2
canplayer -I candump-2025-02-13_124653_conector_rojo.log vcan_motor_drv=can0
```

## Estructura del proyecto

```
socketcan_test/
├── CMakeLists.txt
├── include/
│   └── socketcan_interface.h
└── src/
    ├── socketcan_interface.cpp
    └── main.cpp
```

## Detalles de implementación

- `SocketCANInterface`: Clase que encapsula la funcionalidad de SocketCAN
- `read()`: Lee hasta 4 mensajes CAN a la vez usando epoll
- `write()`: Envía un mensaje CAN
- El programa principal ejecuta la lectura a 25Hz (cada 10 ciclos a 250Hz)

## Depuración

Si encuentras problemas con la interfaz CAN, puedes verificar su estado:

```bash
# Ver estado de todas las interfaces CAN
ip -details link show

# Ver estadísticas
ip -details -statistics link show can0

# Monitorear mensajes CAN (útil para depuración)
candump can0
```

## Notas

- El programa captura la señal SIGINT (Ctrl+C) para cerrar limpiamente la interfaz.
- Si usas una interfaz física, asegúrate de que esté correctamente conectada y configurada.
- Para interfaces virtuales, necesitarás enviar mensajes manualmente para ver resultados.

## Personalización

- Modifica `max_frames` en la llamada a `read()` para cambiar el número de mensajes leídos en cada ciclo.
- Ajusta el tiempo de espera en `epoll_wait()` (actualmente 100ms) según tus necesidades.
- Cambia el intervalo de sueño en el bucle principal para ajustar la frecuencia base.

## Licencia

Este proyecto se distribuye bajo la licencia MIT.
