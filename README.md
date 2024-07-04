# caddy_ai2_sensors_SICK_LMS291-S05

Este repositorio tiene el objetivo de guardar los documentos, CADs, programas, código del sensor y tener un driver funcional para ROS 2.

## Instalación del driver 

Abrimos una terminal y nos dirigimos a la carpeta del código del driver.

```bash
cd docs_official/code/sicktoolbox-1.0.1-patch/
./configure
find . -type f -name Makefile -exec sed -i.bak 's/CXXFLAGS = -g -O2/CXXFLAGS = -g -O2 -std=c++11 -w/g' {} +
make
sudo make install
```

## Probar los ejemplos

En la paǵina 13 del manual (manuals/sicktoolbox-quickstart.pdf) podemos encontrar la descripción y como usar cada uno de los ejemplos.

#### Por ejemplo

Abrimos una terminal.

```bash
cd docs_official/code/sicktoolbox-1.0.1-patch/
cd c++/examples/lms/lms_partial_scan/src
sudo ./lms_partial_scan /dev/ttyUSB0 38400
```
Nota: Ajuste /dev/ttyUSB0 según el puerto serie de su dispositivo.

Antes de todo es importante configurarlo para ello seguramente cuando lo ejecutamos nos dice que configuración aplicar.

#### Configuración

*Nodo de ROS 2: Unidades -> cm* TODO pendiente verificar la unidades dinamicamente independiente a la configuración, así adaptarse según la configuración del sensor.

```bash
cd docs_official/code/sicktoolbox-1.0.1-patch/
cd c++/examples/lms/lms_config/src
sudo ./lms_config /dev/ttyUSB0 38400
```

## ROS 2 Driver

El driver para ROS 2 para este sensor esta basado en el driver de https://github.com/YDLIDAR/ydlidar_ros2

Nodo y aplicación de prueba para SICK

### Cómo construir el paquete

0) Abre una terminal y dirígete al workspace de ROS 2 o crea uno.
1) Clona este proyecto en la carpeta src del espacio de trabajo.
```bash
git clone https://github.com/racarla96/caddy_ai2_sensors_SICK_LMS291-S05.git
```
2) Ve a la raíz del workspace y compila el espacio de trabajo.
```bash
colcon build
```
3) Crear un alias del puerto serie llamado "/dev/sick"
```bash
cd caddy_ai2_sensors_SICK_LMS291-S05/startup
sudo chmod 777 initenv.sh
sudo sh initenv.sh
```
Si estaba conectado previamente, conectar y desconectar y comprobar que efectivamente aparece el nombre de sick
```bash
ls -la /dev/
```

## Cómo ejecutar el paquete

### 1. Ejecute el nodo y visualícelo usando la aplicación de prueba.

```bash
ros2 run sick sick_node
ros2 run sick sick_client
```

### 2.Ejecute el nodo y visualícelo usando la aplicación de prueba al iniciar

```bash
ros2 launch sick sick_launch.py
```

Con rviz2 podemos ver la salida de puntos del lidar.

## Conclusiones

El sensor del que disponemos la conexión la realiza mediante RS-232 a USB a 38400 baud, esta comunicación NO nos permite explotar todo el potencial del equipo, solo nos proporciona los puntos a una velocidad de 5 Hz con una resolución de 0.5 grados mínimo o una velocidad de 10 Hz con una resolución de un 1 grado.

## Posible mejora

Comprando un adaptador de RS–422 a USB se podría aumentar el baudrate a 500000, esto nos permitiría aumentar el potencial del sensor a 75 Hz.




#### Parche de código aplicado para poder compilar en nuevas versiones, testeado en Ubuntu 22.04 LTS

Este parche comprende las modificaciones de código de los ficheros originales contenidos en sicktoolbox-1.0.1-original.tar.gz a sicktoolbox-1.0.1-patch.tar.xz, ya aplicados necesarios para su compilación.

#### Enlaces útiles
- https://www.ros.org/reps/rep-0103.html
