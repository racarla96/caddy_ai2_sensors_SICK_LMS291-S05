# caddy_ai2_sensors_SICK_LMS291-S05

Este repositorio tiene el objetivo de guardar los documentos, CADs, programas y código del sensors.

## TODOs

- [ ] Desarrollo del package para ROS 2

## Conclusiones

El sensor del que disponemos la conexión la realiza mediante RS-232 a USB a 38400 baud, esta comunicación NO nos permite explotar todo el potencial del equipo, solo nos proporciona los puntos a una velocidad de 5 Hz.

## Posible mejora

Comprando un adaptador de USB–COMi–M USB a RS–422 se podría aumentar el baudrate a 500000, esto nos permitiría aumentar el potencial del sensor a 75 Hz.


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

Antes de todo es importante configurarlo

#### Configuración

```bash
cd docs_official/code/sicktoolbox-1.0.1-patch/
cd c++/examples/lms/lms_config/src
sudo ./lms_config /dev/ttyUSB0 38400
```

Aquí hay que navegar por el menu y activar la opción de 

#### Por ejemplo

Abrimos una terminal.

```bash
cd docs_official/code/sicktoolbox-1.0.1-patch/
cd c++/examples/lms/lms_partial_scan/src
sudo ./lms_partial_scan /dev/ttyUSB0 38400
```

#### Parche de código para poder compilar en nuevas versiones, testeado en Ubuntu 22.04 LTS

Este parche comprende las modificaciones de código de lod ficheros originales contenidos en sicktoolbox-1.0.1-original.tar.gz a sicktoolbox-1.0.1-patch.tar.xz, ya aplicados.

```bash
sed -i 's/static const double SICK_MAX_SCAN_ANGULAR_RESOLUTION/static constexpr double SICK_MAX_SCAN_ANGULAR_RESOLUTION/; s/static const double SICK_DEGREES_PER_MOTOR_STEP/static constexpr double SICK_DEGREES_PER_MOTOR_STEP/' c++/drivers/ld/sickld-1.0/SickLD.hh
sed -i '/#include <iostream>/a #include <unistd.h>' c++/drivers/base/src/SickBufferMonitor.hh
sed -i '/#include <iostream>/a #include <unistd.h>' c++/drivers/base/src/SickLIDAR.hh
sed -i 's/ifstream(config_path.c_str()) != NULL/ifstream(config_path.c_str())/g' c++/examples/ld/ld_config/src/main.cc
```

