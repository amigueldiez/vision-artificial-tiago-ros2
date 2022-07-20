# Proyecto visión con TIAGo

![image](https://img.shields.io/badge/Python-FFD43B?style=for-the-badge&logo=python&logoColor=blue)

## Objetivo y desarrollo del proyecto

El proyecto consiste en permitir analizar las imágenes recibidas por el topic de la cámara de [TIAGo](https://grupoadd.es/el-robot-tiago). Para ello se realiza un nodo que se suscribe a las imágenes del topic, y mediante un algoritmo de visión artificial es capaz de diferenciar tazas y vasos. Este nodo enviará por otro topic las _bounding boxes_ que encuentre en la imagen. Posteriormente, se ha desarrollado otro nodo que se suscribe a dos topics, el de la cámara y el de las _bouding boxes_, y envía por un topic las imagen con las tazas y vasos que ha reconocido.

Para poder distinguir los objetos se ha utilizado como arquitectura [YOLOv5](https://github.com/ultralytics/yolov5) y se ha entrenado con diversas imágenes de tazas y vasos tomadas por nosotros mismos en el laboratorio.

Además, el proyecto incluye un nodo `img_recorder` que permite grabar videos de un topic que transmita `Image`, muy útil para luego extraer _frames_. Estos serán utilizados para etiquetar los objetos que aparezcan.

## Nodos de ROS2

`img_analyser` recibe imágenes, las analiza con YOLOv5 y envía las _bounding boxes_.

`img_recorder` recibe imágenes y graba un vídeo con todos los _frames_ capturados.

`img_visualizer`es un nodo que sincroniza los mensajes con las imágenes de la cámara y las _bounding boxes_ y envía la imagen por un topic con los objetos que ha reconocido.

## Puesta en marcha

[TIAGo](https://grupoadd.es/el-robot-tiago) funciona en ROS1, y el código que hemos desarrollado se encuentra en ROS2. Por lo tanto, hay que utilizar un bridge de ROS1 a ROS2.

### Prerequisitos

Los siguientes requisitos se dan por hecho que se cumplen:

- Tener insalado ROS2 Foxy (se ha probado solamente con esta versión, no se puede afirmar que con otra versión funcione).

- Usar un sistema operativo Linux, Ubuntu como preferencia.

### Descarga de ROS1

Instalar ROS1 Noetic, para ello, se puede consultar la documentación oficial http://wiki.ros.org/noetic/Installation/Ubuntu

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update

sudo apt install ros-noetic-ros-ba
```

### Instalar el Bridge de comunicación entre ROS1 y ROS2

Toda la información sobre el bridge se encuentra en el siguiente repositorio https://github.com/ros2/ros1_bridge

Se puede instalar ejecutando el siguiente comando.

```bash
sudo apt install ros-foxy-ros1-bridge
```

### Aconsejable: Utilizar ros2_utils_scripts

Se aconseja utilizar el repositorio de Miguel Ángel González Santamarta que ofrece scripts que permiten cambiar fácilmente de versión de ROS.

Cómo instalarlo y utilizarlo se puede consultar en el repositorio oficial: https://github.com/mgonzs13/ros2_utils_scripts

### Instalar YOLOv5

Instalar en el directorio ráiz del workspace YOLOv5.

```bash
git clone https://github.com/ultralytics/yolov5  # clone

cd yolov5

pip install -r requirements.txt  # install
```

Además, hay que incluir en el directorio raíz del WS un fichero cuyo nombre será `best.pt`con los pesos de la red entrenada.

### Instalar en tu workspace el paquete de este repositorio

Instala en tu workspace el paquete `cv_basics` que se encuentra en este repositorio.

### Arrancarlo

Se va a utilizar los `ros2_utils_scripts` por la facilidad que ofrece a la hora de utilizar el bridge.

Abrir una terminal y entrar en el workspace que se esté trabajando. Posteriormente ejecutar lo siguiente:

```bash
rosconfig -d foxy

rosconfig -d noetic -m ROS_MATER_URI -i ROS_IP

ros2 run ros1_bridge dynamic_bridge
```

Abrir otra terminal y ejecutar lo siguiente:

```bash
rosconfig -d foxy

colcon build

. install/setup.bash

ros2 launch cv_basics image_visualizer_launch.py

```

Para poder visualizar las imágenes analizadas utilizar `rviz2`.

```bash
rosconfig -d foxy

ros2 run rviz2 rviz2
```

## Autores

| Autores                | Email de contacto               |
| ---------------------- | ------------------------------- |
| Naamán Huerga Pérez    | nhuerp00@estudiantes.unileon.es |
| Beatriz Jové De Castro | bjoved00@estudiantes.unileon.es |
| Alberto Miguel Diez    | amigud00@estudiantes.unileon.es |

Además, dar gracias a Miguel Ángel González Santamarta y Virginia Riego del Castillo por la ayuda en su realización.
