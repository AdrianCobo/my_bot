# My bot package

Proyecto basado en el canal de youtube de [Articulated Robotics](https://www.youtube.com/@ArticulatedRobotics/videos)

## My bot Mk2
<div align="center">
<img width=500px src="https://github.com/AdrianCobo/my_bot/blob/main/imgs/mybot_mk2.jpg" alt="explode"></a>
</div>

## My bot Mk1
<div align="center">
<img width=500px src="https://github.com/AdrianCobo/my_bot/blob/readme_updated/imgs/my_bot_mk1_real.jpg" alt="explode"></a>
</div>

## My bot Mk0
<div align="center">
<img width=500px src="https://github.com/AdrianCobo/my_bot/blob/readme_updated/imgs/my_bot_mk1_sim.png" alt="explode"></a>
</div>


# Objetivo General  
El objetivo de este proyecto es desarrollar y simular un robot diferencial del tamaño de una caja de zapatos, utilizando las capacidades de ROS 2 para la simulación y control de robots. A lo largo de las fases del proyecto, se explorarán diversas tecnologías y conceptos esenciales en robótica.

# Fase 1: Simulación

## Fase 1.1: Uso de Tfs
La primera aproximación elegida fue entender cómo funcionaba el sistema de transformadas en ROS para verificar el comportamiento correcto durante la simulación de nuestro robot a través del paquete tf2.

https://github.com/AdrianCobo/my_bot/assets/98641977/d725b8a4-27a4-4976-9d00-0e46af1a7a92

## Fase 1.2: Diseño de Robot a partir de URDF
Una vez comprendido cómo funcionan los sistemas de transformadas para las articulaciones del robot, se optó por modelar el robot utilizando el formato URDF para que fuera compatible con Gazebo. Para facilitar esta tarea, se decidió también utilizar Xacro, lo que simplificó enormemente el proceso.

## Fase 1.3: Gazebo
Con el archivo URDF listo, se incorporaron los plugins necesarios para que Gazebo pudiera interpretarlo. Además, se tomó la decisión de crear archivos .world y un lanzador para facilitar el inicio de la simulación.

```console
    ros2 launch my_bot launch_real_sim.launch.py
```

Puedes ver el video de demostración aqui: [(Youtube)](https://youtu.be/H0Chc4LrjQw)

[![Alt text](https://img.youtube.com/vi/H0Chc4LrjQw/0.jpg)](https://www.youtube.com/watch?v=H0Chc4LrjQw)

# Fase 2: Simulación de sensores y actuadores

Una vez que se tenía la simulación del chasis, el objetivo pasó a ser la simulación de sensores (cámara, cámara con RGBD y LIDAR), así como el controlador diferencial. Todo esto resultó bastante sencillo de realizar gracias a los plugins ofrecidos por Gazebo para una simulación precisa. Simplemente teníamos que añadir estos plugins utilizando Xacro y generando el URDF que Gazebo utilizaría para la correcta simulación.

Para verificar el correcto funcionamiento, se empleó RViz2 para visualizar los datos captados y el paquete teleop_twist_keyboard para enviar comandos de velocidad al controlador de motores.

Estos archivos se pueden revisar en:
- [Camara](https://github.com/AdrianCobo/my_bot/blob/main/description/camera.xacro)
- [Camara RGBD](https://github.com/AdrianCobo/my_bot/blob/main/description/depth_camera.xacro)
- [Lidar](https://github.com/AdrianCobo/my_bot/blob/main/description/lidar.xacro)
- [Controlador diferencial de Gazebo](https://github.com/AdrianCobo/my_bot/blob/main/description/gazebo_control.xacro)

```console
    ros2 launch my_bot launch_real_sim.launch.py
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Puedes ver un video de demostración aqui: [(Youtube)](https://youtu.be/-zVjHXezQI8)

[![Alt text](https://img.youtube.com/vi/-zVjHXezQI8/0.jpg)](https://www.youtube.com/watch?v=-zVjHXezQI8)

# Fase 3: Control por Joystick y uso de ros2_control.

Aunque el controlador diferencial proporcionado por Gazebo funciona correctamente, dado que se buscaba que el proyecto no solo se quedara en un robot simulado, sino que se llevara a la realidad, se optó por utilizar el paquete estándar de ROS para el control de motores (ros2_control). Esto no solo facilitaría la comunicación con los motores y encoders del robot real, sino que además el controlador resulta considerablemente mejor.

Como en los casos anteriores, simplemente tuvimos que configurar adecuadamente en un archivo .xacro los plugins necesarios para emplear este paquete. El fichero correspondiente se puede revisar [aquí](https://github.com/AdrianCobo/my_bot/blob/main/description/ros2_control.xacro)

Además, dado que el control con teleop_twist_keyboard puede resultar incómodo en algunas ocasiones, también se decidió implementar un control a través de un mando de PlayStation 4. Para lograrlo, se utilizó el paquete ros2 joy, que convierte los valores analógicos del joystick en un mensaje de ROS2 de tipo joy, y el paquete teleop_twist_joy, que convierte los mensajes de tipo joy a tipo cmd, que son los mensajes que comprende el controlador de motores.

*Cabe destacar que el lanzador de la simulación también inicia los nodos del joystick.

# Fase 4: Mapeado y navegación autónoma.

Para esta fase, se optó por utilizar los paquetes más populares de ROS2 para ambos temas. Se empleó el paquete slam_toolbox para la construcción de mapas y el paquete nav2 para la navegación autónoma.

Para lograr un funcionamiento correcto, solo fue necesario instalar los paquetes necesarios y completar los archivos de configuración de ambos paquetes según nuestros requerimientos. Además, slam_toolbox ofrece un panel para RViz2 que simplifica en gran medida el almacenamiento de los mapas generados.

```console
    ros2 launch my_bot launch_real_sim.launch.py
    cd ~/your_ws
    rviz2 -d src/my_bot/robot_view.rviz
    cd ~/your_ws
    ros2 launch nav2_bringup localization_launch.py map:=./src/my_bot/maps/home.yaml use_sim_time:=true
    # publish a 2D point using rviz at your robot map position
    ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true map_subscribe_transient_local:=true
```

Puedes ver un video de demostración aqui: [(Youtube)](https://youtu.be/tctQYJnHBAQ)

[![Alt text](https://img.youtube.com/vi/tctQYJnHBAQ/0.jpg)](https://www.youtube.com/watch?v=tctQYJnHBAQ)

# Fase 5: Planificación

Se consideró una excelente idea pensar en algún uso real para este robot. En ese momento, la práctica propuesta para la asignatura de Planificación y Robótica de Servicios era la de un robot de patrullaje. Por lo tanto, se decidió llevar a cabo este ejercicio utilizando el paquete plansys2, junto con las técnicas de mapeado y navegación autónoma explicadas anteriormente.

Puedes ver un video de demostración aqui: [(Youtube)](https://youtu.be/MbuiRqzs0qQ)

[![Alt text](https://img.youtube.com/vi/MbuiRqzs0qQ/0.jpg)](https://www.youtube.com/watch?v=Q_-EYw8jdps)

# Fase 6: Diseño 2D

Para esta parte del proyecto se ha utilizado **Inkscape**, tanto desde su entorno gráfico para el diseño del circuito y planos del chasis como desde la terminal para imágenes animadas (gifs).

### Primeros pasos para el diseño del circuito
En esta parte se trazaron 'cables' con lineas rectas de colores, el circuito ya estaba diseñado por lo que solo hubo que ir copiando los elementos fisicos del sistema y añadir los cables.


Aqui hay una imagen del resultado del circuito:

<img src="https://github.com/dduro2020/my_bot/blob/main/imgs/circuit2d/Circuito.jpeg" alt="Circuito" width="500"/>

Para la creación del gif se tomaron 2 frames, el primero el del circuito original y luego otro se pintó por encima de todos los cables rojos(5V/12V) como si tuviesen corriente, además se añadió una imagen de una mano para simular que se pulsa el botón. Para la mano nos ayudamos de una IA que recorta todo el fondo de forma que no se viese un cuadrado blanco al añadir la imagen al .svg.

Comando para la creación del gif:
```bash
convert -delay 100 -loop 0 img1.svg img2.svg animate.gif
```

Y aqui una demostración del gif:

<img src="https://github.com/dduro2020/my_bot/blob/main/animate/animacion_circuito.gif" alt="Circuito" width="500"/>

### Diseño de las vistas
En este paso además de utilizar inkscape habia que ir copiando las coordenadas donde colocábamos cada elemento para guardar nuestra escala. En inkscape los elementos se van guardando por mm, y como nuestra escala real era de cm, se convirtió cada cm a 5mm, de esta forma era mucho más visual en inkscape.

Para añadir las flechas con las distancias se utilizaron nuevamente segmentos modificando las puntas de cada uno.

Aqui todas las vistas:

<img src="https://github.com/dduro2020/my_bot/blob/main/imgs/vistas/all.png" alt="Circuito" width="500"/>

# Fase 7: Diseño 3D

El software utilizado para el diseño 3D es **FreeCAD**.

### Primera versión del diseño 3D del robot
La idea inicial para el chasis del robot era utilizar una caja de zapatos de cartón.


Para tener una base de la que partir, creamos una caja maciza para posteriormente integrar poco a poco los elementos visibles del robot. En esta primera versión del diseño no se tenía pensado imprimir un modelo 3D por lo que no se tuvieron en cuenta las medidas de cada uno de los elementos del robot ni si estaban correctamente ubicadas.

En la siguiente animación se muestra esta primera versión del diseño 3D.

https://github.com/daquinga2020/my_bot/assets/90764494/e568a732-69b1-45d3-80f1-09258435fc54


### Segunda versión del diseño 3D del robot
Una vez terminada la primera versión del diseño y ver cómo había quedado, la idea de imprimir el chasis fue cogiendo cada vez más fuerza.

Para este diseño se tomaron correctamente las medidas de cada uno de los elementos del robot, además de modificar la localización de las ruedas motrices con respecto al anterior diseño.

Las principales modificaciones con respecto al primer diseño fueron:
- Cambio de la localización de las ruedas motrices.
- Redimensionamiento del cuerpo principal y vaciado de la caja maciza para colocar el respectivo hardware del robot en su interior.
- Taladros en los laterales del cuerpo principal para los motores de las ruedas, la cámara, el switch y en la base para fijar los componentes hardware del robot.
- Creación de una tapa con los respectivos enganches necesarios y una bisagra con la que poder abrir y cerrar el techo del chasis para poder acceder a los componentes internos si fuerea necesario.


En la siguiente animación se muestra la segunda versión del diseño 3D.

https://github.com/daquinga2020/my_bot/assets/90764494/049c0610-bedf-4ad5-ab16-4d8a47d8ea9a


### Tercera versión del diseño 3D del robot
Para la impresión 3D del chasis del robot(cuerpo, tapa y bisagra) utilizamos la impresora 3D **Sigma D25** que utiliza el software [**BCN3D Stratos**](https://www.bcn3d.com/es/bcn3d-stratos/).

Al ser la primera vez que utilizábamos una impresora 3D no sabíamos si el modelo diseñado iba a ser costoso de imprimir tanto a nivel de recursos como de tiempo. Cuando introducimos nuestro modelo en el software de la impresora, nos dió un tiempo impresión de varios días para la tapa y el cuerpo del robot, obligándonos a rediseñar el modelo 3D para que tardara menos y usara menos material.

La solución por la que optamos fue reducir el grosor de la base y de las paredes tanto de la caja como de la tapa, vaciar la tapa(antes era un rectángulo macizo); dividir el cuerpo principal por partes, y además creamos un tapón para evitar que el cilindro que actua como bisagra se saliera. Por último, decidimos volver a rellenar los taladros anteriormente hechos para realizarlos a mano, consiguiendo de esta manera ajustar cada componente tal como se quiera.

En la siguiente animación se muestra la tercera versión del diseño 3D.

https://github.com/daquinga2020/my_bot/assets/90764494/d1b16ddc-f6ee-42db-b85d-8c9ffe582500


### Diseño 3D de los componentes

A continuación se muestran los componentes que se han creado en cada una de las versiones:

- Cámara

https://github.com/daquinga2020/my_bot/assets/90764494/c81fc53c-187e-4f84-988c-0f723fb702be


- Caster Wheel

https://github.com/daquinga2020/my_bot/assets/90764494/5ea4b8b8-ee9d-47cc-abaf-04b870f58b02


- Chasis taladrado con bisagra

https://github.com/daquinga2020/my_bot/assets/90764494/0adf89d2-fc14-443b-aa53-efdad2502d64


- Enganche

https://github.com/daquinga2020/my_bot/assets/90764494/0e65d31b-dff1-451d-a4e4-608fcec9639d


- Enganche de la tapa

https://github.com/daquinga2020/my_bot/assets/90764494/cfd4c74c-db03-47c0-9e5c-e498cc71f9d8


- Switch

https://github.com/daquinga2020/my_bot/assets/90764494/ef9acf15-8c97-4fef-bd2d-ced013bb0f61


- LiDAR

https://github.com/daquinga2020/my_bot/assets/90764494/a7546a7d-702a-4f6c-bc94-04d39be2bdf6


- Rueda motriz

https://github.com/daquinga2020/my_bot/assets/90764494/116bb856-129e-4830-abbb-09275b083e95


- Tapa de la bisagra

https://github.com/daquinga2020/my_bot/assets/90764494/66058c50-f7b3-49c2-aa96-2603b0abb255


- Tapa hueca

https://github.com/daquinga2020/my_bot/assets/90764494/03134156-ade7-4f7a-be5d-be300c4b1d8e


# Fase 8: Sensores y actuadores reales

Una vez completadas las pruebas de simulación, se tomó la decisión de adquirir los componentes necesarios para realizar las pruebas en el circuito del robot.

La tarea más desafiante fue establecer la comunicación entre los sensores y actuadores con ROS2, en particular con el controlador de motores y el Arduino. Esto requería la creación de un puente entre ROS2 y Arduino, y se optó por utilizar una solución ya existente, como se puede ver en esta opción. Además, se aprovechó el código proporcionado por Buzzology código de Buzzology para implementar el control diferencial de los motores con Arduino.

Finalmente, para el LIDAR, se adquirió un modelo compatible con ROS2 que ya contaba con sus propios nodos y lanzadores de ROS2.

Puedes ver la lista con todos los componentes y precios [aquí](https://github.com/AdrianCobo/my_bot/blob/readme_updated/Bill.md)

Puedes ver un video de demostración aqui: [(Youtube)](https://youtu.be/Q_-EYw8jdps?si=o-zX6n-kzWKuK6uY)

[![Alt text](https://img.youtube.com/vi/Q_-EYw8jdps/0.jpg)](https://www.youtube.com/watch?v=Q_-EYw8jdps)

Recuerda también que tienes todos los pasos para la correcta puesta a punto y lanzamiento del entorno de simulación, así como los nodos necesarios para ejecutar el código en un robot real parecido.

# Tests:

## My bot Mk2 Joystick control

Puedes ver un video de demostración aqui: [(Youtube)](https://www.youtube.com/shorts/yJ6x7noP0hA)

[![Alt text](https://yt3.googleusercontent.com/KT4Z1J3JNjCKyoHJ9xA7ygYh9z6PdhswTFh7YPeBkGz8lkxBnhvhUrZqDoduH89xp_3Sd2Ord0I=s176-c-k-c0x00ffffff-no-rj)](https://www.youtube.com/shorts/yJ6x7noP0hA)

## My bot Mk2 lidar
<div>
<img width=500px src="https://github.com/AdrianCobo/my_bot/blob/main/imgs/test/lidar_test.jpg" alt="explode"></a>
</div>

## My bot Mk2 camera
<div>
<img width=500px src="https://github.com/AdrianCobo/my_bot/blob/main/imgs/test/camera_test.jpg" alt="explode"></a>
</div>


# Dificultades encontradas:
- Actualización de paquetes de Ros.
- Descubrimiento de nodos de Ros en diferentes maquinas por wifi.
- Impresión 3D.
- Falta de documentación de la medida de los taladros de varios de los componentes (Camara, etc).
- Paralelo de las ruedas.
- Precisión y exactitud de los encoders.

# Referencias
- [Canal de Josh Newans](https://www.youtube.com/@ArticulatedRobotics)
- [Gazebo](https://gazebosim.org/home)
- [Xacro](https://wiki.ros.org/xacro)
- [Urdf](https://wiki.ros.org/urdf)
- [Tf](https://wiki.ros.org/tf)
- [Teleop_twist_keyboard](https://wiki.ros.org/teleop_twist_keyboard)
- [teleop_twist_joy](https://wiki.ros.org/teleop_twist_joy)
- [Lidar](https://www.slamtec.com/en/lidar/a1)
- [Joy](https://wiki.ros.org/joy)
- [Slam_toolbox](https://wiki.ros.org/slam_toolbox)
- [Nav2](https://navigation.ros.org/)
- [Plansys2](https://plansys2.github.io/)
