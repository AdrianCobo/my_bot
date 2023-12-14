# My bot package

Proyecto basado en el canal de youtube de [Articulated Robotics](https://www.youtube.com/@ArticulatedRobotics/videos)

## My bot Mk1

<div align="center">
<img width=500px src="https://github.com/AdrianCobo/my_bot/blob/readme_updated/imgs/my_bot_mk1_real.jpg" alt="explode"></a>
</div>
<div align="center">
<img width=500px src="https://github.com/AdrianCobo/my_bot/blob/readme_updated/imgs/my_bot_mk1_sim.png" alt="explode"></a>
</div>


# Objetivo General  
Este proyecto tiene como objetivo desarrollar y simular un robot diferencial del tamaño de una caja de zapatos utilizando las capacidades de ROS 2 para la simulación y control de robots. A lo largo de las fases del proyecto, se explorarán diversas tecnologías y conceptos esenciales en robótica.

# Fase 1: Simulación

## Fase 1.1: Uso de Tfs
La primera aproximación elegida fue entender como funcionaba el sistema de transformadas en ros para poder verfificar el correcto comportamiento durante la simulación de nuestro robot a traves del paquete tf2

https://github.com/AdrianCobo/my_bot/assets/98641977/d725b8a4-27a4-4976-9d00-0e46af1a7a92

## Fase 1.2: Diseño de Robot a partir de URDF
Una vez entendido como funcionan los sistemas de transformadas para las articulaciones del robot, se decidió modelar el robot utilizando el formato URDF para que fuese compatible con gazebo. Para ayudarnos en esta tarea se decidió utilizar también Xacro facilitando muchisimo esta tarea.

## Fase 1.3: Gazebo
Una vez que estaba listo el fichero URDF, se le añadieron los plugins necesarios para que Gazebo lo pudiese entender y se decidió crear ficheros .world y launcher para facilitar el lanzamiento de la simulación.

```console
    ros2 launch my_bot launch_real_sim.launch.py
```

Puedes ver el video de demostración aqui: [(Youtube)](https://youtu.be/H0Chc4LrjQw)

[![Alt text](https://img.youtube.com/vi/H0Chc4LrjQw/0.jpg)](https://www.youtube.com/watch?v=H0Chc4LrjQw)

# Fase 2: Simulación de sensores y actuadores

Una vez que ya se tenia la simulacíon del chasis se tuvo como objetivo la simulación de sensores (camara, camara con rgbd y lidar) así como el controlador diferencial. Todo esto fue bastante facil de hacer debido a los plugins ofrecidos por gazebo para la correcta simulacion. Simplemente teniamos que añadir estos plugins usando xacro y generando el urdf que usaría gazebo para la correcta simulación. 

Para comprobar que todo funcionaba correctamente se utilizo rviz2 para ver los datos sensados y el paquete teleop_twist_keyboard para comandar velocidades al controlador de motores.

Estos ficheros se pueden ver en:
- [Caramara](https://github.com/AdrianCobo/my_bot/blob/main/description/camera.xacro)
- [Caramara RGBD](https://github.com/AdrianCobo/my_bot/blob/main/description/depth_camera.xacro)
- [Lidar](https://github.com/AdrianCobo/my_bot/blob/main/description/lidar.xacro)
- [Controlador diferencial de Gazebo](https://github.com/AdrianCobo/my_bot/blob/main/description/gazebo_control.xacro)

```console
    ros2 launch my_bot launch_real_sim.launch.py
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Puedes ver un video de demostración aqui: [(Youtube)](https://youtu.be/-zVjHXezQI8)

[![Alt text](https://img.youtube.com/vi/-zVjHXezQI8/0.jpg)](https://www.youtube.com/watch?v=-zVjHXezQI8)

# Fase 3: Control por Joystick y uso de ros2_control.

Aunque el controlador diferencial facilitado por gazebo funciona bien, como se quería que el proyecto no solo fuese un robot simulado si no que se llevase a la realidad, se decidió usar el paquete estandard de ros para el control de motores (ros2_control) ya que luego nos facilitaria mucho la vida para comunicarnos con los motores y encoders del robot real y además el controlador es bastante mejor.

Como en los casos anteriores, simplemente teniamos que configurar bien en un archivo .xacro los pluggins necesarios para emplear este paquete pudiendose ver el fichero [aquí](https://github.com/AdrianCobo/my_bot/blob/main/description/ros2_control.xacro)

Tambien como el control con teleop_twist_keyboard a veces puede ser un poco incomodo, se decidió implementar tambien un control a traves de un mando de play 4 para ello, se usó el paquete de ros2 joy el cual combierte los valores analogicos del joystick a un mensaje de ros2 de tipo joy y el paquete teleop_twist_joy que combierte los mensajes de tipo joy a tipo cmd que son los mensajes que entiende el controlador de motores.

*El launcher de la simulacion también lanza los nodos del joystick

# Fase 4: Mapeado y navegación autónoma.

Para esta fase se decidió usar los paquetes mas populares de ros2 para ambos temas. Para la construcción de mapas se usó el paquete slam_toolbox y para navegación autonoma nav2. 

Para que funcionase correctamente simplemente teniamos que instalar los paquetes necesarios y completar los ficheros de configuración de ambos paquetes para que se cumpliesen nuestros requerimientos. Ademas slam_toolbox ofrece un panel para rviz2 que facilita mucho el almacenamiento de los mapas generados.

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

Se pensó también que era buena idea pensar algún uso real que se le podría dar a este robotito y como por aquel entonces la practica propuesta para la asignatura de Planificación y robótica de serviciós era la de un robot de patrullaje se decidio probar a realiza este ejercicio usando el paquete de plansys2 junto con las tecnicas de mapeado y navegación autonoma explicadas antes.

Puedes ver un video de demostración aqui: [(Youtube)](https://youtu.be/MbuiRqzs0qQ)

[![Alt text](https://img.youtube.com/vi/MbuiRqzs0qQ/0.jpg)](https://www.youtube.com/watch?v=Q_-EYw8jdps)

# Fase 6: Diseño 2D

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

Una vez ya completado las pruebas de simulación, se decidio comprar los componentes necesariós para poder testear el circuito del robot. 

Lo mas complicado fue la comunicación de los sensores y actuadores con ros2 en concreto con el controlador de motores y el arduino ya que hacía falta crear un bridge entre ros2 y arduino o usar uno ya hecho [Opción elegida](https://github.com/joshnewans/serial_motor_demo). También usamos [codigo de Buzzology](https://github.com/AdrianCobo/diffdrive_arduino.git) para el control diferencial de los motores con arduino.

Finalmente para el lidar se compró uno que era comptaible con ros2 y que tenia ya sus propios nodos y launchers de ros2.

Puedes ver la lista con todos los componentes y precios [aquí](https://github.com/AdrianCobo/my_bot/blob/readme_updated/Bill.md)

Puedes ver un video de demostración aqui: [(Youtube)](https://youtu.be/Q_-EYw8jdps?si=o-zX6n-kzWKuK6uY)

[![Alt text](https://img.youtube.com/vi/Q_-EYw8jdps/0.jpg)](https://www.youtube.com/watch?v=Q_-EYw8jdps)

Recuerda también que tienes todos los pasos para la correcta puesta a punto y lanzamiento del entorno de simulación o de los nodos necesariós para ejecutar el codigo en un robot real parecido.

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
