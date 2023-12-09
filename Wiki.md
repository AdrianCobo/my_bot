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

<video src='https://github.com/AdrianCobo/my_bot/blob/main/animate/tfs_demo.mp4' width=180/>

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


# Fase 6: Sensores y actuadores reales

Una vez ya completado las pruebas de simulación, se decidio comprar los componentes necesariós para poder testear el circuito del robot. 

Lo mas complicado fue la comunicación de los sensores y actuadores con ros2 en concreto con el controlador de motores y el arduino ya que hacía falta crear un bridge entre ros2 y arduino o usar uno ya hecho [Opción elegida](https://github.com/joshnewans/serial_motor_demo). También usamos [codigo de Buzzology](https://github.com/AdrianCobo/diffdrive_arduino.git) para el control diferencial de los motores con arduino.

Finalmente para el lidar se compró uno que era comptaible con ros2 y que tenia ya sus propios nodos y launchers de ros2.

Puedes ver la lista con todos los componentes y precios [aquí](https://github.com/AdrianCobo/my_bot/blob/readme_updated/Bill.md)

Puedes ver un video de demostración aqui: [(Youtube)](https://youtu.be/Q_-EYw8jdps?si=o-zX6n-kzWKuK6uY)

[![Alt text](https://img.youtube.com/vi/Q_-EYw8jdps/0.jpg)](https://www.youtube.com/watch?v=Q_-EYw8jdps)

Recuerda también que tienes todos los pasos para la correcta puesta a punto y lanzamiento del entorno de simulación o de los nodos necesariós para ejecutar el codigo en un robot real parecido.

# Referencias
- [Canal de Josh Newans](https://www.youtube.com/@ArticulatedRobotics)
- [Plansys2](https://plansys2.github.io/)
