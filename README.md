# My bot package

[Project based on](https://www.youtube.com/@ArticulatedRobotics/videos)

## Installation:

---

```console
    sudo apt-get install ros-humble-gazebo-ros-pkgs ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-twist-mux ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control ros-humble-slam-toolbox
    cd ~/your_ws/src
    git clone (this repo)
    git clone https://github.com/AdrianCobo/Pal_Gazebo_Worlds.git
    colcon build --symlink-install
```

---

## Launch examples:

### Gazebo Simulation and joystick control:

1. change world_name at config/map_params.yaml
2.

```console
    ros2 launch my_bot launch_sim.launch.py
```

You can see the video demonstration here: [(Youtube)](https://youtu.be/H0Chc4LrjQw)

### Gazebo Simulation and keyboard control:

1. change world_name at config/map_params.yaml
2.

```console
    ros2 launch my_bot launch_sim.launch.py
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

You can see the video demonstration here: [(Youtube)](https://youtu.be/-zVjHXezQI8)

### Gazebo Simulation and Mapping

```console
    ros2 launch my_bot launch_sim.launch.py
    cd ~/your_ws
    ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/my_bot/config/mapper_params_online_async.yaml use_sim_time:=true
    cd ~/your_ws
    rviz2 -d src/my_bot/robot_view.rviz
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

You can see the video demonstration here: [(Youtube)](https://youtu.be/yNdHQ1RQuCk)

### Gazebo Simulation and nav2 localization

```console
    ros2 launch my_bot launch_sim.launch.py
    cd ~/your_ws
    rviz2 -d src/my_bot/robot_view.rviz
    cd ~/your_ws
    ros2 launch nav2_bringup localization_launch.py map:=./src/my_bot/maps/home.yaml use_sim_time:=true
    # publish a 2D point using rviz at your robot map position
    ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true map_subscribe_transient_local:=true
```

You can see the video demonstration here: [(Youtube)](https://youtu.be/tctQYJnHBAQ)

## Bill

- [CAMARA OFICIAL 8MPX V2 PARA RASPBERRY PI (27.95)€](https://www.tiendatec.es/raspberry-pi/camaras/236-camara-oficial-8mpx-v2-para-raspberry-pi-652508442112.html)

- [RASPBERRY PI 4 - MODELO B - 4GB (78.95€)](https://www.tiendatec.es/raspberry-pi/gama-raspberry-pi/1100-raspberry-pi-4-modelo-b-4gb-765756931182.html)

- [DISIPADOR INTEGRAL CON VENTILADOR CONTROLADO POR SOFTWARE - PWM(9.95 €)](https://www.tiendatec.es/raspberry-pi/accesorios/1980-disipador-integral-con-ventilador-controlado-por-software-pwm-8472496024945.html)

- Gastos de envio de lo anterior(3.95€)

- [Micro-ssd 64GB(10€)](https://www.elcorteingles.es/search/?s=microssd&hierarchy=&deep_search=&stype=text_box)

- Cable usb a usbc (3.9€): Alcampo

- Cable HDMI a micro-usb(9.77€): Alcampo

- [Slamtec RPLIDAR A1 2D 360 grados 12 metros(119.99€)](https://www.amazon.es/Slamtec-RPLIDAR-esc%C3%A1ner-bst%C3%A1culos-navegaci%C3%B3n/dp/B07TJW5SXF)

- [ZOP POWER 11.1V 2700mAh 30C 3S LiPo Battery XT60 Plug(20.24€ + ShippingFee:7.96€ = 28.60)](https://usa.banggood.com/ZOP-POWER-11_1V-2700mAh-30C-3S-LiPo-Battery-XT60-Plug-for-RC-Drone-p-1984639.html?cur_warehouse=CN)

- [Multimeter AC/DC Votage Current...(9.19€)](https://usa.banggood.com/ANENG-SZ302-Digital-Multimeter-AC-or-DC-Votage-Current-Automatic-Tester-NCV-Detector-Resistance-Ohm-Ammeter-Capacitance-Meter-p-1975571.html?cur_warehouse=CN&ID=6287830)

- [80W 6A Lipo Battery Balance Charger with Power Supply Adapter(25.52€)](https://usa.banggood.com/index.php?com=account&t=ordersDetail&ordersId=112329387&version=2&status=0)

- [Conectores de bala XT60 XT-60 macho y hembra, cable de silicona de 14 AWG (1.48€)](https://es.aliexpress.com/item/1005003658392996.html?spm=a2g0o.order_list.order_list_main.5.192e194dBpok7U&gatewayAdapt=glo2esp)

- [FATJAY-conector T a JST, adaptador de conversión macho y hembra, cable de 18AWG(2.15€)](https://es.aliexpress.com/item/32912278147.html?spm=a2g0o.order_list.order_list_main.10.192e194dBpok7U&gatewayAdapt=glo2esp)

- [Caja de fusibles impermeable para coche, portafusibles estándar(2.11€)](https://es.aliexpress.com/item/1005003617756967.html?spm=a2g0o.order_list.order_list_main.15.192e194dBpok7U&gatewayAdapt=glo2esp)

- [Lote de 10 conectores macho y hembra Jack 12V(0.99€)](https://es.aliexpress.com/item/1005004369381250.html?spm=a2g0o.order_list.order_list_main.20.192e194dBpok7U&gatewayAdapt=glo2esp)

- [Interruptor basculante KCD1, interruptor de encendido de 3 pines con luz... (0.72€)](https://es.aliexpress.com/item/32957731774.html?spm=a2g0o.order_list.order_list_main.25.192e194dBpok7U&gatewayAdapt=glo2esp)

- [Arduino uno (29.28€)](https://www.amazon.es/Arduino-UNO-A000066-microcontrolador-ATmega328/dp/B008GRTSV6/ref=sr_1_1_sspa?keywords=arduino+uno&qid=1688576294&s=electronics&sr=1-1-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&psc=1)

- [Controlador L298N Motores DC PAP Driver Stepper Doble puente H (3.99€)](https://www.amazon.es/gp/product/B07YNR5KWP/ref=ppx_od_dt_b_asin_title_s00?ie=UTF8&psc=1)

- [Cable Eléctrico 18 AWG (7,99€)](https://www.amazon.es/gp/product/B075M4VJ8J/ref=ppx_od_dt_b_asin_title_s01?ie=UTF8&psc=1)

- [2x DC 12V DIY Encoder Motorreductor Con Soporte de Montaje (24.74€)](https://www.amazon.es/gp/product/B07WT22RNJ/ref=ppx_od_dt_b_asin_title_s00?ie=UTF8&psc=1)

- [Hub USB 3.0 Alimentado (22.99€)](https://www.amazon.es/gp/product/B0BCPB7HQX/ref=ppx_od_dt_b_asin_title_s00?ie=UTF8&psc=1)

- [20Pcs Bloques de Terminales Eléctricos Regleta Electrica (14.99€)](https://www.amazon.es/gp/product/B092RZR3LX/ref=ppx_od_dt_b_asin_title_s00?ie=UTF8&psc=1)

- Gastos de envio de los 4 ultimos productos(3.29€), descuento de (-3.99€) e impuestos(17.26€)

- [df-dfr0205 convertidor Step Down (21,27€)](https://www.amazon.es/gp/product/B07FS7SLCB/ref=ppx_od_dt_b_asin_title_s00?ie=UTF8&psc=1)

Total actual: 468,45€
