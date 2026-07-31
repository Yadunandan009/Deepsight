import os

# 1. FIX THE TANK FILE (Defines 'sand' look)
tank_xml = """<?xml version="1.0"?>
<scenario>
    <environment>
        <ned latitude="56.136459" longitude="-2.706819"/>
        <ocean>
            <water density="1031.0" jerlov="0.15"/>
            <waves height="0.0"/>
            <particles enabled="true"/>
            <current type="uniform">
                <velocity xyz="0.0 0.0 0.0"/>
            </current>
            <current type="jet">
                <center xyz="0.0 0.0 0.0"/>
                <outlet radius="100.0"/>
                <velocity xyz="0.0 0.0 0.0"/>
            </current>
        </ocean>
        <atmosphere>
            <sun azimuth="45.0" elevation="90.0"/>
        </atmosphere>
    </environment>

    <materials>
        <material name="Neutral" density="1000.0" restitution="0.1"/>
        <material name="Rock" density="3000.0" restitution="0.8"/>
        <material name="Fiberglass" density="1500.0" restitution="0.3"/>
        <material name="Aluminium" density="2710.0" restitution="0.5"/>
        <friction_table>
            <friction material1="Neutral" material2="Neutral" static="0.5" dynamic="0.2"/>
            <friction material1="Neutral" material2="Rock" static="0.2" dynamic="0.1"/>
            <friction material1="Neutral" material2="Fiberglass" static="0.5" dynamic="0.2"/>
            <friction material1="Neutral" material2="Aluminium" static="0.1" dynamic="0.02"/>
            <friction material1="Rock" material2="Rock" static="0.9" dynamic="0.7"/>
            <friction material1="Rock" material2="Fiberglass" static="0.6" dynamic="0.4"/>
            <friction material1="Rock" material2="Aluminium" static="0.6" dynamic="0.3"/>
            <friction material1="Fiberglass" material2="Fiberglass" static="0.5" dynamic="0.2"/>
            <friction material1="Fiberglass" material2="Aluminium" static="0.5" dynamic="0.2"/>
            <friction material1="Aluminium" material2="Aluminium" static="0.8" dynamic="0.5"/>
        </friction_table>
    </materials>

    <looks>
        <look name="black" gray="0.05" roughness="0.2"/>
        <look name="yellow" rgb="1.0 0.9 0.0" roughness="0.3"/>
        <look name="gray" gray="0.5" roughness="0.4" metalness="0.5"/>
        <look name="ltgray" gray="0.75" roughness="1.0" metalness="0.0"/>
        <look name="tank" rgb="0.9 0.9 0.9" roughness="0.9"/>
        <look name="duct" gray="0.1" roughness="0.4" metalness="0.5"/>
        <look name="sand" rgb="0.9 0.8 0.6" roughness="0.8"/> 
    </looks>

    <static name="Seabed" type="model">
        <physical>
            <mesh filename="flat.obj" scale="1.0"/>
            <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
        </physical>
        <material name="Rock"/>
        <look name="sand"/>
        <world_transform rpy="0.0 0.0 0.0" xyz="0.0 0.0 6.0"/>
    </static>

    <include file="$(find stonefish_bluerov2)/scenarios/bluerov2.scn">
        <arg name="vehicle_name" value="bluerov2"/>
        <arg name="position" value="0.0 0.0 0.5"/>
        <arg name="orientation" value="0.0 0.0 0.0"/>
    </include>
</scenario>
"""

# 2. FIX THE ROBOT FILE (Adds missing limits and blue look)
robot_xml = """<?xml version="1.0"?>
<scenario>
    <looks>
        <look name="br2" gray="1.0" roughness="0.4" metalness="0.5" texture="bluerov2/br2.png"/>
        <look name="blue" rgb="0.0 0.5 1.0" roughness="0.3"/>
        <look name="black" gray="0.05" roughness="0.5"/>
        <look name="gray" gray="0.5" roughness="0.5"/>
    </looks>

    <robot name="$(arg vehicle_name)" fixed="false" self_collisions="false">
        <base_link name="base_link" type="compound" physics="submerged">
            <external_part name="HullBottom" type="model" physics="submerged" buoyant="false">
                <physical>
                    <mesh filename="bluerov2/bluerov2_phy.obj" scale="1"/>
                    <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
                    <thickness value="0.005"/>
                </physical>
                <visual>
                    <mesh filename="bluerov2/bluerov2.obj" scale="1"/>
                    <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
                </visual>
                <material name="Fiberglass"/>
                <look name="br2"/>
                <compound_transform rpy="0 0.0 0.0" xyz="-0.0 0.0 0.0"/>
            </external_part>

            <external_part name="HeavyFit" type="model" physics="submerged" buoyant="false">
                <physical>
                    <mesh filename="bluerov2/bluerov2_ring.obj" scale="1"/>
                    <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.03"/>
                    <thickness value="0.005"/>
                </physical>
                <visual>
                    <mesh filename="bluerov2/bluerov2_wings.obj" scale="1"/>
                    <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
                </visual>
                <material name="Fiberglass"/>
                <look name="br2"/>
                <compound_transform rpy="0 0.0 0.0" xyz="-0.0 0.0 0.0"/>
            </external_part>

            <internal_part name="BackLeft" type="box" physics="submerged" buoyant="true">
                <dimensions xyz="0.2 0.15 0.091"/>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                <material name="Neutral"/>
                <look name="blue"/>
                <mass value="0.025"/>
                <compound_transform rpy="0.0 0.0 0.0" xyz="-0.1 -0.1 0.0"/>
            </internal_part>

            <internal_part name="BackRight" type="box" physics="submerged" buoyant="true">
                <dimensions xyz="0.2 0.15 0.091"/>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                <material name="Neutral"/>
                <look name="blue"/>
                <mass value="0.025"/>
                <compound_transform rpy="0.0 0.0 0.0" xyz="-0.1 0.1 0.0"/>
            </internal_part>

            <internal_part name="FrontLeft" type="box" physics="submerged" buoyant="true">
                <dimensions xyz="0.2 0.15 0.091"/>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                <material name="Neutral"/>
                <look name="blue"/>
                <mass value="0.025"/>
                <compound_transform rpy="0.0 0.0 0.0" xyz="0.09 -0.1 0.0"/>
            </internal_part>

            <internal_part name="FrontRight" type="box" physics="submerged" buoyant="true">
                <dimensions xyz="0.2 0.15 0.091"/>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                <material name="Neutral"/>
                <look name="blue"/>
                <mass value="0.025"/>
                <compound_transform rpy="0.0 0.0 0.0" xyz="0.09 0.1 0.0"/>
            </internal_part>

            <internal_part name="WeightCenter" type="sphere" physics="submerged" buoyant="false">
                <dimensions radius="0.01"/>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                <material name="Steel"/>
                <look name="blue"/>
                <mass value="2"/>
                <compound_transform rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.1"/>
            </internal_part>

            <internal_part name="WeightLeft" type="sphere" physics="submerged" buoyant="false">
                <dimensions radius="0.01"/>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                <material name="Steel"/>
                <look name="blue"/>
                <mass value="1"/>
                <compound_transform rpy="0.0 0.0 0.0" xyz="0.0 -0.075 0.1"/>
            </internal_part>

            <internal_part name="WeightRight" type="sphere" physics="submerged" buoyant="false">
                <dimensions radius="0.01"/>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                <material name="Steel"/>
                <look name="blue"/>
                <mass value="1"/>
                <compound_transform rpy="0.0 0.0 0.0" xyz="0.0 0.075 0.1"/>
            </internal_part>
        </base_link>
        
        <actuator name="FrontRight" type="thruster">
            <link name="base_link"/>
            <origin xyz="0.1355 0.1 0.0725" rpy="0 0 -0.7853981634"/>
            <specs thrust_coeff="0.167" torque_coeff="0.016" max_rpm="3600.0" inverted="true"/>
            <limits min="-1.0" max="1.0"/>
            <propeller diameter="0.076" right="true">
                <mesh filename="bluerov2/ccw.obj" scale="1.0"/>
                <material name="Neutral"/>
                <look name="blue"/>
            </propeller>
        </actuator>

        <actuator name="FrontLeft" type="thruster">
            <link name="base_link"/>
            <origin xyz="0.1355 -0.1 0.0725" rpy="0 0 0.7853981634"/>
            <specs thrust_coeff="0.167" torque_coeff="0.016" max_rpm="3600.0" inverted="true"/>
            <limits min="-1.0" max="1.0"/>
            <propeller diameter="0.076" right="true">
                <mesh filename="bluerov2/ccw.obj" scale="1.0"/>
                <material name="Neutral"/>
                <look name="blue"/>
            </propeller>
        </actuator>

        <actuator name="BackRight" type="thruster">
            <link name="base_link"/>
            <origin xyz="-0.1475 0.1 0.0725" rpy="0 0 -2.3561944902"/>
            <specs thrust_coeff="0.167" torque_coeff="0.016" max_rpm="3600.0" inverted="false"/>
            <limits min="-1.0" max="1.0"/>
            <propeller diameter="0.076" right="false">
                <mesh filename="bluerov2/cw.obj" scale="1.0"/>
                <material name="Neutral"/>
                <look name="blue"/>
            </propeller>
        </actuator>

        <actuator name="BackLeft" type="thruster">
            <link name="base_link"/>
            <origin xyz="-0.1475 -0.1 0.0725" rpy="0 0 2.3561944902"/>
            <specs thrust_coeff="0.167" torque_coeff="0.016" max_rpm="3600.0" inverted="false"/>
            <limits min="-1.0" max="1.0"/>
            <propeller diameter="0.076" right="false">
                <mesh filename="bluerov2/cw.obj" scale="1.0"/>
                <material name="Neutral"/>
                <look name="blue"/>
            </propeller>
        </actuator>

        <actuator name="DiveFrontRight" type="thruster">
            <link name="base_link"/>
            <origin xyz="0.12 0.218 0.0" rpy="0 -1.5707963268 0"/>
            <specs thrust_coeff="0.167" torque_coeff="0.016" max_rpm="3600.0" inverted="false"/>
            <limits min="-1.0" max="1.0"/>
            <propeller diameter="0.076" right="true">
                <mesh filename="bluerov2/cw.obj" scale="1.0"/>
                <material name="Neutral"/>
                <look name="blue"/>
            </propeller>
        </actuator>

        <actuator name="DiveFrontLeft" type="thruster">
            <link name="base_link"/>
            <origin xyz="0.12 -0.218 0.0" rpy="0 -1.5707963268 0"/>
            <specs thrust_coeff="0.167" torque_coeff="0.016" max_rpm="3600.0" inverted="true"/>
            <limits min="-1.0" max="1.0"/>
            <propeller diameter="0.076" right="false">
                <mesh filename="bluerov2/ccw.obj" scale="1.0"/>
                <material name="Neutral"/>
                <look name="blue"/>
            </propeller>
        </actuator>

        <actuator name="DiveBackRight" type="thruster">
            <link name="base_link"/>
            <origin xyz="-0.12 0.218 0.0" rpy="0 -1.5707963268 0"/>
            <specs thrust_coeff="0.167" torque_coeff="0.016" max_rpm="3600.0" inverted="true"/>
            <limits min="-1.0" max="1.0"/>
            <propeller diameter="0.076" right="false">
                <mesh filename="bluerov2/ccw.obj" scale="1.0"/>
                <material name="Neutral"/>
                <look name="blue"/>
            </propeller>
        </actuator>

        <actuator name="DiveBackLeft" type="thruster">
            <link name="base_link"/>
            <origin xyz="-0.12 -0.218 0.0" rpy="0 -1.5707963268 0"/>
            <specs thrust_coeff="0.167" torque_coeff="0.016" max_rpm="3600.0" inverted="false"/>
            <limits min="-1.0" max="1.0"/>
            <propeller diameter="0.076" right="true">
                <mesh filename="bluerov2/cw.obj" scale="1.0"/>
                <material name="Neutral"/>
                <look name="blue"/>
            </propeller>
        </actuator>

        <sensor name="odometry" type="odometry" rate="100.0">
            <link name="base_link"/>
            <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
            <ros_publisher topic="/$(arg vehicle_name)/odometry"/>
        </sensor>

        <sensor name="INS" rate="100.0" type="ins">
            <output_frame rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
            <noise angular_velocity="0.00001745" linear_acceleration="0.00005"/>
            <external_sensors dvl="dvl" gps="gps" pressure="pressure"/>
            <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
            <link name="base_link"/>
            <ros_publisher topic="/$(arg vehicle_name)/INS"/>
        </sensor>

        <sensor name="pressure" type="pressure" rate="5.0">
            <link name="base_link"/>
            <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
            <noise pressure="5.0"/>
            <ros_publisher topic="/$(arg vehicle_name)/pressure"/>
        </sensor>

        <sensor name="imu_filter" type="imu" rate="20.0">
            <link name="base_link"/>
            <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
            <noise angle="0.000001745" angular_velocity="0.00001745"/>
            <ros_publisher topic="/$(arg vehicle_name)/imu"/>
        </sensor>

        <sensor name="gps" type="gps" rate="1.0">
            <link name="base_link"/>
            <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 -0.15"/>
            <noise ned_position="0.5"/>
            <ros_publisher topic="/$(arg vehicle_name)/gps"/>
        </sensor>

        <sensor name="dvl" type="dvl" rate="5.0">
            <link name="base_link"/>
            <origin rpy="3.1416 0.0 0.0" xyz="0.0 0.0 0.0"/>
            <specs beam_angle="30.0"/>
            <range velocity="9.0 9.0 9.0" altitude_min="0.2" altitude_max="100.0"/>
            <noise velocity="0.0015" altitude="0.001"/>
            <ros_publisher topic="/$(arg vehicle_name)/dvl_sim" altitude_topic="/$(arg vehicle_name)/altitude"/>
        </sensor>

        <sensor name="camera_front" type="camera" rate="10.0">
            <link name="base_link"/>
            <origin rpy="1.2 0.0 1.571" xyz="0.3 0.0 0.0"/>
            <specs resolution_x="800" resolution_y="600" horizontal_fov="55.0"/>
            <ros_publisher topic="/$(arg vehicle_name)/camera"/>
        </sensor>

        <sensor name="camera_left" rate="30.0" type="camera">
            <specs resolution_x="640" resolution_y="480" horizontal_fov="75.0"/>
            <origin xyz="0.16 -0.0725 0.15" rpy="1.571 0.0 1.571"/>
            <link name="base_link"/>
            <ros_publisher topic="/$(arg vehicle_name)/left"/>
        </sensor>
        
        <sensor name="camera_right" rate="30.0" type="camera">
            <specs resolution_x="640" resolution_y="480" horizontal_fov="75.0" baseline="-60.5"/>
            <origin xyz="0.16 0.0725 0.15" rpy="1.571 0.0 1.571"/>
            <link name="base_link"/>
            <ros_publisher topic="/$(arg vehicle_name)/right"/>
        </sensor>

        <sensor name="multibeam" rate="10.0" type="multibeam">
            <specs fov="90.0" steps="128"/>
            <range distance_min="0.25" distance_max="50.0"/>
            <noise distance="0.1"/>
            <history samples="1"/>
            <origin xyz="0.2 0.0 0.2" rpy="0.0 -0.0 0.0"/>
            <link name="base_link"/>
            <ros_publisher topic="/$(arg vehicle_name)/multibeam"/>
        </sensor>

        <world_transform xyz="$(arg position)" rpy="$(arg orientation)"/>
        <ros_subscriber thrusters="/$(arg vehicle_name)/setpoint/pwm"/>
    </robot>
</scenario>
"""

path_tank = os.path.expanduser("~/ros2_ws/src/stonefish_bluerov2/scenarios/bluerov2_tank.scn")
path_robot = os.path.expanduser("~/ros2_ws/src/stonefish_bluerov2/scenarios/bluerov2.scn")

try:
    with open(path_tank, "w") as f: f.write(tank_xml)
    print(f"FIXED: {path_tank}")
    with open(path_robot, "w") as f: f.write(robot_xml)
    print(f"FIXED: {path_robot}")
except Exception as e:
    print(f"ERROR: {e}")
