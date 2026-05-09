#!/usr/bin/env python3
"""
Build new model.sdf for tracked_robot from the Fusion export.

Reads:  GazeboTank3SDFogURDF/GazeboTank3SDFogURDF/Gazebo/Gazebo.sdf
Writes: src/robot_gz/models/tracked_robot/model.sdf

Changes applied vs. raw Fusion export:
  1. Model name: Gazebo -> tracked_robot
  2. Fix mesh URIs: model://Gazebo/meshes/ -> meshes/
  3. Add gz namespace for sensor frame_id
  4. Add base_link (root ROS link) with yaw=+pi/2 in model frame
  5. Add base_link_to_Chassis fixed joint
  6. Rename arm/gripper joints to match existing plugin/bridge config
  7. Add spring/damping + effort/velocity limits to 4 leg joints
  8. Replace track mesh collision with TrackedVehicle drive proxy box (fdir1=0 1 0)
  9. Add camera/IMU/LiDAR sensors to Chassis link
 10. Add sensor frame definitions (camera_link, imu_link, lidar_link)
 11. Add all Gazebo plugins (TrackedVehicle, TrackController x2,
     JointPositionController x4, JointStatePublisher, PosePublisher)

Orientation strategy (fixes sideways driving):
  base_link has yaw=+pi/2 in model frame.
  World spawn yaw must be set to -pi/2 in tracked_robot_world.sdf.
  Result: base_link world yaw=0, forward=base_link X = model Y = track fdir1 direction.
"""

import os
import sys

SRC = "GazeboTank3SDFogURDF/GazeboTank3SDFogURDF/Gazebo/Gazebo.sdf"
DST = "src/robot_gz/models/tracked_robot/model.sdf"

# ---------------------------------------------------------------------------
# Drive proxy constants (positions in track link-local frame)
# Spawn Z = 0.016916, track pose Z ≈ -0.000145
# proxy_Z = 0.0035 - 0.016916 + 0.000145 = -0.013271
# ---------------------------------------------------------------------------
PROXY_X = 0.0912
PROXY_Y = 0.018
PROXY_Z = -0.013271
PROXY_SIZE = "0.030 0.210 0.007"

def build():
    if not os.path.exists(SRC):
        print(f"ERROR: {SRC} not found. Run from workspace root.", file=sys.stderr)
        sys.exit(1)

    with open(SRC, "r") as f:
        c = f.read()

    # ------------------------------------------------------------------
    # 1. SDF version + gz namespace
    # ------------------------------------------------------------------
    c = c.replace(
        '<sdf version="1.7">',
        '<sdf version="1.7" xmlns:gz="http://gazebosim.org/schema">'
    )

    # ------------------------------------------------------------------
    # 2. Model name
    # ------------------------------------------------------------------
    c = c.replace('<model name="Gazebo">', '<model name="tracked_robot">')

    # ------------------------------------------------------------------
    # 3. Mesh URIs
    # ------------------------------------------------------------------
    c = c.replace("model://Gazebo/meshes/", "meshes/")

    # ------------------------------------------------------------------
    # 4. Rename arm / gripper joints
    # ------------------------------------------------------------------
    c = c.replace('name="RLeadscrew_Slider-22"',          'name="robotarm_z_joint"')
    c = c.replace('name="Leadscrew-Rigg_Slider-29"',      'name="robotarm_x_joint"')
    c = c.replace('name="Gripper_Revolute-35"',            'name="right_gripper_finger_joint"')
    c = c.replace('name="Gripper_Revolute-36"',            'name="left_gripper_finger_joint"')

    # ------------------------------------------------------------------
    # 5. Add base_link right after <model name="tracked_robot">
    # ------------------------------------------------------------------
    BASE_LINK_XML = """
        <!-- Root link for ROS TF. yaw=+pi/2 so base_link X = model Y = track forward direction.
             World spawn must use yaw=-pi/2 to cancel: base_link world yaw = 0. -->
        <link name="base_link">
            <pose>0 0 0 0 0 1.5707963267948972</pose>
        </link>"""
    c = c.replace(
        '<model name="tracked_robot">',
        '<model name="tracked_robot">' + BASE_LINK_XML
    )

    # ------------------------------------------------------------------
    # 6. Replace LTracks mesh collision with drive proxy box
    # ------------------------------------------------------------------
    LTRACKS_OLD = """\
            <collision name="LTracks_collision">
                <geometry>
                    <mesh>
                        <uri>meshes/LTracks.stl</uri>
                        <scale>0.001 0.001 0.001</scale>
                    </mesh>
                </geometry>
            </collision>"""

    LTRACKS_NEW = f"""\
            <collision name="LTracks_drive_proxy_collision">
                <pose>{PROXY_X} {PROXY_Y} {PROXY_Z} 0 0 0</pose>
                <geometry>
                    <box>
                        <size>{PROXY_SIZE}</size>
                    </box>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>0.7</mu>
                            <mu2>150</mu2>
                            <fdir1>0 1 0</fdir1>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <visual name="LTracks_drive_proxy_visual">
                <pose>{PROXY_X} {PROXY_Y} {PROXY_Z} 0 0 0</pose>
                <geometry>
                    <box>
                        <size>{PROXY_SIZE}</size>
                    </box>
                </geometry>
                <transparency>0.25</transparency>
                <material>
                    <ambient>0.0 0.25 1.0 1</ambient>
                    <diffuse>0.0 0.25 1.0 1</diffuse>
                </material>
            </visual>"""

    if LTRACKS_OLD not in c:
        print("WARNING: LTracks collision pattern not found. Skipping replacement.")
    else:
        c = c.replace(LTRACKS_OLD, LTRACKS_NEW)

    # ------------------------------------------------------------------
    # 7. Replace RTracks mesh collision with drive proxy box
    # ------------------------------------------------------------------
    RTRACKS_OLD = """\
            <collision name="RTracks_collision">
                <geometry>
                    <mesh>
                        <uri>meshes/RTracks.stl</uri>
                        <scale>0.001 0.001 0.001</scale>
                    </mesh>
                </geometry>
            </collision>"""

    RTRACKS_NEW = f"""\
            <collision name="RTracks_drive_proxy_collision">
                <pose>{PROXY_X} {PROXY_Y} {PROXY_Z} 0 0 0</pose>
                <geometry>
                    <box>
                        <size>{PROXY_SIZE}</size>
                    </box>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>0.7</mu>
                            <mu2>150</mu2>
                            <fdir1>0 1 0</fdir1>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <visual name="RTracks_drive_proxy_visual">
                <pose>{PROXY_X} {PROXY_Y} {PROXY_Z} 0 0 0</pose>
                <geometry>
                    <box>
                        <size>{PROXY_SIZE}</size>
                    </box>
                </geometry>
                <transparency>0.25</transparency>
                <material>
                    <ambient>0.0 0.8 0.25 1</ambient>
                    <diffuse>0.0 0.8 0.25 1</diffuse>
                </material>
            </visual>"""

    if RTRACKS_OLD not in c:
        print("WARNING: RTracks collision pattern not found. Skipping replacement.")
    else:
        c = c.replace(RTRACKS_OLD, RTRACKS_NEW)

    # ------------------------------------------------------------------
    # 8. Add spring/damping + limits to suspension joints
    # ------------------------------------------------------------------

    def patch_leg_joint(content, joint_name, pose_str, parent, child, axis_xyz,
                        lower, upper, spring_ref):
        old = f"""\
        <joint name="{joint_name}" type="revolute">
            <pose>{pose_str}</pose>
            <parent>{parent}</parent>
            <child>{child}</child>
            <axis>
                <xyz expressed_in="__model__">{axis_xyz}</xyz>
                <limit>
                    <lower>{lower}</lower>
                    <upper>{upper}</upper>
                </limit>
            </axis>
        </joint>"""
        new = f"""\
        <joint name="{joint_name}" type="revolute">
            <pose>{pose_str}</pose>
            <parent>{parent}</parent>
            <child>{child}</child>
            <axis>
                <xyz expressed_in="__model__">{axis_xyz}</xyz>
                <limit>
                    <lower>{lower}</lower>
                    <upper>{upper}</upper>
                    <effort>1000000</effort>
                    <velocity>1000000</velocity>
                </limit>
                <dynamics>
                    <spring_reference>{spring_ref}</spring_reference>
                    <spring_stiffness>350</spring_stiffness>
                    <damping>18</damping>
                </dynamics>
            </axis>
        </joint>"""
        if old not in content:
            print(f"WARNING: pattern for {joint_name} not found. Check whitespace.")
        else:
            content = content.replace(old, new)
        return content

    c = patch_leg_joint(c,
        "Chassis_Revolute-1",
        "-0.07570000000000006 -0.005149000000000186 -0.06997399999999994 0.0 0.0 0.0",
        "Chassis", "BRLegs", "-1.0 -0.0 -0.0",
        "0.0", "0.261799", "0.18")

    c = patch_leg_joint(c,
        "Chassis_Revolute-2",
        "-0.07570000000000009 0.04185699999999977 -0.06997799999999992 0.0 0.0 0.0",
        "Chassis", "FRLegs", "-1.0 -0.0 -0.0",
        "-0.261799", "0.0", "-0.18")

    c = patch_leg_joint(c,
        "Chassis_Revolute-7",
        "-0.0757 0.041857 0.069978 0.0 0.0 0.0",
        "Chassis", "FLLegs", "1.0 0.0 0.0",
        "0.0", "0.261799", "0.18")

    c = patch_leg_joint(c,
        "Chassis_Revolute-8",
        "-0.0757 -0.005149 0.069974 0.0 0.0 0.0",
        "Chassis", "BLLegs", "1.0 0.0 0.0",
        "-0.261799", "0.0", "-0.18")

    # ------------------------------------------------------------------
    # 9. Add sensors to Chassis link (insert before its </link>)
    # Chassis link is the first link; it ends just before <link name="Mega-wsheeld-v5-1">
    # ------------------------------------------------------------------
    SENSORS_XML = """
            <!-- Sensors -->
            <sensor name="camera" type="camera">
                <pose relative_to="camera_link">0 0 0 0 0 0</pose>
                <always_on>true</always_on>
                <update_rate>15</update_rate>
                <visualize>false</visualize>
                <topic>/camera</topic>
                <gz:frame_id>camera_link</gz:frame_id>
                <camera>
                    <horizontal_fov>1.3962634</horizontal_fov>
                    <image>
                        <width>640</width>
                        <height>480</height>
                        <format>R8G8B8</format>
                    </image>
                    <clip>
                        <near>0.02</near>
                        <far>300</far>
                    </clip>
                </camera>
            </sensor>
            <sensor name="imu" type="imu">
                <pose relative_to="imu_link">0 0 0 0 0 0</pose>
                <always_on>true</always_on>
                <update_rate>100</update_rate>
                <topic>/imu/data</topic>
                <gz:frame_id>imu_link</gz:frame_id>
            </sensor>
            <sensor name="lidar" type="gpu_lidar">
                <pose relative_to="lidar_link">0 0 0 0 0 0</pose>
                <always_on>true</always_on>
                <update_rate>10</update_rate>
                <visualize>false</visualize>
                <topic>/lidar</topic>
                <gz:frame_id>base_laser</gz:frame_id>
                <lidar>
                    <scan>
                        <horizontal>
                            <samples>360</samples>
                            <resolution>1</resolution>
                            <min_angle>-3.14159</min_angle>
                            <max_angle>3.14159</max_angle>
                        </horizontal>
                        <vertical>
                            <samples>1</samples>
                            <resolution>1</resolution>
                            <min_angle>0</min_angle>
                            <max_angle>0</max_angle>
                        </vertical>
                    </scan>
                    <range>
                        <min>0.08</min>
                        <max>10.0</max>
                        <resolution>0.01</resolution>
                    </range>
                    <noise>
                        <type>gaussian</type>
                        <mean>0</mean>
                        <stddev>0.01</stddev>
                    </noise>
                </lidar>
            </sensor>"""

    # Chassis link ends with its collision block, then </link>, then Mega link starts
    CHASSIS_LINK_END_OLD = """\
            </collision>
        </link>
        <link name="Mega-wsheeld-v5-1">"""

    CHASSIS_LINK_END_NEW = """\
            </collision>""" + SENSORS_XML + """
        </link>
        <link name="Mega-wsheeld-v5-1">"""

    if CHASSIS_LINK_END_OLD not in c:
        print("WARNING: Chassis link end pattern not found. Sensors NOT inserted.")
    else:
        c = c.replace(CHASSIS_LINK_END_OLD, CHASSIS_LINK_END_NEW)

    # ------------------------------------------------------------------
    # 10+11. Before </model>: add joint, frames, and plugins
    # ------------------------------------------------------------------
    PLUGINS_AND_FRAMES = """
        <!-- ROS root joint -->
        <joint name="base_link_to_Chassis" type="fixed">
            <parent>base_link</parent>
            <child>Chassis</child>
        </joint>

        <!-- Sensor frame definitions (positions in Chassis frame) -->
        <frame name="camera_link" attached_to="Chassis">
            <pose>0.2149999962059069 0.1018999999999998 0.06275000205719003 0 0 0</pose>
        </frame>
        <frame name="imu_link" attached_to="Chassis">
            <pose>0.22470999620590701 0.081339999999999829 0.072750002057190011 0 0 0</pose>
        </frame>
        <frame name="lidar_link" attached_to="Chassis">
            <pose>0.23059999620590699 0.077399999999999816 0.08275000205719002 0 0 0</pose>
        </frame>

        <!-- TrackedVehicle: left/right swapped because LTracks is on the +Y (left) side
             and RTracks is near origin. The old model had them swapped - keep that. -->
        <plugin filename="gz-sim-tracked-vehicle-system" name="gz::sim::systems::TrackedVehicle">
            <left_track>
                <link>RTracks</link>
            </left_track>
            <right_track>
                <link>LTracks</link>
            </right_track>
            <tracks_separation>0.1824</tracks_separation>
            <tracks_height>0.007</tracks_height>
            <steering_efficiency>0.7</steering_efficiency>
            <topic>/model/tracked_robot/cmd_vel</topic>
            <odom_publish_frequency>50</odom_publish_frequency>
            <odom_topic>/wheel/odom</odom_topic>
            <tf_topic>tf</tf_topic>
            <frame_id>odom</frame_id>
            <child_frame_id>base_link</child_frame_id>
        </plugin>
        <plugin filename="gz-sim-track-controller-system" name="gz::sim::systems::TrackController">
            <link>LTracks</link>
            <min_velocity>-0.5555555555555556</min_velocity>
            <max_velocity>0.5555555555555556</max_velocity>
        </plugin>
        <plugin filename="gz-sim-track-controller-system" name="gz::sim::systems::TrackController">
            <link>RTracks</link>
            <min_velocity>-0.5555555555555556</min_velocity>
            <max_velocity>0.5555555555555556</max_velocity>
        </plugin>
        <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
            <joint_name>robotarm_x_joint</joint_name>
            <topic>/robotarm/x_position_cmd</topic>
            <p_gain>80</p_gain>
            <i_gain>0</i_gain>
            <d_gain>20</d_gain>
            <cmd_min>0.01</cmd_min>
            <cmd_max>0.09</cmd_max>
        </plugin>
        <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
            <joint_name>robotarm_z_joint</joint_name>
            <topic>/robotarm/z_position_cmd</topic>
            <p_gain>80</p_gain>
            <i_gain>10</i_gain>
            <d_gain>20</d_gain>
            <cmd_min>0.0</cmd_min>
            <cmd_max>0.25</cmd_max>
        </plugin>
        <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
            <joint_name>left_gripper_finger_joint</joint_name>
            <topic>/gripper/left_position_cmd</topic>
            <p_gain>20</p_gain>
            <i_gain>0</i_gain>
            <d_gain>2</d_gain>
            <cmd_min>-3.228859</cmd_min>
            <cmd_max>-0.523599</cmd_max>
        </plugin>
        <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
            <joint_name>right_gripper_finger_joint</joint_name>
            <topic>/gripper/right_position_cmd</topic>
            <p_gain>20</p_gain>
            <i_gain>0</i_gain>
            <d_gain>2</d_gain>
            <cmd_min>0.523599</cmd_min>
            <cmd_max>2.96706</cmd_max>
        </plugin>
        <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">
            <topic>/joint_states</topic>
            <update_rate>30</update_rate>
            <joint_name>robotarm_x_joint</joint_name>
            <joint_name>robotarm_z_joint</joint_name>
            <joint_name>left_gripper_finger_joint</joint_name>
            <joint_name>right_gripper_finger_joint</joint_name>
            <joint_name>Chassis_Revolute-1</joint_name>
            <joint_name>Chassis_Revolute-2</joint_name>
            <joint_name>Chassis_Revolute-7</joint_name>
            <joint_name>Chassis_Revolute-8</joint_name>
            <joint_name>BRLegs_Revolute-44</joint_name>
            <joint_name>FRLegs_Revolute-45</joint_name>
            <joint_name>FLLegs_Revolute-42</joint_name>
            <joint_name>BLLegs_Revolute-43</joint_name>
        </plugin>
        <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
            <publish_link_pose>true</publish_link_pose>
            <publish_sensor_pose>false</publish_sensor_pose>
            <publish_collision_pose>false</publish_collision_pose>
            <publish_visual_pose>false</publish_visual_pose>
            <publish_nested_model_pose>false</publish_nested_model_pose>
        </plugin>
"""

    # Insert before </model>
    MODEL_END = "    </model>"
    if MODEL_END not in c:
        print("WARNING: </model> not found. Plugins NOT added.")
    else:
        # Replace last occurrence (should only be one)
        idx = c.rfind(MODEL_END)
        c = c[:idx] + PLUGINS_AND_FRAMES + c[idx:]

    # ------------------------------------------------------------------
    # Write output
    # ------------------------------------------------------------------
    os.makedirs(os.path.dirname(DST), exist_ok=True)
    with open(DST, "w") as f:
        f.write(c)
    print(f"Done. Written to {DST}")
    print(f"Lines: {c.count(chr(10))}")

if __name__ == "__main__":
    build()
