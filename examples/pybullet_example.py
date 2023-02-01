import pybullet as p
import pybullet_utils.bullet_client as bc
import pkgutil

import numpy as np
from tactile_gym.assets import add_assets_path
from tactile_servo_control.utils_robot_sim.robot_embodiment import RobotEmbodiment
from tactile_servo_control.utils.pose_transforms import transform_pose, inv_transform_pose
from user_input.space_mouse import SpaceMouse

POSE_UNITS = np.array([1e-3, 1e-3, 1e-3, np.pi/180, np.pi/180, np.pi/180])


def setup_pybullet_env(
    workframe,
    sensor_params,
    show_gui,
    show_tactile,
    quick_mode=False
):

    # ========= environment set up ===========
    time_step = 1.0 / 240

    if show_gui:
        pb = bc.BulletClient(connection_mode=p.GUI)
    else:
        pb = bc.BulletClient(connection_mode=p.DIRECT)
        egl = pkgutil.get_loader("eglRenderer")
        if egl:
            p.loadPlugin(egl.get_filename(), "_eglRendererPlugin")
        else:
            p.loadPlugin("eglRendererPlugin")

    pb.setGravity(0, 0, -10)
    pb.setPhysicsEngineParameter(
        fixedTimeStep=time_step,
        numSolverIterations=300,
        numSubSteps=1,
        contactBreakingThreshold=0.0005,
        erp=0.05,
        contactERP=0.05,
        frictionERP=0.2,
        solverResidualThreshold=1e-7,
        contactSlop=0.001,
        globalCFM=0.0001,
    )

    plane_id = pb.loadURDF(
        add_assets_path("shared_assets/environment_objects/plane/plane.urdf"),
        [0, 0, -0.625],
    )
    table_id = pb.loadURDF(
        add_assets_path("shared_assets/environment_objects/table/table.urdf"),
        [0.50, 0.00, -0.625],
        [0.0, 0.0, 0.0, 1.0],
    )

    # turn off collisions with environment to speed up simulation
    # in static environment these collisions are not useful
    pb.setCollisionFilterGroupMask(plane_id, -1, 0, 0)
    pb.setCollisionFilterGroupMask(table_id, -1, 0, 0)

    # set debug camera position
    cam_params = {
        'image_size': [512, 512],
        'dist': 0.5,
        'yaw': 90.0,
        'pitch': -25.0,
        'pos': [0.65, 0.0, 0.15],
        'fov': 75.0,
        'near_val': 0.1,
        'far_val': 100.0
    }

    if show_gui:
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
        p.resetDebugVisualizerCamera(
            cam_params['dist'],
            cam_params['yaw'],
            cam_params['pitch'],
            cam_params['pos']
        )

    # set the workrame of the robot (relative to world frame)
    workframe *= POSE_UNITS
    workframe_pos, workframe_rpy = workframe[:3], workframe[3:]

    # create the robot and sensor embodiment
    embodiment = RobotEmbodiment(
        pb,
        workframe_pos=workframe_pos,
        workframe_rpy=workframe_rpy,
        image_size=sensor_params["image_size"],
        arm_type="ur5",
        t_s_params=sensor_params,
        cam_params=cam_params,
        show_gui=show_gui,
        show_tactile=show_tactile,
        quick_mode=quick_mode
    )

    return embodiment


def main():

    workframe = [650, 0.0, 150, -180, 0.0, 90]
    sensor_params = {
        "name": "tactip",
        "type": "standard",
        "core": "no_core",
        "dynamics": {},
        "image_size": [256, 256],
        "turn_off_border": False,
    }

    n_eps = 10
    show_gui = True
    show_tactile = False

    # setup the robot
    embodiment = setup_pybullet_env(
        workframe,
        sensor_params,
        show_gui,
        show_tactile,
        quick_mode=False
    )

    space_mouse = SpaceMouse()
    with space_mouse:

        for i in range(n_eps):
            embodiment.move_linear(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
            d = False
            while not d:

                sm_state = space_mouse.get_state()

                # intuitive control
                sm_pose = np.array([
                    sm_state.x,
                    -sm_state.y,
                    -sm_state.z,
                    -sm_state.pitch,
                    -sm_state.roll,
                    sm_state.yaw,
                ]) * 0.25

                # sm_pose = np.array([
                #     sm_state.x,
                #     sm_state.y,
                #     sm_state.z,
                #     sm_state.roll,
                #     sm_state.pitch,
                #     sm_state.yaw,
                # ]) * 0.25

                # get pose of tcp in workframe
                tcp_pose = embodiment.get_tcp_pose()

                # calculate sm pose in workframe
                pose = inv_transform_pose(sm_pose, tcp_pose)

                # move to new pose
                embodiment.move_linear(pose)

                # visualise features
                # embodiment.arm.draw_workframe()
                # embodiment.arm.draw_TCP()

                # check for keyboard interrupts
                q_key = ord("q")
                r_key = ord("r")
                keys = p.getKeyboardEvents()
                if q_key in keys and keys[q_key] & p.KEY_WAS_TRIGGERED:
                    exit()
                elif r_key in keys and keys[r_key] & p.KEY_WAS_TRIGGERED:
                    d = True


if __name__ == "__main__":
    main()
