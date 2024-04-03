import numpy as np
import dvrk
import PyKDL
import rospy
import tf_conversions.posemath as pm
from scipy.spatial.transform import Rotation as R
from numpy import linalg as LA


def pickup_transform(cam_offset):

    psm3_T_cam = PyKDL.Frame().Identity()
    psm3_T_cam.p[2] += cam_offset

    return psm3_T_cam

def ring_transform(ring_offset):

    psm1_T_R = PyKDL.Frame().Identity()
    psm1_T_R.p[2] += ring_offset

    return psm1_T_R
    
def orient_camera(psm1_T_R, ecm_T_R, ecm_T_w, df):

    y_R = pm.toMatrix(ecm_T_R)[0:3, 1]
    z_cam = - y_R / LA.norm(y_R)

    z_w = pm.toMatrix(ecm_T_w)[0:3, 2]

    x_cam = np.cross(z_w, z_cam) / LA.norm(np.cross(z_w, z_cam))
    y_cam = np.cross(z_cam, x_cam) / LA.norm(np.cross(z_cam, x_cam))

    x_cam_vec = PyKDL.Vector(x_cam[0], x_cam[1], x_cam[2])
    y_cam_vec = PyKDL.Vector(y_cam[0], y_cam[1], y_cam[2])
    z_cam_vec = PyKDL.Vector(z_cam[0], z_cam[1], z_cam[2])

    ecm_R_cam = PyKDL.Rotation(x_cam_vec, y_cam_vec, z_cam_vec)


    ecm_p_cam = ecm_T_R.p - df * z_cam_vec

    ecm_T_cam_desired = PyKDL.Frame(ecm_R_cam, ecm_p_cam)

    ecm_T_psm3_desired = ecm_T_cam_desired * psm1_T_R.Inverse()

    return ecm_T_psm3_desired

## When the state is distabled, then don't run teloperation
def disabled():
    return None

## Setting arm states. We want to "enable" and "home" the two arms. Check operating_state for the two arms.

def setting_arms_state(arm):

    if arm.operating_state() == "DISABLED":
        arm.enable()
        arm.home()
        
if __name__ == '__main__':

    print("Initializing arms...")

    psm1 = dvrk.psm("PSM1")
    psm3 = dvrk.psm("PSM3")
    ecm = dvrk.ecm("ECM")

    setting_arms_state(psm1)
    setting_arms_state(psm3)
    setting_arms_state(ecm)

    # Query the poses for psm1, psm3 and ecm
    psm1_pose = psm1.setpoint_cp()
    psm3_pose = psm3.setpoint_cp()
    print("PSM3 initial Pose")
    print(psm3_pose)
    ecm_pose = ecm.setpoint_cp()

    ring_offset = 0.01 ## 1 cm
    psm1_T_R = ring_transform(ring_offset)
    ecm_T_R = psm1_pose * psm1_T_R
    ecm_T_w = ecm.setpoint_cp().Inverse()

    df = 0.10 ## in cm
    ## HARD CODED OFFSET FOR GIVEN JOINT CONFIGURATION
    offset = PyKDL.Vector(-0.0746767, -0.0291252, 0.285394)

    # Returns the desired pose of psm3 wrt ecm that should be executed.
    ecm_T_psm3_desired_Pose = orient_camera(psm1_T_R, ecm_T_R, ecm_T_w, df)

    ## Maybe we can do some sort of an interpolation
    print("PSM3 desired Pose")
    print(ecm_T_psm3_desired_Pose)
    # First we set the orientation to match the desired orientation:
    # psm3_pose.M = ecm_T_psm3_desired_Pose.M
    ecm_T_psm3_desired_Pose.p = ecm_T_psm3_desired_Pose.p  - offset

    psm3.move_cp(ecm_T_psm3_desired_Pose).wait()

    # iterations = 1000
    # dv = (1/iterations) * (ecm_T_psm3_desired_Pose.p - psm3_pose.p)
    # message_rate = 0.01

    # for i in range(iterations):

    #     psm3_pose.p = psm3_pose.p + dv
    #     psm3.servo_cp(psm3_pose)

    #     rospy.sleep(message_rate)

    # psm3_pose.M = ecm_T_psm3_desired_Pose.M
    # psm3.move_cp(ecm_T_psm3_desired_Pose).wait()   
    # print(ecm_T_psm3_desired_Pose)
    # psm3.move_cp(ecm_T_psm3_desired_Pose).wait()
    rospy.sleep(2.0)
    print("PSM3 actual Pose")
    print(psm3.setpoint_cp())
    print("Difference in position vector")
    print((psm3.setpoint_cp().p - ecm_T_psm3_desired_Pose.p))

    # print((psm1.measured_cp().p - psm3.measured_cp().p))



    








    




