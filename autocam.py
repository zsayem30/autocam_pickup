import numpy as np
import dvrk
import PyKDL
import rospy
import sys
import tf_conversions.posemath as pm
from scipy.spatial.transform import Rotation as R
from numpy import linalg as LA

if sys.version_info.major < 3:
    input = raw_input
    
def pickup_transform(cam_offset):

    psm3_T_cam = PyKDL.Frame().Identity()
    psm3_T_cam.p[2] += cam_offset

    return psm3_T_cam

def ring_transform(ring_offset):

    psm1_T_R = PyKDL.Frame().Identity()
    psm1_T_R.p[2] += ring_offset

    return psm1_T_R
    
def orient_camera(psm3_T_cam, ecm_T_R, ecm_T_w, z_i, df, offset):

    y_R = pm.toMatrix(ecm_T_R)[0:3, 1]

    y_R = y_R / LA.norm(y_R)
    z_i = z_i / LA.norm(z_i)
    z_w = pm.toMatrix(ecm_T_w)[0:3, 2] / LA.norm(pm.toMatrix(ecm_T_w)[0:3, 2])
    
    sgn1 =  np.sign(np.dot(z_i, y_R)) 
    sgn2 = np.sign(np.dot(z_w, y_R))

    z_cam = sgn1 * sgn2 * y_R 


    x_cam = np.cross(z_w, z_cam) / LA.norm(np.cross(z_w, z_cam))
    y_cam = np.cross(z_cam, x_cam) / LA.norm(np.cross(z_cam, x_cam))

    x_cam_vec = PyKDL.Vector(x_cam[0], x_cam[1], x_cam[2])
    y_cam_vec = PyKDL.Vector(y_cam[0], y_cam[1], y_cam[2])
    z_cam_vec = PyKDL.Vector(z_cam[0], z_cam[1], z_cam[2])

    ecm_R_cam = PyKDL.Rotation(x_cam_vec, y_cam_vec, z_cam_vec)


    ecm_p_cam = ecm_T_R.p - df * z_cam_vec

    ecm_T_cam_desired = PyKDL.Frame(ecm_R_cam, ecm_p_cam)
    ecm_T_psm3_desired = ecm_T_cam_desired * psm3_T_cam.Inverse()
    ecm_T_psm3_desired.p = ecm_T_psm3_desired.p  - offset

    return ecm_T_psm3_desired

## When the state is distabled, then don't run teloperation
def disabled():
    return None

## Setting arm states. We want to "enable" and "home" the two arms. Check operating_state for the two arms.

def setting_arms_state(arm):

    if arm.operating_state() == "DISABLED":
        arm.enable()
        arm.home()
        
# def initial_align(psm3, start_pose):
    
if __name__ == '__main__':

    print("Initializing arms...")

    psm1 = dvrk.psm("PSM1")
    psm3 = dvrk.psm("PSM3")
    ecm = dvrk.ecm("ECM")

    setting_arms_state(psm1)
    setting_arms_state(psm3)
    setting_arms_state(ecm)

    ecm_pose = ecm.setpoint_cp()

    ring_offset = 0.015 ## 1.5 cm
    cam_offset = 0.04 ## 4 cm
    df = 0.15 ## in cms
    ## HARD CODED OFFSET FOR GIVEN JOINT CONFIGURATION
    offset = PyKDL.Vector(   -0.0264435,   0.0354474,    0.294303)

    # Find respective transforms from psm1 to ring and from psm3 to cam
    psm1_T_R = ring_transform(ring_offset)
    psm3_T_cam = pickup_transform(cam_offset)    
    ecm_T_w = ecm.setpoint_cp().Inverse()

    message_rate = 0.01

    ## For first iteration, we need to gracefully park PSM3 at the start of our tracking...
    # Query the poses for psm1, psm3 and ecm
    psm1_pose = psm1.setpoint_cp()
    psm3_pose = psm3.setpoint_cp()

    ecm_T_R = psm1_pose * psm1_T_R

    z_i = pm.toMatrix(psm3_pose)[0:3, 2]

    ecm_T_psm3_desired_Pose = orient_camera(psm3_T_cam, ecm_T_R, ecm_T_w, z_i, df, offset)
    
    print("Parking PSM3 to starting position...")
    fixed_posed = ecm_T_psm3_desired_Pose.p
    psm3.move_cp(ecm_T_psm3_desired_Pose).wait()

    input("    Press Enter to start autonomous tracking...")
    
    ## For every iteration:
    while not rospy.is_shutdown():

        # Query the poses for psm1, psm3 and ecm
        psm1_pose = psm1.setpoint_cp()
        psm3_pose = psm3.setpoint_cp()

        ecm_T_R = psm1_pose * psm1_T_R

        z_i = pm.toMatrix(psm3_pose)[0:3, 2]

        ecm_T_psm3_desired_Pose = orient_camera(psm3_T_cam, ecm_T_R, ecm_T_w, z_i, df, offset)
        # ecm_T_psm3_desired_Pose.p = fixed_posed
        
        psm3.move_cp(ecm_T_psm3_desired_Pose)
        rospy.sleep(message_rate)
