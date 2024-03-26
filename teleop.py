import numpy as np
import dvrk
import PyKDL
import rospy
import tf_conversions.posemath as pm
from scipy.spatial.transform import Rotation as R

## Important notes:
## For MTM motion:
    # Position: 3D position of MTML (w.r.t to surgeon console) is used to compute the relative translation
    #           applied to PSM1 position (w.r.t to **PSM3**)
    # Orientation: 3D orientation of PSM1 w.r.t PSM3 should always match the orientation of MTML w.r.t 
    #              surgeon console. When user is not driving the PSM (user not present or pressing clutch)
    #              MTML orientation (w.r.t surgeon console) moves to match PSM1 orientation (w.r.t PSM3).
    #              When user is driving PSM1 using MTML (i.e follow mode), PSM3 orientation (w.r.t PSM3)
    #              moves to match MTML orientation (w.r.t surgeon console).
    # Gripper: 1-1 Relationship


# class Teleop:

## States:

## When the state is distabled, then don't run teloperation
def disabled():
    return None

## Setting arm states. We want to "enable" and "home" the two arms. Check operating_state for the two arms.

def setting_arms_state(arm):

    if arm.operating_state() == "DISABLED":
        arm.enable()
        arm.home()
        
def pickup_transform(offset):

    psm3_T_cam = PyKDL.Identity()
    psm3_T_cam.p[2] += offset

    return psm3_T_cam

## The first thing the teleoperation has to do is make sure the MTML orientation matches the PSM1 one. 
## The PSM1 orientation (from PSM1/setpoint_cp) is used along with the current MTML position (MTML/measured_cp)
## to compute the desired pose for the MTML. We use a move command (MTML/move_cp)
## Need to convert the PSM1 pose wrt to ECM to PSM1 
        
def align_MTML(mtml, psm1, psm3):

    ## Get PSM1 pose w.r.t ECM: {ECM}^T_{PSM1}
    ecm_T_psm1 = psm1.setpoint_cp()

    ## Get ECM pose w.r.t PSM3: {PSM3}^T_{ECM}
    ecm_T_psm3 = psm3.setpoint_cp()

    psm3_T_ecm = ecm_T_psm3.Inverse()

    ## Get MTML pose w.r.t display
    hrsv_T_mtml = mtml.measured_cp()

    ## The psm1 pose w.r.t psm3 is just inv(psm3_pose_ecm) * psm1_pose_ecm
    psm3_T_psm1 = psm3_T_ecm * ecm_T_psm1

    hrsv_T_mtml.M = psm3_T_psm1.M

    mtml.move_cp(hrsv_T_mtml).wait()

    rospy.sleep(2)

    alignment_offset = mtml.measured_cp().M.Inverse() * psm3_T_psm1.M
    
    return alignment_offset

def axis_angle_offset(alignment_offset):

    alignment_offset_angle, _ = alignment_offset.GetRotAngle()
    return alignment_offset_angle * (180/np.pi)

def enable_mtml(mtml):

    zero_wrench = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    mtml.use_gravity_compensation(True)
    mtml.body.servo_cf(zero_wrench)

    return True


def follow_mode(mtml, psm1, psm3, scale_factor, alignment_offset):
    
    # Save the mtml, psm1, psm3 poses
    hrsv_T_mtml_ini = mtml.measured_cp()
    ecm_T_psm1_ini = psm1.setpoint_cp() ## w.r.t ECM
    # psm1_initial_pose = inv(psm3_initial_pose) * pm.toMatrix(psm1.setpoint_cp()) ## w.r.t PSM3

    print("PSM1 Current Pose (w.r.t ECM):")
    print(ecm_T_psm1_ini.p)

    ## For every iteration:

    i = 0
    message_rate = 0.01

    while not rospy.is_shutdown():


        hrsv_T_mtml_curr = mtml.setpoint_cp()

        mtml_jaw_angle = mtml.gripper.measured_js()
        
        ecm_T_psm3_curr = psm3.setpoint_cp() ## w.r.t ECM
                
        ecm_T_psm1_next = ecm_T_psm1_ini

        ecm_T_psm1_next.M = ecm_T_psm3_curr.M * hrsv_T_mtml_curr.M * alignment_offset


        mtml_translation = scale_factor * (hrsv_T_mtml_curr.p - hrsv_T_mtml_ini.p)

        ecm_T_psm1_next.p = ecm_T_psm1_next.p + ecm_T_psm3_curr.M * mtml_translation

        if i == 1:
            print("desired PSM1 pose: ")
            print(ecm_T_psm1_next.p)


        psm1.servo_cp(ecm_T_psm1_next)
        psm1.jaw.servo_jp(np.array(mtml_jaw_angle[0]))

        rospy.sleep(message_rate)

        hrsv_T_mtml_ini = hrsv_T_mtml_curr
        ecm_T_psm1_ini = ecm_T_psm1_next

        i = i + 1





if __name__ == '__main__':
    
    print("Initializing arms...")

    psm1 = dvrk.psm("PSM1")
    psm3 = dvrk.psm("PSM3")
    mtml = dvrk.mtm("MTML")

    print("Readying arms...")
    setting_arms_state(psm1)
    setting_arms_state(psm3)
    setting_arms_state(mtml)

    # print("Aligning MTML...")
    alignment_offset = align_MTML(mtml, psm1, psm3)
    
    alignment_offset_angle = axis_angle_offset(alignment_offset)
    offset_angle_threshold = 5.0 ## Degrees
    scale_factor = 0.2

    print("The alignment offset is: " + str(alignment_offset_angle))

    print("The final alignment offset is: " + str(alignment_offset_angle))

    print("Enable MTML")
    follow_mode_Start = enable_mtml(mtml)


    follow_mode(mtml, psm1, psm3, scale_factor, alignment_offset)

