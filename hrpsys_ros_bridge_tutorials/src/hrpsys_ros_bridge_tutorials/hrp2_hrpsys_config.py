#!/usr/bin/env python

pkg = 'hrpsys'
import imp
imp.find_module(pkg)

from hrpsys.hrpsys_config import *
import OpenHRP

class JSKHRP2HrpsysConfigurator(HrpsysConfigurator):
    ROBOT_NAME = None

    def getRTCList (self):
        return self.getRTCListUnstable()
    def init (self, robotname="Robot", url=""):
        HrpsysConfigurator.init(self, robotname, url)
        print "initialize rtc parameters"
        self.setStAbcParameters()
        self.loadForceMomentOffsetFile()

    def defJointGroups (self):
        rleg_6dof_group = ['rleg', ['RLEG_JOINT0', 'RLEG_JOINT1', 'RLEG_JOINT2', 'RLEG_JOINT3', 'RLEG_JOINT4', 'RLEG_JOINT5']]
        lleg_6dof_group = ['lleg', ['LLEG_JOINT0', 'LLEG_JOINT1', 'LLEG_JOINT2', 'LLEG_JOINT3', 'LLEG_JOINT4', 'LLEG_JOINT5']]
        rleg_7dof_group = ['rleg', ['RLEG_JOINT0', 'RLEG_JOINT1', 'RLEG_JOINT2', 'RLEG_JOINT3', 'RLEG_JOINT4', 'RLEG_JOINT5', 'RLEG_JOINT6']]
        lleg_7dof_group = ['lleg', ['LLEG_JOINT0', 'LLEG_JOINT1', 'LLEG_JOINT2', 'LLEG_JOINT3', 'LLEG_JOINT4', 'LLEG_JOINT5', 'LLEG_JOINT6']]
        torso_group = ['torso', ['CHEST_JOINT0', 'CHEST_JOINT1']]
        head_group = ['head', ['HEAD_JOINT0', 'HEAD_JOINT1']]
        rarm_group = ['rarm', ['RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5', 'RARM_JOINT6', 'RARM_JOINT7']]
        larm_group = ['larm', ['LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5', 'LARM_JOINT6', 'LARM_JOINT7']]
        if self.ROBOT_NAME == "HRP2JSKNT" or self.ROBOT_NAME == "HRP2JSKNTS":
            self.Groups = [rleg_7dof_group, lleg_7dof_group, torso_group, head_group, rarm_group, larm_group]
        elif self.ROBOT_NAME == "HRP2JSK":
            self.Groups = [rleg_6dof_group, lleg_6dof_group, torso_group, head_group, rarm_group, larm_group]
        else: # HRP2W, HRP2G
            self.Groups = [torso_group, head_group, rarm_group, larm_group]

    def hrp2ResetPose (self):
        if self.ROBOT_NAME.find("HRP2JSKNT") != -1:
            return [0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.174533, -0.174533, 0.0, -0.436332, 0.0, 0.0, -0.174533, 0.261799, 0.174533, 0.174533, 0.0, -0.436332, 0.0, 0.0, -0.174533, -0.261799]
        elif self.ROBOT_NAME.find("HRP2JSK") != -1:
            return [0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, 0.0, 0.0, 0.174533, -0.174533, 0.0, -0.436332, 0.0, 0.0, -0.174533, 0.261799, 0.174533, 0.174533, 0.0, -0.436332, 0.0, 0.0, -0.174533, -0.261799]
        else:
            return [0.0, 0.0, 0.0, 0.0, 0.174533, -0.174533, 0.0, -1.5708, 0.0, 0.0, -0.174533, 0.261799, 0.174533, 0.174533, 0.0, -1.5708, 0.0, 0.0, -0.174533, -0.261799]

    def hrp2ResetManipPose (self):
        if self.ROBOT_NAME.find("HRP2JSKNT") != -1:
            return [0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, 0.0, 0.0, 0.698132, 0.872665, -0.523599, -0.174533, -2.0944, -0.436332, -0.087266, -0.349066, 1.0472, 0.872665, 0.523599, 0.174533, -2.0944, 0.436332, 0.087266, -0.349066, -1.0472]
        elif self.ROBOT_NAME.find("HRP2JSK") != -1:
            return [0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, 0.0, 0.698132, 0.872665, -0.523599, -0.174533, -2.0944, -0.436332, -0.087266, -0.349066, 1.0472, 0.872665, 0.523599, 0.174533, -2.0944, 0.436332, 0.087266, -0.349066, -1.0472]
        else:
            return [0.0, 0.0, 0.0, 0.698132, 0.872665, -0.523599, -0.174533, -2.0944, -0.436332, -0.087266, -0.349066, 1.0472, 0.872665, 0.523599, 0.174533, -2.0944, 0.436332, 0.087266, -0.349066, -1.0472]

    def hrp2InitPose (self):
        if self.ROBOT_NAME.find("HRP2JSKNT") != -1:
            ret=[0]*len(self.hrp2ResetPose())
            ret[2]=-0.0174532925
            ret[3]=0.034906585
            ret[4]=-0.0174532925
            ret[9]=-0.0174532925
            ret[10]=0.034906585
            ret[11]=-0.0174532925
            ret[21]=-0.034906585
            ret[29]=-0.034906585
            return ret
        elif self.ROBOT_NAME.find("HRP2JSK") != -1:
            ret=[0]*len(self.hrp2ResetPose())
            ret[31]=-0.261799
            ret[23]=0.261799
            return ret
        else:
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.5708, 0.0, 0.0, 0.0, 1.0472, 0.0, 0.0, 0.0, -1.5708, 0.0, 0.0, 0.0, -1.0472]

    def setStAbcParameters (self):
        if self.ROBOT_NAME == "HRP2JSKNT":
            self.setStAbcParametershrp2016c()
        elif self.ROBOT_NAME == "HRP2JSKNTS":
            self.setStAbcParametershrp2017c() # for hrp2017
        elif self.ROBOT_NAME == "HRP2JSK":
            self.setStAbcParametershrp2007c() 

    # for eefm Stabilizer, hrp2017, new
    def setStAbcParametershrp2017c(self):
        # ABC parameters
        abcp=self.abc_svc.getAutoBalancerParam()[1]
        #abcp.default_zmp_offsets = [[0.015, -0.01, 0], [0.015, 0.01, 0], [0, 0, 0], [0, 0, 0]]
        #abcp.default_zmp_offsets = [[0.015, 0.01, 0], [0.015, -0.01, 0], [0, 0, 0], [0, 0, 0]]
        abcp.default_zmp_offsets = [[0.01, 0.01, 0], [0.01, -0.01, 0], [0, 0, 0], [0, 0, 0]]
        self.abc_svc.setAutoBalancerParam(abcp)
        # ST parameters
        stp=self.st_svc.getParameter()
        stp.st_algorithm=OpenHRP.StabilizerService.MCS
        #   eefm st params
        stp.eefm_use_quaternion_body_attitude_control=True
        stp.eefm_body_attitude_control_gain=[1.5, 1.5]
        stp.eefm_body_attitude_control_time_const=[10000, 10000]
        # EEFM parameters for 4 limbs
        #stp.eefm_rot_damping_gain = [[20*1.6, 20*1.6, 1e5]]*4
        #stp.eefm_pos_damping_gain = [[3500*50, 3500*50, 3500*1.0]]*4
        stp.eefm_rot_damping_gain = [[35, 35, 1e5]]*4
        stp.eefm_pos_damping_gain = [[3500*50, 3500*50, 3700*1.0]]*4
        stp.eefm_rot_time_const = [[1.5, 1.5, 1.5]]*4
        stp.eefm_pos_time_const_support = [[1.5, 1.5, 1.5]]*4
        stp.eefm_swing_pos_damping_gain=stp.eefm_pos_damping_gain[0]
        stp.eefm_swing_rot_damping_gain=stp.eefm_rot_damping_gain[0]
        stp.eefm_use_swing_damping=True
        stp.eefm_wrench_alpha_blending = 0.6
        stp.eefm_pos_time_const_swing=0.08
        stp.eefm_pos_transition_time=0.01
        stp.eefm_pos_margin_time=0.02
        stp.eefm_zmp_delay_time_const=[0.055, 0.055]
        stp.eefm_cogvel_cutoff_freq=6.0
        #   mechanical foot edge
        #stp.eefm_leg_inside_margin=0.065
        #stp.eefm_leg_front_margin=0.140
        #stp.eefm_leg_rear_margin=0.105
        #   margined foot edge
        tmp_leg_inside_margin=0.062
        tmp_leg_outside_margin=0.062
        tmp_leg_front_margin=0.130
        tmp_leg_rear_margin=0.095
        stp.eefm_leg_inside_margin=tmp_leg_inside_margin
        stp.eefm_leg_outside_margin=tmp_leg_outside_margin
        stp.eefm_leg_front_margin=tmp_leg_front_margin
        stp.eefm_leg_rear_margin=tmp_leg_rear_margin
        tmp_arm_inside_margin=0.005
        tmp_arm_outside_margin=0.005
        tmp_arm_front_margin=0.02
        tmp_arm_rear_margin=0.02
        stp.eefm_arm_inside_margin=tmp_arm_inside_margin
        stp.eefm_arm_outside_margin=tmp_arm_outside_margin
        stp.eefm_arm_front_margin=tmp_arm_front_margin
        stp.eefm_arm_rear_margin=tmp_arm_rear_margin
        rleg_vertices = [OpenHRP.StabilizerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, tmp_leg_inside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, -1*tmp_leg_outside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, -1*tmp_leg_outside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, tmp_leg_inside_margin])]
        lleg_vertices = [OpenHRP.StabilizerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, tmp_leg_outside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, -1*tmp_leg_inside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, -1*tmp_leg_inside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, tmp_leg_outside_margin])]
        rarm_vertices = rleg_vertices
        larm_vertices = lleg_vertices
        stp.eefm_support_polygon_vertices_sequence = map (lambda x : OpenHRP.StabilizerService.SupportPolygonVertices(vertices=x), [rleg_vertices, lleg_vertices, rarm_vertices, larm_vertices])
        #   tpcc st params
        stp.k_tpcc_p=[2.0, 2.0]
        stp.k_tpcc_x=[5.0, 5.0]
        stp.k_brot_p=[0.0, 0.0]
        stp.k_brot_tc=[0.1, 0.1]
        #   cog height = 800[mm], alpha = -13.0, beta = -4.0, time_const = 0.04[s]
        #stp.eefm_k1=[-1.41413,-1.41413]
        #stp.eefm_k2=[-0.403901,-0.403901]
        #stp.eefm_k3=[-0.179953,-0.179953]
        stp.eefm_k1=[-1.272861, -1.272861]
        stp.eefm_k2=[-0.36367379999999999, -0.36367379999999999]
        stp.eefm_k3=[-0.16200000000000001, -0.16200000000000001]
        # for estop
        stp.emergency_check_mode=OpenHRP.StabilizerService.CP;
        stp.cp_check_margin=[50*1e-3, 45*1e-3, 0, 100*1e-3];
        # for swing
        stp.eefm_swing_pos_spring_gain = [[1]*3, [1]*3, [0]*3, [0]*3]
        stp.eefm_swing_rot_spring_gain = [[1]*3, [1]*3, [0]*3, [0]*3]
        # for MCS
        stp.mcs_debug_ratio = 0
        stp.mcs_sv_ratio = 1e-12
        stp.mcs_qactv_cutoff_freq = 100.0
        stp.mcs_acttauv_cutoff_freq = 10.0
        stp.mcs_coiltemp_cutoff_freq = 1.0
        stp.mcs_surfacetemp_cutoff_freq = 1.0
        #stp.is_joint_enable = [True, True, True, True, True, True, False] + [True, True, True, True, True, True, False] + [False, False] + [False, False] + [False, False, False, False, False, False, False, False] + [False, False, False, False, False, False, False, False]
        stp.is_joint_enable = [True, True, True, True, True, True, False] + [True, True, True, True, True, True, False] + [True, True] + [False, False] + [True, True, True, True, True, True, True, False] + [True, True, True, True, True, True, True, False]
        stp.tau_weight = 1e-8
        #stp.tauvel_weight = 1e4
        stp.temp_safe_time = 100.0
        stp.temp_danger_time = 30.0
        stp.force_weight = 1e-8
        #stp.forcevel_weight = 1e2
        stp.intvel_weight = 1e-4
        stp.P_weight = 1e-2
        stp.Pvel_weight = 1e-4
        stp.L_weight = 1e-2
        stp.Lvel_weight = 1e-4
        stp.vel_weight = 1e-6
        stp.acc_weight = 1e-6
        stp.reference_weight = 1e-8
        stp.etau_weight = 1e-3
        #stp.etauvel_weight = 1e10
        #stp.etau_time = 1
        stp.eforce_weight = 1e-3
        #stp.eforcevel_weight = 1e10
        #stp.eforce_time = 10
        stp.taumax_weight = 1e1
        #stp.taumaxvel_weight = 1e5
        stp.tau_M = 10
        stp.tau_D = 500
        stp.tau_K = 100
        stp.mcs_mcs_passive_vel = 0.034907
        stp.mcs_sync2activetime = 5.0
        stp.mcs_sync2referencetime = 5.0
        stp.mcs_passive_torquedirection = [0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0] + [0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0] + [0.0, 0.0] + [0.0, 0.0] + [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] + [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        stp.mcs_collisionthre = 0.005
        stp.mcs_eeparams[0].is_ik_enable = True
        stp.mcs_eeparams[0].contact_decision_threshold = 10.0
        stp.mcs_eeparams[0].act_force_cutoff_freq = 10.0
        stp.mcs_eeparams[0].act_moment_cutoff_freq = 10.0
        stp.mcs_eeparams[0].contact_type = OpenHRP.StabilizerService.SURFACE
        stp.mcs_eeparams[0].friction_coefficient = 0.3
        stp.mcs_eeparams[0].rotation_friction_coefficient = 0.04
        stp.mcs_eeparams[0].upper_cop_x_margin = 0.063
        stp.mcs_eeparams[0].lower_cop_x_margin = -0.090
        stp.mcs_eeparams[0].upper_cop_y_margin = 0.052
        stp.mcs_eeparams[0].lower_cop_y_margin = -0.052
        stp.mcs_eeparams[0].max_fz = 1200.0
        stp.mcs_eeparams[0].min_fz = 20.0
        stp.mcs_eeparams[0].target_max_fz = 5.0
        stp.mcs_eeparams[0].z_leave_weight = 1e0
        stp.mcs_eeparams[0].z_leavevel_weight = 1e-3
        stp.mcs_eeparams[0].other_leave_weight = 1e-5
        stp.mcs_eeparams[0].wrench_M = 10
        stp.mcs_eeparams[0].wrench_D = 500
        stp.mcs_eeparams[0].wrench_K = 100
        stp.mcs_eeparams[0].pos_interact_weight = 1.0
        stp.mcs_eeparams[0].rot_interact_weight = 1.0
        stp.mcs_eeparams[0].M_p = 100
        stp.mcs_eeparams[0].D_p = 2000
        stp.mcs_eeparams[0].K_p = 4000
        stp.mcs_eeparams[0].M_r = 50
        stp.mcs_eeparams[0].D_r = 1000
        stp.mcs_eeparams[0].K_r = 2000
        stp.mcs_eeparams[0].force_gain = [1,1,1]
        stp.mcs_eeparams[0].moment_gain = [1,1,1]
        stp.mcs_eeparams[0].pos_compensation_limit = 0.2
        stp.mcs_eeparams[0].rot_compensation_limit = 0.523599
        stp.mcs_eeparams[0].footorigin_weight = 1e0
        stp.mcs_eeparams[0].z_contact_weight = 1e0
        stp.mcs_eeparams[0].z_contact_vel = 0.01
        stp.mcs_eeparams[0].rot_contact_weight = 1e0
        stp.mcs_eeparams[0].rot_contact_vel = 0.05
        stp.mcs_eeparams[0].outside_upper_cop_x_margin = 0.072
        stp.mcs_eeparams[0].outside_lower_cop_x_margin = -0.098
        stp.mcs_eeparams[0].outside_upper_cop_y_margin = 0.063
        stp.mcs_eeparams[0].outside_lower_cop_y_margin = -0.063
        stp.mcs_eeparams[1].is_ik_enable = True
        stp.mcs_eeparams[1].contact_decision_threshold = 10.0
        stp.mcs_eeparams[1].act_force_cutoff_freq = 10.0
        stp.mcs_eeparams[1].act_moment_cutoff_freq = 10.0
        stp.mcs_eeparams[1].contact_type = OpenHRP.StabilizerService.SURFACE
        stp.mcs_eeparams[1].friction_coefficient = 0.3
        stp.mcs_eeparams[1].rotation_friction_coefficient = 0.04
        stp.mcs_eeparams[1].upper_cop_x_margin = 0.063
        stp.mcs_eeparams[1].lower_cop_x_margin = -0.090
        stp.mcs_eeparams[1].upper_cop_y_margin = 0.052
        stp.mcs_eeparams[1].lower_cop_y_margin = -0.052
        stp.mcs_eeparams[1].max_fz = 1200.0
        stp.mcs_eeparams[1].min_fz = 20.0
        stp.mcs_eeparams[1].target_max_fz = 5.0
        stp.mcs_eeparams[1].z_leave_weight = 1e0
        stp.mcs_eeparams[1].z_leavevel_weight = 1e-3
        stp.mcs_eeparams[1].other_leave_weight = 1e-5
        stp.mcs_eeparams[1].wrench_M = 10
        stp.mcs_eeparams[1].wrench_D = 500
        stp.mcs_eeparams[1].wrench_K = 100
        stp.mcs_eeparams[1].pos_interact_weight = 1.0
        stp.mcs_eeparams[1].rot_interact_weight = 1.0
        stp.mcs_eeparams[1].M_p = 100
        stp.mcs_eeparams[1].D_p = 2000
        stp.mcs_eeparams[1].K_p = 2000
        stp.mcs_eeparams[1].M_r = 50
        stp.mcs_eeparams[1].D_r = 1000
        stp.mcs_eeparams[1].K_r = 2000
        stp.mcs_eeparams[1].force_gain = [1,1,1]
        stp.mcs_eeparams[1].moment_gain = [1,1,1]
        stp.mcs_eeparams[1].pos_compensation_limit = 0.2
        stp.mcs_eeparams[1].rot_compensation_limit = 0.523599
        stp.mcs_eeparams[1].footorigin_weight = 1e0
        stp.mcs_eeparams[1].z_contact_weight = 1e0
        stp.mcs_eeparams[1].z_contact_vel = 0.01
        stp.mcs_eeparams[1].rot_contact_weight = 1e0
        stp.mcs_eeparams[1].rot_contact_vel = 0.05
        stp.mcs_eeparams[1].outside_upper_cop_x_margin = 0.072
        stp.mcs_eeparams[1].outside_lower_cop_x_margin = -0.098
        stp.mcs_eeparams[1].outside_upper_cop_y_margin = 0.063
        stp.mcs_eeparams[1].outside_lower_cop_y_margin = -0.063
        stp.mcs_eeparams[2].is_ik_enable = True
        stp.mcs_eeparams[2].contact_decision_threshold = 5.0
        stp.mcs_eeparams[2].act_force_cutoff_freq = 10.0
        stp.mcs_eeparams[2].act_moment_cutoff_freq = 10.0
        stp.mcs_eeparams[2].contact_type = OpenHRP.StabilizerService.SURFACE
        stp.mcs_eeparams[2].friction_coefficient = 0.001
        stp.mcs_eeparams[2].rotation_friction_coefficient = 0.00001
        stp.mcs_eeparams[2].upper_cop_x_margin = tmp_arm_front_margin
        stp.mcs_eeparams[2].lower_cop_x_margin = -tmp_arm_rear_margin
        stp.mcs_eeparams[2].upper_cop_y_margin = tmp_arm_inside_margin
        stp.mcs_eeparams[2].lower_cop_y_margin = -tmp_arm_outside_margin
        stp.mcs_eeparams[2].max_fz = 200.0
        stp.mcs_eeparams[2].min_fz = 10.0
        stp.mcs_eeparams[2].target_max_fz = 2.0
        stp.mcs_eeparams[2].z_leave_weight = 1e0
        stp.mcs_eeparams[2].z_leavevel_weight = 1e-3
        stp.mcs_eeparams[2].other_leave_weight = 1e-5
        stp.mcs_eeparams[2].wrench_M = 10
        stp.mcs_eeparams[2].wrench_D = 500
        stp.mcs_eeparams[2].wrench_K = 100
        stp.mcs_eeparams[2].pos_interact_weight = 1.0
        stp.mcs_eeparams[2].rot_interact_weight = 1.0
        stp.mcs_eeparams[2].M_p = 10
        stp.mcs_eeparams[2].D_p = 200
        stp.mcs_eeparams[2].K_p = 400
        stp.mcs_eeparams[2].M_r = 5
        stp.mcs_eeparams[2].D_r = 100
        stp.mcs_eeparams[2].K_r = 200
        stp.mcs_eeparams[2].force_gain = [1,1,1]
        stp.mcs_eeparams[2].moment_gain = [1,1,1]
        stp.mcs_eeparams[2].pos_compensation_limit = 0.2
        stp.mcs_eeparams[2].rot_compensation_limit = 0.523599
        stp.mcs_eeparams[2].footorigin_weight = 1e0
        stp.mcs_eeparams[2].z_contact_weight = 1e0
        stp.mcs_eeparams[2].z_contact_vel = 0.01
        stp.mcs_eeparams[2].rot_contact_weight = 1e0
        stp.mcs_eeparams[2].rot_contact_vel = 0.05
        stp.mcs_eeparams[2].outside_upper_cop_x_margin = tmp_arm_front_margin + 0.001
        stp.mcs_eeparams[2].outside_lower_cop_x_margin = -tmp_arm_rear_margin - 0.001
        stp.mcs_eeparams[2].outside_upper_cop_y_margin = tmp_arm_inside_margin + 0.001
        stp.mcs_eeparams[2].outside_lower_cop_y_margin = -tmp_arm_outside_margin - 0.001
        stp.mcs_eeparams[3].is_ik_enable = True
        stp.mcs_eeparams[3].contact_decision_threshold = 5.0
        stp.mcs_eeparams[3].act_force_cutoff_freq = 10.0
        stp.mcs_eeparams[3].act_moment_cutoff_freq = 10.0
        stp.mcs_eeparams[3].contact_type = OpenHRP.StabilizerService.SURFACE
        stp.mcs_eeparams[3].friction_coefficient = 0.001
        stp.mcs_eeparams[3].rotation_friction_coefficient = 0.00001
        stp.mcs_eeparams[3].upper_cop_x_margin = tmp_arm_front_margin
        stp.mcs_eeparams[3].lower_cop_x_margin = -tmp_arm_rear_margin
        stp.mcs_eeparams[3].upper_cop_y_margin = tmp_arm_outside_margin
        stp.mcs_eeparams[3].lower_cop_y_margin = -tmp_arm_inside_margin
        stp.mcs_eeparams[3].max_fz = 200.0
        stp.mcs_eeparams[3].min_fz = 10.0
        stp.mcs_eeparams[3].target_max_fz = 2.0
        stp.mcs_eeparams[3].z_leave_weight = 1e0
        stp.mcs_eeparams[3].z_leavevel_weight = 1e-3
        stp.mcs_eeparams[3].other_leave_weight = 1e-5
        stp.mcs_eeparams[3].wrench_M = 10
        stp.mcs_eeparams[3].wrench_D = 500
        stp.mcs_eeparams[3].wrench_K = 100
        stp.mcs_eeparams[3].pos_interact_weight = 1.0
        stp.mcs_eeparams[3].rot_interact_weight = 1.0
        stp.mcs_eeparams[3].M_p = 10
        stp.mcs_eeparams[3].D_p = 200
        stp.mcs_eeparams[3].K_p = 400
        stp.mcs_eeparams[3].M_r = 5
        stp.mcs_eeparams[3].D_r = 100
        stp.mcs_eeparams[3].K_r = 200
        stp.mcs_eeparams[3].force_gain = [1,1,1]
        stp.mcs_eeparams[3].moment_gain = [1,1,1]
        stp.mcs_eeparams[3].pos_compensation_limit = 0.2
        stp.mcs_eeparams[3].rot_compensation_limit = 0.523599
        stp.mcs_eeparams[3].footorigin_weight = 1e0
        stp.mcs_eeparams[3].z_contact_weight = 1e0
        stp.mcs_eeparams[3].z_contact_vel = 0.01
        stp.mcs_eeparams[3].rot_contact_weight = 1e0
        stp.mcs_eeparams[3].rot_contact_vel = 0.05
        stp.mcs_eeparams[3].outside_upper_cop_x_margin = tmp_arm_front_margin + 0.001
        stp.mcs_eeparams[3].outside_lower_cop_x_margin = -tmp_arm_rear_margin - 0.001
        stp.mcs_eeparams[3].outside_upper_cop_y_margin = tmp_arm_outside_margin + 0.001
        stp.mcs_eeparams[3].outside_lower_cop_y_margin = -tmp_arm_inside_margin - 0.001
        stp.mcs_eeparams[4].is_ik_enable = False
        stp.mcs_eeparams[4].contact_decision_threshold = 10.0
        stp.mcs_eeparams[4].act_force_cutoff_freq = 10.0
        stp.mcs_eeparams[4].act_moment_cutoff_freq = 10.0
        stp.mcs_eeparams[4].contact_type = OpenHRP.StabilizerService.POINT
        stp.mcs_eeparams[4].friction_coefficient = 0.2
        stp.mcs_eeparams[4].rotation_friction_coefficient = 0
        stp.mcs_eeparams[4].upper_cop_x_margin = 0
        stp.mcs_eeparams[4].lower_cop_x_margin = 0
        stp.mcs_eeparams[4].upper_cop_y_margin = 0
        stp.mcs_eeparams[4].lower_cop_y_margin = 0
        stp.mcs_eeparams[4].max_fz = 1200.0
        stp.mcs_eeparams[4].min_fz = 15.0
        stp.mcs_eeparams[4].target_max_fz =5.0
        stp.mcs_eeparams[4].z_leave_weight = 1e0
        stp.mcs_eeparams[4].z_leavevel_weight = 1e-3
        stp.mcs_eeparams[4].other_leave_weight = 1e-5
        stp.mcs_eeparams[4].wrench_M = 10
        stp.mcs_eeparams[4].wrench_D = 500
        stp.mcs_eeparams[4].wrench_K = 100
        stp.mcs_eeparams[4].pos_interact_weight = 1.0
        stp.mcs_eeparams[4].rot_interact_weight = 1.0
        stp.mcs_eeparams[4].M_p = 100
        stp.mcs_eeparams[4].D_p = 2000
        stp.mcs_eeparams[4].K_p = 4000
        stp.mcs_eeparams[4].M_r = 50
        stp.mcs_eeparams[4].D_r = 1000
        stp.mcs_eeparams[4].K_r = 2000
        stp.mcs_eeparams[4].force_gain = [1,1,1]
        stp.mcs_eeparams[4].moment_gain = [1,1,1]
        stp.mcs_eeparams[4].pos_compensation_limit = 0.2
        stp.mcs_eeparams[4].rot_compensation_limit = 0.523599
        stp.mcs_eeparams[4].footorigin_weight = 1e0
        stp.mcs_eeparams[4].z_contact_weight = 1e0
        stp.mcs_eeparams[4].z_contact_vel = 0.01
        stp.mcs_eeparams[4].rot_contact_weight = 0.0
        stp.mcs_eeparams[4].rot_contact_vel = 0.0
        stp.mcs_eeparams[4].outside_upper_cop_x_margin = 0
        stp.mcs_eeparams[4].outside_lower_cop_x_margin = 0
        stp.mcs_eeparams[4].outside_upper_cop_y_margin = 0
        stp.mcs_eeparams[4].outside_lower_cop_y_margin = 0
        stp.mcs_eeparams[5].is_ik_enable = False
        stp.mcs_eeparams[5].contact_decision_threshold = 10.0
        stp.mcs_eeparams[5].act_force_cutoff_freq = 10.0
        stp.mcs_eeparams[5].act_moment_cutoff_freq = 10.0
        stp.mcs_eeparams[5].contact_type = OpenHRP.StabilizerService.POINT
        stp.mcs_eeparams[5].friction_coefficient = 0.2
        stp.mcs_eeparams[5].rotation_friction_coefficient = 0
        stp.mcs_eeparams[5].upper_cop_x_margin = 0
        stp.mcs_eeparams[5].lower_cop_x_margin = 0
        stp.mcs_eeparams[5].upper_cop_y_margin = 0
        stp.mcs_eeparams[5].lower_cop_y_margin = 0
        stp.mcs_eeparams[5].max_fz = 1200.0
        stp.mcs_eeparams[5].min_fz = 15.0
        stp.mcs_eeparams[5].target_max_fz = 5.0
        stp.mcs_eeparams[5].z_leave_weight = 1e0
        stp.mcs_eeparams[5].z_leavevel_weight = 1e-3
        stp.mcs_eeparams[5].other_leave_weight = 1e-5
        stp.mcs_eeparams[5].wrench_M = 10
        stp.mcs_eeparams[5].wrench_D = 500
        stp.mcs_eeparams[5].wrench_K = 100
        stp.mcs_eeparams[5].pos_interact_weight = 1.0
        stp.mcs_eeparams[5].rot_interact_weight = 1.0
        stp.mcs_eeparams[5].M_p = 100
        stp.mcs_eeparams[5].D_p = 2000
        stp.mcs_eeparams[5].K_p = 4000
        stp.mcs_eeparams[5].M_r = 50
        stp.mcs_eeparams[5].D_r = 1000
        stp.mcs_eeparams[5].K_r = 2000
        stp.mcs_eeparams[5].force_gain = [1,1,1]
        stp.mcs_eeparams[5].moment_gain = [1,1,1]
        stp.mcs_eeparams[5].pos_compensation_limit = 0.2
        stp.mcs_eeparams[5].rot_compensation_limit = 0.523599
        stp.mcs_eeparams[5].footorigin_weight = 1e0
        stp.mcs_eeparams[5].z_contact_weight = 1e0
        stp.mcs_eeparams[5].z_contact_vel = 0.01
        stp.mcs_eeparams[5].rot_contact_weight = 0.0
        stp.mcs_eeparams[5].rot_contact_vel = 0.0
        stp.mcs_eeparams[5].outside_upper_cop_x_margin = 0
        stp.mcs_eeparams[5].outside_lower_cop_x_margin = 0
        stp.mcs_eeparams[5].outside_upper_cop_y_margin = 0
        stp.mcs_eeparams[5].outside_lower_cop_y_margin = 0
        stp.mcs_eeparams[6].is_ik_enable = False
        stp.mcs_eeparams[6].contact_decision_threshold = 10.0
        stp.mcs_eeparams[6].act_force_cutoff_freq = 10.0
        stp.mcs_eeparams[6].act_moment_cutoff_freq = 10.0
        stp.mcs_eeparams[6].contact_type = OpenHRP.StabilizerService.SURFACE
        stp.mcs_eeparams[6].friction_coefficient = 0.3
        stp.mcs_eeparams[6].rotation_friction_coefficient = 0.04
        stp.mcs_eeparams[6].upper_cop_x_margin = 0.090
        stp.mcs_eeparams[6].lower_cop_x_margin = -0.090
        stp.mcs_eeparams[6].upper_cop_y_margin = 0.054
        stp.mcs_eeparams[6].lower_cop_y_margin = -0.054
        stp.mcs_eeparams[6].max_fz = 1200.0
        stp.mcs_eeparams[6].min_fz = 20.0
        stp.mcs_eeparams[6].target_max_fz = 5.0
        stp.mcs_eeparams[6].z_leave_weight = 1e0
        stp.mcs_eeparams[6].z_leavevel_weight = 1e-3
        stp.mcs_eeparams[6].other_leave_weight = 1e-5
        stp.mcs_eeparams[6].wrench_M = 10
        stp.mcs_eeparams[6].wrench_D = 500
        stp.mcs_eeparams[6].wrench_K = 100
        stp.mcs_eeparams[6].pos_interact_weight = 1.0
        stp.mcs_eeparams[6].rot_interact_weight = 1.0
        stp.mcs_eeparams[6].M_p = 100
        stp.mcs_eeparams[6].D_p = 2000
        stp.mcs_eeparams[6].K_p = 4000
        stp.mcs_eeparams[6].M_r = 50
        stp.mcs_eeparams[6].D_r = 1000
        stp.mcs_eeparams[6].K_r = 2000
        stp.mcs_eeparams[6].force_gain = [1,1,1]
        stp.mcs_eeparams[6].moment_gain = [1,1,1]
        stp.mcs_eeparams[6].pos_compensation_limit = 0.2
        stp.mcs_eeparams[6].rot_compensation_limit = 0.523599
        stp.mcs_eeparams[6].footorigin_weight = 1e0
        stp.mcs_eeparams[6].z_contact_weight = 1e0
        stp.mcs_eeparams[6].z_contact_vel = 0.01
        stp.mcs_eeparams[6].rot_contact_weight = 1e0
        stp.mcs_eeparams[6].rot_contact_vel = 0.05
        stp.mcs_eeparams[6].outside_upper_cop_x_margin = 0.098
        stp.mcs_eeparams[6].outside_lower_cop_x_margin = -0.098
        stp.mcs_eeparams[6].outside_upper_cop_y_margin = 0.063
        stp.mcs_eeparams[6].outside_lower_cop_y_margin = -0.063
        stp.mcs_eeparams[7].is_ik_enable = False
        stp.mcs_eeparams[7].contact_decision_threshold = 10.0
        stp.mcs_eeparams[7].act_force_cutoff_freq = 10.0
        stp.mcs_eeparams[7].act_moment_cutoff_freq = 10.0
        stp.mcs_eeparams[7].contact_type = OpenHRP.StabilizerService.SURFACE
        stp.mcs_eeparams[7].friction_coefficient = 0.3
        stp.mcs_eeparams[7].rotation_friction_coefficient = 0.04
        stp.mcs_eeparams[7].upper_cop_x_margin = 0.090
        stp.mcs_eeparams[7].lower_cop_x_margin = -0.090
        stp.mcs_eeparams[7].upper_cop_y_margin = 0.054
        stp.mcs_eeparams[7].lower_cop_y_margin = -0.054
        stp.mcs_eeparams[7].max_fz = 1200.0
        stp.mcs_eeparams[7].min_fz = 20.0
        stp.mcs_eeparams[7].target_max_fz = 5.0
        stp.mcs_eeparams[7].z_leave_weight = 1e0
        stp.mcs_eeparams[7].z_leavevel_weight = 1e-3
        stp.mcs_eeparams[7].other_leave_weight = 1e-5
        stp.mcs_eeparams[7].wrench_M = 10
        stp.mcs_eeparams[7].wrench_D = 500
        stp.mcs_eeparams[7].wrench_K = 100
        stp.mcs_eeparams[7].pos_interact_weight = 1.0
        stp.mcs_eeparams[7].rot_interact_weight = 1.0
        stp.mcs_eeparams[7].M_p = 100
        stp.mcs_eeparams[7].D_p = 2000
        stp.mcs_eeparams[7].K_p = 4000
        stp.mcs_eeparams[7].M_r = 50
        stp.mcs_eeparams[7].D_r = 1000
        stp.mcs_eeparams[7].K_r = 2000
        stp.mcs_eeparams[7].force_gain = [1,1,1]
        stp.mcs_eeparams[7].moment_gain = [1,1,1]
        stp.mcs_eeparams[7].pos_compensation_limit = 0.2
        stp.mcs_eeparams[7].rot_compensation_limit = 0.523599
        stp.mcs_eeparams[7].footorigin_weight = 1e0
        stp.mcs_eeparams[7].z_contact_weight = 1e0
        stp.mcs_eeparams[7].z_contact_vel = 0.01
        stp.mcs_eeparams[7].rot_contact_weight = 1e0
        stp.mcs_eeparams[7].rot_contact_vel = 0.05
        stp.mcs_eeparams[7].outside_upper_cop_x_margin = 0.098
        stp.mcs_eeparams[7].outside_lower_cop_x_margin = -0.098
        stp.mcs_eeparams[7].outside_upper_cop_y_margin = 0.063
        stp.mcs_eeparams[7].outside_lower_cop_y_margin = -0.063
        self.st_svc.setParameter(stp)
        #self.st_svc.setReferenceJoints(["CHEST_JOINT0","CHEST_JOINT1","HEAD_JOINT0","HEAD_JOINT1","LARM_JOINT0","LARM_JOINT1","LARM_JOINT2","LARM_JOINT3","LARM_JOINT4","LARM_JOINT5","LARM_JOINT6","LARM_JOINT7","RARM_JOINT0","RARM_JOINT1","RARM_JOINT2","RARM_JOINT3","RARM_JOINT4","RARM_JOINT5","RARM_JOINT6","RARM_JOINT7"])
        self.st_svc.setReferenceJoints(["HEAD_JOINT0","HEAD_JOINT1","LARM_JOINT7","RARM_JOINT7"])
        self.st_svc.setReferenceJoints(["RLEG_JOINT6","LLEG_JOINT6"])
        # GG parameters
        gg=self.abc_svc.getGaitGeneratorParam()[1]
        gg.default_step_time=1.1
        gg.default_double_support_ratio=0.32
        #gg.swing_trajectory_delay_time_offset=0.35
        gg.swing_trajectory_delay_time_offset=0.2
        gg.stair_trajectory_way_point_offset=[0.03, 0.0, 0.0]
        #  Orbit time parameters for delayhoffarbib (simultaneous xy and z landing)
        #gg.swing_trajectory_final_distance_weight=3.0
        #gg.swing_trajectory_time_offset_xy2z=0.0
        #  Orbit time parameters for delayhoffarbib (xy is faster than z)
        gg.swing_trajectory_final_distance_weight=1.5
        gg.swing_trajectory_time_offset_xy2z=0.1 # [s]
        #
        gg.default_orbit_type = OpenHRP.AutoBalancerService.CYCLOIDDELAY
        gg.toe_pos_offset_x = 1e-3*142.869;
        gg.heel_pos_offset_x = 1e-3*-105.784;
        gg.toe_zmp_offset_x = 1e-3*79.411;
        gg.heel_zmp_offset_x = 1e-3*-105.784;
        gg.use_toe_joint = True
        self.abc_svc.setGaitGeneratorParam(gg)
        # Estop
        esp=self.es_svc.getEmergencyStopperParam()[1]
        esp.default_recover_time=10.0 # [s]
        esp.default_retrieve_time=1.0 # [s]
        self.es_svc.setEmergencyStopperParam(esp)

    def setStAbcParametershrp2016c (self):
        # ABC parameters
        abcp=self.abc_svc.getAutoBalancerParam()[1]
        #abcp.default_zmp_offsets = [[0.015, 0.01, 0], [0.015, -0.01, 0], [0, 0, 0], [0, 0, 0]]
        abcp.default_zmp_offsets = [[0.010, 0.01, 0], [0.010, -0.01, 0], [0, 0, 0], [0, 0, 0]]
        #abcp.default_zmp_offsets = [[0.01, 0.035, 0], [0.01, -0.035, 0], [0, 0, 0], [0, 0, 0]]
        self.abc_svc.setAutoBalancerParam(abcp)
        # ST parameters
        stp=self.st_svc.getParameter()
        stp.st_algorithm=OpenHRP.StabilizerService.EEFMQPCOP
        #   eefm st params
        stp.eefm_body_attitude_control_gain=[1.5, 1.5]
        stp.eefm_body_attitude_control_time_const=[10000, 10000]
        # EEFM parameters for 4 limbs
        stp.eefm_rot_damping_gain = [[20*1.1, 20*1.1, 1e5]]*4
        stp.eefm_pos_damping_gain = [[3500*50, 3500*50, 3500*1.1]]*4
        stp.eefm_rot_time_const = [[1.5, 1.5, 1.5]]*4
        stp.eefm_pos_time_const_support = [[1.5, 1.5, 1.5]]*4
        stp.eefm_swing_pos_damping_gain=stp.eefm_pos_damping_gain[0]
        stp.eefm_swing_rot_damping_gain=stp.eefm_rot_damping_gain[0]
        stp.eefm_use_swing_damping=True
        stp.eefm_wrench_alpha_blending = 0.75
        stp.eefm_pos_time_const_swing=0.08
        stp.eefm_pos_transition_time=0.01
        stp.eefm_pos_margin_time=0.02
        stp.eefm_zmp_delay_time_const=[0.055, 0.055]
        stp.eefm_cogvel_cutoff_freq=6.0
        #   mechanical foot edge
        #stp.eefm_leg_inside_margin=0.065
        #stp.eefm_leg_front_margin=0.140
        #stp.eefm_leg_rear_margin=0.105
        #   margined foot edge
        tmp_leg_inside_margin=0.062
        tmp_leg_outside_margin=0.062
        tmp_leg_front_margin=0.130
        tmp_leg_rear_margin=0.095
        stp.eefm_leg_inside_margin=tmp_leg_inside_margin
        stp.eefm_leg_outside_margin=tmp_leg_outside_margin
        stp.eefm_leg_front_margin=tmp_leg_front_margin
        stp.eefm_leg_rear_margin=tmp_leg_rear_margin
        rleg_vertices = [OpenHRP.StabilizerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, tmp_leg_inside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, -1*tmp_leg_outside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, -1*tmp_leg_outside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, tmp_leg_inside_margin])]
        lleg_vertices = [OpenHRP.StabilizerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, tmp_leg_outside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, -1*tmp_leg_inside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, -1*tmp_leg_inside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, tmp_leg_outside_margin])]
        rarm_vertices = rleg_vertices
        larm_vertices = lleg_vertices
        stp.eefm_support_polygon_vertices_sequence = map (lambda x : OpenHRP.StabilizerService.SupportPolygonVertices(vertices=x), [rleg_vertices, lleg_vertices, rarm_vertices, larm_vertices])
        #   tpcc st params
        stp.k_tpcc_p=[2.0, 2.0]
        stp.k_tpcc_x=[5.0, 5.0]
        stp.k_brot_p=[0.0, 0.0]
        stp.k_brot_tc=[0.1, 0.1]
        #   cog height = 800[mm], alpha = -13.0, beta = -4.0, time_const = 0.04[s]
        #stp.eefm_k1=[-1.41413,-1.41413]
        #stp.eefm_k2=[-0.403901,-0.403901]
        #stp.eefm_k3=[-0.179953,-0.179953]
        stp.eefm_k1=[-1.272861, -1.272861]
        stp.eefm_k2=[-0.36367379999999999, -0.36367379999999999]
        stp.eefm_k3=[-0.16200000000000001, -0.16200000000000001]
        # for estop
        stp.emergency_check_mode=OpenHRP.StabilizerService.CP;
        stp.cp_check_margin=[50*1e-3, 45*1e-3, 0, 100*1e-3];
        # for swing
        stp.eefm_swing_pos_spring_gain = [[1]*3, [1]*3, [0]*3, [0]*3]
        stp.eefm_swing_rot_spring_gain = [[1]*3, [1]*3, [0]*3, [0]*3]
        self.st_svc.setParameter(stp)
        # GG parameters
        gg=self.abc_svc.getGaitGeneratorParam()[1]
        gg.default_step_time=1.1
        gg.default_double_support_ratio=0.32
        #gg.swing_trajectory_delay_time_offset=0.35
        gg.swing_trajectory_delay_time_offset=0.2
        gg.stair_trajectory_way_point_offset=[0.03, 0.0, 0.0]
        #  Orbit time parameters for delayhoffarbib (simultaneous xy and z landing)
        #gg.swing_trajectory_final_distance_weight=3.0
        #gg.swing_trajectory_time_offset_xy2z=0.0
        #  Orbit time parameters for delayhoffarbib (xy is faster than z)
        gg.swing_trajectory_final_distance_weight=1.5
        gg.swing_trajectory_time_offset_xy2z=0.1 # [s]
        #
        gg.default_orbit_type = OpenHRP.AutoBalancerService.CYCLOIDDELAY
        gg.toe_pos_offset_x = 1e-3*142.869;
        gg.heel_pos_offset_x = 1e-3*-105.784;
        gg.toe_zmp_offset_x = 1e-3*79.411;
        gg.heel_zmp_offset_x = 1e-3*-105.784;
        gg.use_toe_joint = True
        self.abc_svc.setGaitGeneratorParam(gg)
        # Estop
        esp=self.es_svc.getEmergencyStopperParam()[1]
        esp.default_recover_time=10.0 # [s]
        esp.default_retrieve_time=1.0 # [s]
        self.es_svc.setEmergencyStopperParam(esp)

    def setStAbcParametershrp2007c (self):
        # ABC parameters
        abcp=self.abc_svc.getAutoBalancerParam()[1]
        #abcp.default_zmp_offsets = [[0.015, 0.01, 0], [0.015, -0.01, 0], [0, 0, 0], [0, 0, 0]]
        abcp.default_zmp_offsets = [[0.010, 0.01, 0], [0.010, -0.01, 0], [0, 0, 0], [0, 0, 0]]
        #abcp.default_zmp_offsets = [[0.01, 0.035, 0], [0.01, -0.035, 0], [0, 0, 0], [0, 0, 0]]
        self.abc_svc.setAutoBalancerParam(abcp)
        # ST parameters
        stp=self.st_svc.getParameter()
        stp.st_algorithm=OpenHRP.StabilizerService.EEFMQPCOP
        #   eefm st params
        stp.eefm_body_attitude_control_gain=[1.5, 1.5]
        stp.eefm_body_attitude_control_time_const=[10000, 10000]
        # EEFM parameters for 4 limbs
        stp.eefm_rot_damping_gain = [[20*1.1, 20*1.1, 1e5]]*4
        stp.eefm_pos_damping_gain = [[3500*50, 3500*50, 3500*1.1]]*4
        stp.eefm_rot_time_const = [[1.5, 1.5, 1.5]]*4
        stp.eefm_pos_time_const_support = [[1.5, 1.5, 1.5]]*4
        stp.eefm_swing_pos_damping_gain=stp.eefm_pos_damping_gain[0]
        stp.eefm_swing_rot_damping_gain=stp.eefm_rot_damping_gain[0]
        stp.eefm_use_swing_damping=True
        stp.eefm_wrench_alpha_blending = 0.7
        stp.eefm_pos_time_const_swing=0.08
        stp.eefm_pos_transition_time=0.01
        stp.eefm_pos_margin_time=0.02
        stp.eefm_zmp_delay_time_const=[0.055, 0.055]
        stp.eefm_cogvel_cutoff_freq=6.0
        #   mechanical foot edge
        #stp.eefm_leg_inside_margin=0.07
        #stp.eefm_leg_front_margin=0.135
        #stp.eefm_leg_rear_margin=0.105
        #   margined foot edge
        tmp_leg_inside_margin=0.062
        tmp_leg_outside_margin=0.062
        tmp_leg_front_margin=0.125
        tmp_leg_rear_margin=0.095
        stp.eefm_leg_inside_margin=tmp_leg_inside_margin
        stp.eefm_leg_outside_margin=tmp_leg_outside_margin
        stp.eefm_leg_front_margin=tmp_leg_front_margin
        stp.eefm_leg_rear_margin=tmp_leg_rear_margin
        rleg_vertices = [OpenHRP.StabilizerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, tmp_leg_inside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, -1*tmp_leg_outside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, -1*tmp_leg_outside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, tmp_leg_inside_margin])]
        lleg_vertices = [OpenHRP.StabilizerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, tmp_leg_outside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, -1*tmp_leg_inside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, -1*tmp_leg_inside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, tmp_leg_outside_margin])]
        rarm_vertices = rleg_vertices
        larm_vertices = lleg_vertices
        stp.eefm_support_polygon_vertices_sequence = map (lambda x : OpenHRP.StabilizerService.SupportPolygonVertices(vertices=x), [rleg_vertices, lleg_vertices, rarm_vertices, larm_vertices])
        #   tpcc st params
        stp.k_tpcc_p=[2.0, 2.0]
        stp.k_tpcc_x=[5.0, 5.0]
        stp.k_brot_p=[0.0, 0.0]
        stp.k_brot_tc=[0.1, 0.1]
        #   cog height = 800[mm], alpha = -13.0, beta = -4.0, time_const = 0.04[s]
        #stp.eefm_k1=[-1.41413,-1.41413]
        #stp.eefm_k2=[-0.403901,-0.403901]
        #stp.eefm_k3=[-0.179953,-0.179953]
        stp.eefm_k1=[-1.272861, -1.272861]
        stp.eefm_k2=[-0.36367379999999999, -0.36367379999999999]
        stp.eefm_k3=[-0.16200000000000001, -0.16200000000000001]
        # for estop
        stp.emergency_check_mode=OpenHRP.StabilizerService.CP;
        stp.cp_check_margin=[50*1e-3, 45*1e-3, 0, 100*1e-3];
        # for swing
        stp.eefm_swing_pos_spring_gain = [[1]*3, [1]*3, [0]*3, [0]*3]
        stp.eefm_swing_rot_spring_gain = [[1]*3, [1]*3, [0]*3, [0]*3]
        self.st_svc.setParameter(stp)
        # GG parameters
        gg=self.abc_svc.getGaitGeneratorParam()[1]
        gg.default_step_time=1.1
        gg.default_double_support_ratio=0.32
        #gg.swing_trajectory_delay_time_offset=0.35
        gg.swing_trajectory_delay_time_offset=0.2
        gg.stair_trajectory_way_point_offset=[0.03, 0.0, 0.0]
        #  Orbit time parameters for delayhoffarbib (simultaneous xy and z landing)
        #gg.swing_trajectory_final_distance_weight=3.0
        #gg.swing_trajectory_time_offset_xy2z=0.0
        #  Orbit time parameters for delayhoffarbib (xy is faster than z)
        gg.swing_trajectory_final_distance_weight=1.5
        gg.swing_trajectory_time_offset_xy2z=0.1 # [s]
        #
        gg.default_orbit_type = OpenHRP.AutoBalancerService.CYCLOIDDELAY
        gg.toe_pos_offset_x = 1e-3*137.525;
        gg.heel_pos_offset_x = 1e-3*-106.925;
        gg.toe_zmp_offset_x = 1e-3*137.525;
        gg.heel_zmp_offset_x = 1e-3*-106.925;
        self.abc_svc.setGaitGeneratorParam(gg)
        # Estop
        esp=self.es_svc.getEmergencyStopperParam()[1]
        esp.default_recover_time=10.0 # [s]
        esp.default_retrieve_time=1.0 # [s]
        self.es_svc.setEmergencyStopperParam(esp)

    def setResetPose(self):
        self.seq_svc.setJointAnglesSequenceFull([self.hrp2ResetPose()], [], [[0]*len(self.hrp2ResetPose())], [[0]*3], [[0]*3], [[0]*3], [[0]*3], [[0]*6*4], [[1.0]*2 + [0]*6 + [0]*8], [5.0])
        #self.seq_svc.setJointAngles(self.hrp2ResetPose(), 5.0)

    def setResetManipPose(self):
        self.seq_svc.setJointAnglesSequenceFull([self.hrp2ResetManipPose()], [], [[0]*len(self.hrp2ResetManipPose())], [[0]*3], [[0]*3], [[0]*3], [[0]*3], [[0]*6*4], [[1.0]*2 + [0]*6 + [0]*8], [5.0])
        #self.seq_svc.setJointAngles(self.hrp2ResetManipPose(), 5.0)

    def setInitPose(self):
        self.seq_svc.setJointAnglesSequenceFull([self.hrp2InitPose()], [], [[0]*len(self.hrp2InitPose())], [[0]*3], [[0]*3], [[0]*3], [[0]*3], [[0]*6*4], [[1.0]*2 + [0]*6 + [0]*8], [5.0])
        #self.seq_svc.setJointAngles(self.hrp2InitPose(), 5.0)

    def loadForceMomentOffsetFile (self):
        import rospkg
        if self.ROBOT_NAME == "HRP2JSKNT":
            self.rmfo_svc.loadForceMomentOffsetParams(rospkg.RosPack().get_path('hrpsys_ros_bridge_tutorials')+"/models/hand_force_calib_offset_HRP2JSKNT")
        elif self.ROBOT_NAME == "HRP2JSKNTS":
            self.rmfo_svc.loadForceMomentOffsetParams(rospkg.RosPack().get_path('hrpsys_ros_bridge_tutorials')+"/models/hand_force_calib_offset_HRP2JSKNTS")
        elif self.ROBOT_NAME == "HRP2JSK":
            self.rmfo_svc.loadForceMomentOffsetParams(rospkg.RosPack().get_path('hrpsys_ros_bridge_tutorials')+"/models/hand_force_calib_offset_thumb_60deg_HRP2JSK")
        else:
            print "No force moment offset file"

    def __init__(self, robotname=""):
        self.ROBOT_NAME = robotname
        HrpsysConfigurator.__init__(self)
        self.defJointGroups()
