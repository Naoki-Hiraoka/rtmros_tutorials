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
            return [0]*len(self.hrp2ResetPose())
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
        stp.mcs_k1 = 0.2
        stp.mcs_k2 = 0.0
        stp.mcs_k3 = 0.0
        stp.mcs_contact_vel = 0.01
        stp.mcs_joint_torque_distribution_weight = [0.006569, 0.001059, 0.000639, 0.000518, 0.001883, 0.0061, 0.042297, 0.006569, 0.001059, 0.000639, 0.000518, 0.001883, 0.0061, 0.042297, 0.001525, 0.000422, 0.06464, 0.027769, 0.001269, 0.002507, 0.020466, 0.003355, 0.03075, 0.027769, 0.043767, 0.06464, 0.001269, 0.002507, 0.020466, 0.003355, 0.03075, 0.027769, 0.043767, 0.06464]
        stp.mcs_equality_weight = [100,100,100,100,100,100]
        stp.mcs_rot_damping_gain = [[35,35,35*10]] * 4
        stp.mcs_rot_time_const = [[15,15,15]] * 4
        stp.mcs_pos_damping_gain = [[3500,3500,3500]] * 4
        stp.mcs_pos_time_const = [[15,15,15]] * 4
        stp.mcs_contacteeforiginweight = [1.0,1.0,0.001,0.001]
        stp.mcs_ik_optional_weight_vector = [1.0,1.0,1.0,1.0,1.0,1.0,0.0] + [1.0,1.0,1.0,1.0,1.0,1.0,0.0] + [1.0,1.0] + [0.0,0.0] + [1.0,1.0,1.0,1.0,1.0,1.0,1.0,0.0] + [1.0,1.0,1.0,1.0,1.0,1.0,1.0,0.0]
        stp.mcs_ceeparams[0].friction_coefficient = 0.3
        stp.mcs_ceeparams[0].rotation_friction_coefficient = 0.3
        stp.mcs_ceeparams[0].upper_cop_x_margin = tmp_leg_front_margin
        stp.mcs_ceeparams[0].lower_cop_x_margin = -tmp_leg_rear_margin
        stp.mcs_ceeparams[0].upper_cop_y_margin = tmp_leg_inside_margin
        stp.mcs_ceeparams[0].lower_cop_y_margin = -tmp_leg_outside_margin
        stp.mcs_ceeparams[0].min_fz = 0.0
        stp.mcs_ceeparams[0].ee_forcemoment_distribution_weight=[1e-10,1e-10,1e-10,1e-4,1e-4,1e-10]
        stp.mcs_ceeparams[1].friction_coefficient = 0.3
        stp.mcs_ceeparams[1].rotation_friction_coefficient = 0.3
        stp.mcs_ceeparams[1].upper_cop_x_margin = 0.087
        stp.mcs_ceeparams[1].lower_cop_x_margin = -0.078
        stp.mcs_ceeparams[1].upper_cop_y_margin = 0.060
        stp.mcs_ceeparams[1].lower_cop_y_margin = -0.060
        stp.mcs_ceeparams[1].min_fz = 20.0
        stp.mcs_ceeparams[1].ee_forcemoment_distribution_weight=[1e-10,1e-10,1e-10,1e-4,1e-4,1e-10]
        stp.mcs_ceeparams[2].friction_coefficient = 0.3
        stp.mcs_ceeparams[2].rotation_friction_coefficient = 0.3
        stp.mcs_ceeparams[2].upper_cop_x_margin = tmp_leg_front_margin
        stp.mcs_ceeparams[2].lower_cop_x_margin = -tmp_leg_rear_margin
        stp.mcs_ceeparams[2].upper_cop_y_margin = tmp_leg_outside_margin
        stp.mcs_ceeparams[2].lower_cop_y_margin = -tmp_leg_inside_margin
        stp.mcs_ceeparams[2].min_fz = 0.0
        stp.mcs_ceeparams[2].ee_forcemoment_distribution_weight=[1e-10,1e-10,1e-10,1e-4,1e-4,1e-10]
        stp.mcs_ceeparams[3].friction_coefficient = 0.3
        stp.mcs_ceeparams[3].rotation_friction_coefficient = 0.3
        stp.mcs_ceeparams[3].upper_cop_x_margin = 0.087
        stp.mcs_ceeparams[3].lower_cop_x_margin = -0.078
        stp.mcs_ceeparams[3].upper_cop_y_margin = 0.060
        stp.mcs_ceeparams[3].lower_cop_y_margin = -0.060
        stp.mcs_ceeparams[3].min_fz = 20.0
        stp.mcs_ceeparams[3].ee_forcemoment_distribution_weight=[1e-10,1e-10,1e-10,1e-4,1e-4,1e-10]
        stp.mcs_ceeparams[4].friction_coefficient = 0.3
        stp.mcs_ceeparams[4].rotation_friction_coefficient = 0.3
        stp.mcs_ceeparams[4].upper_cop_x_margin = tmp_arm_front_margin
        stp.mcs_ceeparams[4].lower_cop_x_margin = -tmp_arm_rear_margin
        stp.mcs_ceeparams[4].upper_cop_y_margin = tmp_arm_inside_margin
        stp.mcs_ceeparams[4].lower_cop_y_margin = -tmp_arm_outside_margin
        stp.mcs_ceeparams[4].min_fz = 15.0
        stp.mcs_ceeparams[4].ee_forcemoment_distribution_weight=[1e-10,1e-10,1e-10,1e-4,1e-4,1e-10]
        stp.mcs_ceeparams[5].friction_coefficient = 0.3
        stp.mcs_ceeparams[5].rotation_friction_coefficient = 0.3
        stp.mcs_ceeparams[5].upper_cop_x_margin = tmp_arm_front_margin
        stp.mcs_ceeparams[5].lower_cop_x_margin = -tmp_arm_rear_margin
        stp.mcs_ceeparams[5].upper_cop_y_margin = tmp_arm_outside_margin
        stp.mcs_ceeparams[5].lower_cop_y_margin = -tmp_arm_inside_margin
        stp.mcs_ceeparams[5].min_fz = 15.0
        stp.mcs_ceeparams[5].ee_forcemoment_distribution_weight=[1e-10,1e-10,1e-10,1e-4,1e-4,1e-10]
        stp.mcs_eeparams[0].contact_decision_threshold = 10
        stp.mcs_eeparams[1].contact_decision_threshold = 10
        stp.mcs_eeparams[2].contact_decision_threshold = 5
        stp.mcs_eeparams[3].contact_decision_threshold = 5
        stp.mcs_impedance_params[0].force_gain = [0,0,0]
        stp.mcs_impedance_params[0].moment_gain = [0,0,0]
        stp.mcs_impedance_params[0].M_p = 0
        stp.mcs_impedance_params[0].D_p = 174533
        stp.mcs_impedance_params[0].K_p = 116667
        stp.mcs_impedance_params[0].M_r = 0
        stp.mcs_impedance_params[0].D_r = 99733
        stp.mcs_impedance_params[0].K_r = 66667
        stp.mcs_impedance_params[1].force_gain = [0,0,0]
        stp.mcs_impedance_params[1].moment_gain = [0,0,0]
        stp.mcs_impedance_params[1].M_p = 0
        stp.mcs_impedance_params[1].D_p = 174533
        stp.mcs_impedance_params[1].K_p = 116667
        stp.mcs_impedance_params[1].M_r = 0
        stp.mcs_impedance_params[1].D_r = 99733
        stp.mcs_impedance_params[1].K_r = 66667
        stp.mcs_impedance_params[2].force_gain = [0,0,0]
        stp.mcs_impedance_params[2].moment_gain = [0,0,0]
        stp.mcs_impedance_params[2].M_p = 10
        stp.mcs_impedance_params[2].D_p = 200
        stp.mcs_impedance_params[2].K_p = 400
        stp.mcs_impedance_params[2].M_r = 5
        stp.mcs_impedance_params[2].D_r = 100
        stp.mcs_impedance_params[2].K_r = 200
        stp.mcs_impedance_params[3].force_gain = [0,0,0]
        stp.mcs_impedance_params[3].moment_gain = [0,0,0]
        stp.mcs_impedance_params[3].M_p = 10
        stp.mcs_impedance_params[3].D_p = 200
        stp.mcs_impedance_params[3].K_p = 400
        stp.mcs_impedance_params[3].M_r = 5
        stp.mcs_impedance_params[3].D_r = 100
        stp.mcs_impedance_params[3].K_r = 200
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
        self.seq_svc.setJointAngles(self.hrp2ResetPose(), 5.0)

    def setResetManipPose(self):
        self.seq_svc.setJointAngles(self.hrp2ResetManipPose(), 5.0)

    def setInitPose(self):
        self.seq_svc.setJointAngles(self.hrp2InitPose(), 5.0)

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
