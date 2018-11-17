#!/usr/bin/env roseus

;; <launch>
;;   <node pkg="hrpsys_gazebo_tutorials" type="p3d_to_odom.l" name="zed_odom">
;;     <remap from="input" to="/zed/odom_raw"/>
;;     <remap from="output" to="/zed/odom"/>
;;     <rosparam>
;;       xyzoffset: [-0.13,0.0,0.21]
;;       rpyoffset: [0.0,0.0,3.14]
;;       childframeid: zed_odom_frame
;;     </rosparam>
;;   </node>
;; </launch>

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "nav_msgs")

(ros::roseus "convert_odom_frame")
;;rosparam の単位は[m][rad]
(setq *offset* (make-coords :pos (scale 1000 (concatenate float-vector (if (ros::has-param "~xyzoffset")
									    (ros::get-param "~xyzoffset")
									  '(0 0 0))))
			    :rpy (reverse (concatenate float-vector (if (ros::has-param "~rpyoffset")
									(ros::get-param "~rpyoffset")
								   '(0 0 0))))))
(setq *child-frame-id* (if (ros::has-param "~childframeid")
			   (ros::get-param "~childframeid")
			 nil))
(setq *initialpose* nil)

(print *offset*)
(print (scale 1000 (concatenate float-vector (if (ros::has-param "~xyzoffset")
									    (ros::get-param "~xyzoffset")
					       '(0 0 0)))))
(print (reverse (concatenate float-vector (if (ros::has-param "~rpyoffset")
									(ros::get-param "~rpyoffset")
					    '(0 0 0)))))
(print (ros::get-param "~rpyoffset"))

(defun odom-cb (msg)
  (let ((newmsg (instance nav_msgs::Odometry :init))
	(msg-coords (send (ros::tf-pose->coords (send msg :pose :pose)) :transform *offset* :local))
	)
    (unless *initialpose*
      (setq *initialpose* msg-coords))
    (send newmsg :header (send msg :header))
    (send newmsg :child_frame_id (if *child-frame-id*
				     *child-frame-id*
				   (send msg :child_frame_id)))
    (send newmsg :pose :covariance (send msg :pose :covariance))
    (send newmsg :twist :covariance (send msg :twist :covariance))

    (send newmsg :pose :pose (ros::coords->tf-pose (send *initialpose* :transformation msg-coords :local)))
    (send newmsg :twist :twist :linear (ros::pos->tf-translation
					(let* ((lin (send msg :twist :twist :linear))
					      (vel (scale 1000 (float-vector (send lin :x) (send lin :y) (send lin :z))))
					      (ang (send msg :twist :twist :angular))
					      (omega (float-vector (send ang :x) (send ang :y) (send ang :z))))
					  (transform (transpose (send *offset* :rot)) (v+ vel (v* omega (send *offset* :pos)))))))
    (send newmsg :twist :twist :angular (ros::pos->tf-translation
					 (let* ((ang (send msg :twist :twist :angular))
						(omega (float-vector (send ang :x) (send ang :y) (send ang :z))))
					   (scale 1000 (transform (transpose (send *offset* :rot)) omega)))))
    (ros::publish "output" newmsg)
    )
  )

(ros::advertise "output" nav_msgs::Odometry 1)
(ros::subscribe "input" nav_msgs::Odometry #'odom-cb)

(while (ros::ok)
  (ros::spin-once)
 )

(ros::roseus "shutdown")
(exit)
