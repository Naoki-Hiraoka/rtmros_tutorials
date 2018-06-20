(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :hrp2jsknts "package://hrpsys_ros_bridge_tutorials/models/hrp2jsknts.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-utils.l"))
  (require :hrp2jsknts-utils "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-utils.l"))
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2-common-interface.l")
(if (ros::resolve-ros-path "package://jsk_hrp2_ros_bridge")
    (ros::load-ros-manifest "jsk_hrp2_ros_bridge"))

(defclass hrp2jsknts-interface
  :super hrp2-common-interface
  :slots ())
(defmethod hrp2jsknts-interface
  (:init (&rest args)
	 ;; add controller methods
	 (dolist (limb '(:rarm :larm :rleg :lleg :head :torso))
	   (eval `(defmethod ,(send (class self) :name)
		    (,(read-from-string (format nil "~A-controller" limb))
		     ()
		     (list
		      (list
		       (cons :group-name ,(string-downcase limb))
		       (cons :controller-action ,(format nil "~A_controller/follow_joint_trajectory_action" (string-downcase limb)))
		       (cons :controller-state ,(format nil "~A_controller/state" (string-downcase limb)))
		       (cons :action-type control_msgs::FollowJointTrajectoryAction)
		       (cons :joint-names (list ,@(send-all (send (instance hrp2jsknts-robot :init) limb :joint-list) :name))))
		      )))))

	 (prog1
	   (send-super* :init :robot hrp2jsknts-robot args)
	   ;; add controller
	   (dolist (limb '(:rarm :larm :rleg :lleg :head :torso))
	     (send self :add-controller (read-from-string (format nil "~A-controller" limb)) :joint-enable-check t :create-actions nil)))
	 )
  (:default-controller
   ()
   (append
    (send self :larm-controller)
    (send self :rarm-controller)
    (send self :lleg-controller)
    (send self :rleg-controller)
    (send self :head-controller)
    (send self :torso-controller)))
  )

(defun hrp2jsknts-init (&rest args)
  (if (not (boundp '*ri*))
      (setq *ri* (instance* hrp2jsknts-interface :init args)))
  (if (not (boundp '*hrp2jsknts*))
      (setq *hrp2jsknts* (instance hrp2jsknts-robot :init))))
