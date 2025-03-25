;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::KinematicLimitsList)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'KinematicLimitsList (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::KINEMATICLIMITSLIST")
  (make-package "KORTEX_DRIVER::KINEMATICLIMITSLIST"))

(in-package "ROS")
;;//! \htmlinclude KinematicLimitsList.msg.html


(defclass kortex_driver::KinematicLimitsList
  :super ros::object
  :slots (_kinematic_limits_list ))

(defmethod kortex_driver::KinematicLimitsList
  (:init
   (&key
    ((:kinematic_limits_list __kinematic_limits_list) ())
    )
   (send-super :init)
   (setq _kinematic_limits_list __kinematic_limits_list)
   self)
  (:kinematic_limits_list
   (&rest __kinematic_limits_list)
   (if (keywordp (car __kinematic_limits_list))
       (send* _kinematic_limits_list __kinematic_limits_list)
     (progn
       (if __kinematic_limits_list (setq _kinematic_limits_list (car __kinematic_limits_list)))
       _kinematic_limits_list)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/KinematicLimits[] _kinematic_limits_list
    (apply #'+ (send-all _kinematic_limits_list :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/KinematicLimits[] _kinematic_limits_list
     (write-long (length _kinematic_limits_list) s)
     (dolist (elem _kinematic_limits_list)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/KinematicLimits[] _kinematic_limits_list
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _kinematic_limits_list (let (r) (dotimes (i n) (push (instance kortex_driver::KinematicLimits :init) r)) r))
     (dolist (elem- _kinematic_limits_list)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::KinematicLimitsList :md5sum-) "85620e63d1ea4e2fbf21a5cb16cb2964")
(setf (get kortex_driver::KinematicLimitsList :datatype-) "kortex_driver/KinematicLimitsList")
(setf (get kortex_driver::KinematicLimitsList :definition-)
      "
KinematicLimits[] kinematic_limits_list
================================================================================
MSG: kortex_driver/KinematicLimits

uint32 control_mode
float32 twist_linear
float32 twist_angular
float32[] joint_speed_limits
float32[] joint_acceleration_limits
")



(provide :kortex_driver/KinematicLimitsList "85620e63d1ea4e2fbf21a5cb16cb2964")


