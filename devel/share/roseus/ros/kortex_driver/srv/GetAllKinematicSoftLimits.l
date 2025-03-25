;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetAllKinematicSoftLimits)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetAllKinematicSoftLimits (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETALLKINEMATICSOFTLIMITS")
  (make-package "KORTEX_DRIVER::GETALLKINEMATICSOFTLIMITS"))
(unless (find-package "KORTEX_DRIVER::GETALLKINEMATICSOFTLIMITSREQUEST")
  (make-package "KORTEX_DRIVER::GETALLKINEMATICSOFTLIMITSREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETALLKINEMATICSOFTLIMITSRESPONSE")
  (make-package "KORTEX_DRIVER::GETALLKINEMATICSOFTLIMITSRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetAllKinematicSoftLimitsRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetAllKinematicSoftLimitsRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::Empty :init))
    )
   (send-super :init)
   (setq _input __input)
   self)
  (:input
   (&rest __input)
   (if (keywordp (car __input))
       (send* _input __input)
     (progn
       (if __input (setq _input (car __input)))
       _input)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/Empty _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Empty _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Empty _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetAllKinematicSoftLimitsResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetAllKinematicSoftLimitsResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::KinematicLimitsList :init))
    )
   (send-super :init)
   (setq _output __output)
   self)
  (:output
   (&rest __output)
   (if (keywordp (car __output))
       (send* _output __output)
     (progn
       (if __output (setq _output (car __output)))
       _output)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/KinematicLimitsList _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/KinematicLimitsList _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/KinematicLimitsList _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetAllKinematicSoftLimits
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetAllKinematicSoftLimits :md5sum-) "82a8454ad50f9bddae049fa61b8b3c3d")
(setf (get kortex_driver::GetAllKinematicSoftLimits :datatype-) "kortex_driver/GetAllKinematicSoftLimits")
(setf (get kortex_driver::GetAllKinematicSoftLimits :request) kortex_driver::GetAllKinematicSoftLimitsRequest)
(setf (get kortex_driver::GetAllKinematicSoftLimits :response) kortex_driver::GetAllKinematicSoftLimitsResponse)

(defmethod kortex_driver::GetAllKinematicSoftLimitsRequest
  (:response () (instance kortex_driver::GetAllKinematicSoftLimitsResponse :init)))

(setf (get kortex_driver::GetAllKinematicSoftLimitsRequest :md5sum-) "82a8454ad50f9bddae049fa61b8b3c3d")
(setf (get kortex_driver::GetAllKinematicSoftLimitsRequest :datatype-) "kortex_driver/GetAllKinematicSoftLimitsRequest")
(setf (get kortex_driver::GetAllKinematicSoftLimitsRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
KinematicLimitsList output

================================================================================
MSG: kortex_driver/KinematicLimitsList

KinematicLimits[] kinematic_limits_list
================================================================================
MSG: kortex_driver/KinematicLimits

uint32 control_mode
float32 twist_linear
float32 twist_angular
float32[] joint_speed_limits
")

(setf (get kortex_driver::GetAllKinematicSoftLimitsResponse :md5sum-) "82a8454ad50f9bddae049fa61b8b3c3d")
(setf (get kortex_driver::GetAllKinematicSoftLimitsResponse :datatype-) "kortex_driver/GetAllKinematicSoftLimitsResponse")
(setf (get kortex_driver::GetAllKinematicSoftLimitsResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
KinematicLimitsList output

================================================================================
MSG: kortex_driver/KinematicLimitsList

KinematicLimits[] kinematic_limits_list
================================================================================
MSG: kortex_driver/KinematicLimits

uint32 control_mode
float32 twist_linear
float32 twist_angular
float32[] joint_speed_limits
")



(provide :kortex_driver/GetAllKinematicSoftLimits "82a8454ad50f9bddae049fa61b8b3c3d")


