;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetKinematicHardLimits)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetKinematicHardLimits (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETKINEMATICHARDLIMITS")
  (make-package "KORTEX_DRIVER::GETKINEMATICHARDLIMITS"))
(unless (find-package "KORTEX_DRIVER::GETKINEMATICHARDLIMITSREQUEST")
  (make-package "KORTEX_DRIVER::GETKINEMATICHARDLIMITSREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETKINEMATICHARDLIMITSRESPONSE")
  (make-package "KORTEX_DRIVER::GETKINEMATICHARDLIMITSRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetKinematicHardLimitsRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetKinematicHardLimitsRequest
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

(defclass kortex_driver::GetKinematicHardLimitsResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetKinematicHardLimitsResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::KinematicLimits :init))
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
    ;; kortex_driver/KinematicLimits _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/KinematicLimits _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/KinematicLimits _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetKinematicHardLimits
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetKinematicHardLimits :md5sum-) "024037c761d64c76474f3dff50b971a3")
(setf (get kortex_driver::GetKinematicHardLimits :datatype-) "kortex_driver/GetKinematicHardLimits")
(setf (get kortex_driver::GetKinematicHardLimits :request) kortex_driver::GetKinematicHardLimitsRequest)
(setf (get kortex_driver::GetKinematicHardLimits :response) kortex_driver::GetKinematicHardLimitsResponse)

(defmethod kortex_driver::GetKinematicHardLimitsRequest
  (:response () (instance kortex_driver::GetKinematicHardLimitsResponse :init)))

(setf (get kortex_driver::GetKinematicHardLimitsRequest :md5sum-) "024037c761d64c76474f3dff50b971a3")
(setf (get kortex_driver::GetKinematicHardLimitsRequest :datatype-) "kortex_driver/GetKinematicHardLimitsRequest")
(setf (get kortex_driver::GetKinematicHardLimitsRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
KinematicLimits output

================================================================================
MSG: kortex_driver/KinematicLimits

uint32 control_mode
float32 twist_linear
float32 twist_angular
float32[] joint_speed_limits
")

(setf (get kortex_driver::GetKinematicHardLimitsResponse :md5sum-) "024037c761d64c76474f3dff50b971a3")
(setf (get kortex_driver::GetKinematicHardLimitsResponse :datatype-) "kortex_driver/GetKinematicHardLimitsResponse")
(setf (get kortex_driver::GetKinematicHardLimitsResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
KinematicLimits output

================================================================================
MSG: kortex_driver/KinematicLimits

uint32 control_mode
float32 twist_linear
float32 twist_angular
float32[] joint_speed_limits
")



(provide :kortex_driver/GetKinematicHardLimits "024037c761d64c76474f3dff50b971a3")


