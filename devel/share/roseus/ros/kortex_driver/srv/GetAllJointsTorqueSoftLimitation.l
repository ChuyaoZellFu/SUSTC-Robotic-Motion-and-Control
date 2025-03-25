;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetAllJointsTorqueSoftLimitation)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetAllJointsTorqueSoftLimitation (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETALLJOINTSTORQUESOFTLIMITATION")
  (make-package "KORTEX_DRIVER::GETALLJOINTSTORQUESOFTLIMITATION"))
(unless (find-package "KORTEX_DRIVER::GETALLJOINTSTORQUESOFTLIMITATIONREQUEST")
  (make-package "KORTEX_DRIVER::GETALLJOINTSTORQUESOFTLIMITATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETALLJOINTSTORQUESOFTLIMITATIONRESPONSE")
  (make-package "KORTEX_DRIVER::GETALLJOINTSTORQUESOFTLIMITATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetAllJointsTorqueSoftLimitationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetAllJointsTorqueSoftLimitationRequest
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

(defclass kortex_driver::GetAllJointsTorqueSoftLimitationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetAllJointsTorqueSoftLimitationResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::JointsLimitationsList :init))
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
    ;; kortex_driver/JointsLimitationsList _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/JointsLimitationsList _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/JointsLimitationsList _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetAllJointsTorqueSoftLimitation
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetAllJointsTorqueSoftLimitation :md5sum-) "80ab5247f79a646a096a6bb5bc451a7d")
(setf (get kortex_driver::GetAllJointsTorqueSoftLimitation :datatype-) "kortex_driver/GetAllJointsTorqueSoftLimitation")
(setf (get kortex_driver::GetAllJointsTorqueSoftLimitation :request) kortex_driver::GetAllJointsTorqueSoftLimitationRequest)
(setf (get kortex_driver::GetAllJointsTorqueSoftLimitation :response) kortex_driver::GetAllJointsTorqueSoftLimitationResponse)

(defmethod kortex_driver::GetAllJointsTorqueSoftLimitationRequest
  (:response () (instance kortex_driver::GetAllJointsTorqueSoftLimitationResponse :init)))

(setf (get kortex_driver::GetAllJointsTorqueSoftLimitationRequest :md5sum-) "80ab5247f79a646a096a6bb5bc451a7d")
(setf (get kortex_driver::GetAllJointsTorqueSoftLimitationRequest :datatype-) "kortex_driver/GetAllJointsTorqueSoftLimitationRequest")
(setf (get kortex_driver::GetAllJointsTorqueSoftLimitationRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
JointsLimitationsList output

================================================================================
MSG: kortex_driver/JointsLimitationsList

JointLimitation[] joints_limitations
================================================================================
MSG: kortex_driver/JointLimitation

uint32 joint_identifier
uint32 type
")

(setf (get kortex_driver::GetAllJointsTorqueSoftLimitationResponse :md5sum-) "80ab5247f79a646a096a6bb5bc451a7d")
(setf (get kortex_driver::GetAllJointsTorqueSoftLimitationResponse :datatype-) "kortex_driver/GetAllJointsTorqueSoftLimitationResponse")
(setf (get kortex_driver::GetAllJointsTorqueSoftLimitationResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
JointsLimitationsList output

================================================================================
MSG: kortex_driver/JointsLimitationsList

JointLimitation[] joints_limitations
================================================================================
MSG: kortex_driver/JointLimitation

uint32 joint_identifier
uint32 type
")



(provide :kortex_driver/GetAllJointsTorqueSoftLimitation "80ab5247f79a646a096a6bb5bc451a7d")


