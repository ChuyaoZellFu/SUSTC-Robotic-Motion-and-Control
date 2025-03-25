;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetMeasuredJointAngles)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetMeasuredJointAngles (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETMEASUREDJOINTANGLES")
  (make-package "KORTEX_DRIVER::GETMEASUREDJOINTANGLES"))
(unless (find-package "KORTEX_DRIVER::GETMEASUREDJOINTANGLESREQUEST")
  (make-package "KORTEX_DRIVER::GETMEASUREDJOINTANGLESREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETMEASUREDJOINTANGLESRESPONSE")
  (make-package "KORTEX_DRIVER::GETMEASUREDJOINTANGLESRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetMeasuredJointAnglesRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetMeasuredJointAnglesRequest
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

(defclass kortex_driver::GetMeasuredJointAnglesResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetMeasuredJointAnglesResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::JointAngles :init))
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
    ;; kortex_driver/JointAngles _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/JointAngles _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/JointAngles _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetMeasuredJointAngles
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetMeasuredJointAngles :md5sum-) "a8cbc5dedf438645c485057a0e9b03e4")
(setf (get kortex_driver::GetMeasuredJointAngles :datatype-) "kortex_driver/GetMeasuredJointAngles")
(setf (get kortex_driver::GetMeasuredJointAngles :request) kortex_driver::GetMeasuredJointAnglesRequest)
(setf (get kortex_driver::GetMeasuredJointAngles :response) kortex_driver::GetMeasuredJointAnglesResponse)

(defmethod kortex_driver::GetMeasuredJointAnglesRequest
  (:response () (instance kortex_driver::GetMeasuredJointAnglesResponse :init)))

(setf (get kortex_driver::GetMeasuredJointAnglesRequest :md5sum-) "a8cbc5dedf438645c485057a0e9b03e4")
(setf (get kortex_driver::GetMeasuredJointAnglesRequest :datatype-) "kortex_driver/GetMeasuredJointAnglesRequest")
(setf (get kortex_driver::GetMeasuredJointAnglesRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
JointAngles output

================================================================================
MSG: kortex_driver/JointAngles

JointAngle[] joint_angles
================================================================================
MSG: kortex_driver/JointAngle

uint32 joint_identifier
")

(setf (get kortex_driver::GetMeasuredJointAnglesResponse :md5sum-) "a8cbc5dedf438645c485057a0e9b03e4")
(setf (get kortex_driver::GetMeasuredJointAnglesResponse :datatype-) "kortex_driver/GetMeasuredJointAnglesResponse")
(setf (get kortex_driver::GetMeasuredJointAnglesResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
JointAngles output

================================================================================
MSG: kortex_driver/JointAngles

JointAngle[] joint_angles
================================================================================
MSG: kortex_driver/JointAngle

uint32 joint_identifier
")



(provide :kortex_driver/GetMeasuredJointAngles "a8cbc5dedf438645c485057a0e9b03e4")


