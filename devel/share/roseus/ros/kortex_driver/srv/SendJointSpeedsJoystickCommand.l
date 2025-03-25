;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SendJointSpeedsJoystickCommand)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SendJointSpeedsJoystickCommand (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SENDJOINTSPEEDSJOYSTICKCOMMAND")
  (make-package "KORTEX_DRIVER::SENDJOINTSPEEDSJOYSTICKCOMMAND"))
(unless (find-package "KORTEX_DRIVER::SENDJOINTSPEEDSJOYSTICKCOMMANDREQUEST")
  (make-package "KORTEX_DRIVER::SENDJOINTSPEEDSJOYSTICKCOMMANDREQUEST"))
(unless (find-package "KORTEX_DRIVER::SENDJOINTSPEEDSJOYSTICKCOMMANDRESPONSE")
  (make-package "KORTEX_DRIVER::SENDJOINTSPEEDSJOYSTICKCOMMANDRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SendJointSpeedsJoystickCommandRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SendJointSpeedsJoystickCommandRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::Base_JointSpeeds :init))
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
    ;; kortex_driver/Base_JointSpeeds _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Base_JointSpeeds _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Base_JointSpeeds _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SendJointSpeedsJoystickCommandResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SendJointSpeedsJoystickCommandResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::Empty :init))
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
    ;; kortex_driver/Empty _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Empty _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Empty _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SendJointSpeedsJoystickCommand
  :super ros::object
  :slots ())

(setf (get kortex_driver::SendJointSpeedsJoystickCommand :md5sum-) "35bff15135e19b4099e6a92d5e7d08d5")
(setf (get kortex_driver::SendJointSpeedsJoystickCommand :datatype-) "kortex_driver/SendJointSpeedsJoystickCommand")
(setf (get kortex_driver::SendJointSpeedsJoystickCommand :request) kortex_driver::SendJointSpeedsJoystickCommandRequest)
(setf (get kortex_driver::SendJointSpeedsJoystickCommand :response) kortex_driver::SendJointSpeedsJoystickCommandResponse)

(defmethod kortex_driver::SendJointSpeedsJoystickCommandRequest
  (:response () (instance kortex_driver::SendJointSpeedsJoystickCommandResponse :init)))

(setf (get kortex_driver::SendJointSpeedsJoystickCommandRequest :md5sum-) "35bff15135e19b4099e6a92d5e7d08d5")
(setf (get kortex_driver::SendJointSpeedsJoystickCommandRequest :datatype-) "kortex_driver/SendJointSpeedsJoystickCommandRequest")
(setf (get kortex_driver::SendJointSpeedsJoystickCommandRequest :definition-)
      "Base_JointSpeeds input

================================================================================
MSG: kortex_driver/Base_JointSpeeds

JointSpeed[] joint_speeds
uint32 duration
================================================================================
MSG: kortex_driver/JointSpeed

uint32 joint_identifier
float32 value
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SendJointSpeedsJoystickCommandResponse :md5sum-) "35bff15135e19b4099e6a92d5e7d08d5")
(setf (get kortex_driver::SendJointSpeedsJoystickCommandResponse :datatype-) "kortex_driver/SendJointSpeedsJoystickCommandResponse")
(setf (get kortex_driver::SendJointSpeedsJoystickCommandResponse :definition-)
      "Base_JointSpeeds input

================================================================================
MSG: kortex_driver/Base_JointSpeeds

JointSpeed[] joint_speeds
uint32 duration
================================================================================
MSG: kortex_driver/JointSpeed

uint32 joint_identifier
float32 value
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SendJointSpeedsJoystickCommand "35bff15135e19b4099e6a92d5e7d08d5")


