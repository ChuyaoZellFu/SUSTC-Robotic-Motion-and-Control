;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SendSelectedJointSpeedJoystickCommand)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SendSelectedJointSpeedJoystickCommand (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SENDSELECTEDJOINTSPEEDJOYSTICKCOMMAND")
  (make-package "KORTEX_DRIVER::SENDSELECTEDJOINTSPEEDJOYSTICKCOMMAND"))
(unless (find-package "KORTEX_DRIVER::SENDSELECTEDJOINTSPEEDJOYSTICKCOMMANDREQUEST")
  (make-package "KORTEX_DRIVER::SENDSELECTEDJOINTSPEEDJOYSTICKCOMMANDREQUEST"))
(unless (find-package "KORTEX_DRIVER::SENDSELECTEDJOINTSPEEDJOYSTICKCOMMANDRESPONSE")
  (make-package "KORTEX_DRIVER::SENDSELECTEDJOINTSPEEDJOYSTICKCOMMANDRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SendSelectedJointSpeedJoystickCommandRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SendSelectedJointSpeedJoystickCommandRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::JointSpeed :init))
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
    ;; kortex_driver/JointSpeed _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/JointSpeed _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/JointSpeed _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SendSelectedJointSpeedJoystickCommandResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SendSelectedJointSpeedJoystickCommandResponse
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

(defclass kortex_driver::SendSelectedJointSpeedJoystickCommand
  :super ros::object
  :slots ())

(setf (get kortex_driver::SendSelectedJointSpeedJoystickCommand :md5sum-) "966eae21a761830257b0f20dfcd24c8e")
(setf (get kortex_driver::SendSelectedJointSpeedJoystickCommand :datatype-) "kortex_driver/SendSelectedJointSpeedJoystickCommand")
(setf (get kortex_driver::SendSelectedJointSpeedJoystickCommand :request) kortex_driver::SendSelectedJointSpeedJoystickCommandRequest)
(setf (get kortex_driver::SendSelectedJointSpeedJoystickCommand :response) kortex_driver::SendSelectedJointSpeedJoystickCommandResponse)

(defmethod kortex_driver::SendSelectedJointSpeedJoystickCommandRequest
  (:response () (instance kortex_driver::SendSelectedJointSpeedJoystickCommandResponse :init)))

(setf (get kortex_driver::SendSelectedJointSpeedJoystickCommandRequest :md5sum-) "966eae21a761830257b0f20dfcd24c8e")
(setf (get kortex_driver::SendSelectedJointSpeedJoystickCommandRequest :datatype-) "kortex_driver/SendSelectedJointSpeedJoystickCommandRequest")
(setf (get kortex_driver::SendSelectedJointSpeedJoystickCommandRequest :definition-)
      "JointSpeed input

================================================================================
MSG: kortex_driver/JointSpeed

uint32 joint_identifier
float32 value
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SendSelectedJointSpeedJoystickCommandResponse :md5sum-) "966eae21a761830257b0f20dfcd24c8e")
(setf (get kortex_driver::SendSelectedJointSpeedJoystickCommandResponse :datatype-) "kortex_driver/SendSelectedJointSpeedJoystickCommandResponse")
(setf (get kortex_driver::SendSelectedJointSpeedJoystickCommandResponse :definition-)
      "JointSpeed input

================================================================================
MSG: kortex_driver/JointSpeed

uint32 joint_identifier
float32 value
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SendSelectedJointSpeedJoystickCommand "966eae21a761830257b0f20dfcd24c8e")


