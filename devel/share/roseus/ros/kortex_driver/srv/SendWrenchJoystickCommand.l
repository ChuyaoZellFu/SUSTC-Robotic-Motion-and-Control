;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SendWrenchJoystickCommand)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SendWrenchJoystickCommand (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SENDWRENCHJOYSTICKCOMMAND")
  (make-package "KORTEX_DRIVER::SENDWRENCHJOYSTICKCOMMAND"))
(unless (find-package "KORTEX_DRIVER::SENDWRENCHJOYSTICKCOMMANDREQUEST")
  (make-package "KORTEX_DRIVER::SENDWRENCHJOYSTICKCOMMANDREQUEST"))
(unless (find-package "KORTEX_DRIVER::SENDWRENCHJOYSTICKCOMMANDRESPONSE")
  (make-package "KORTEX_DRIVER::SENDWRENCHJOYSTICKCOMMANDRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SendWrenchJoystickCommandRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SendWrenchJoystickCommandRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::WrenchCommand :init))
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
    ;; kortex_driver/WrenchCommand _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/WrenchCommand _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/WrenchCommand _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SendWrenchJoystickCommandResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SendWrenchJoystickCommandResponse
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

(defclass kortex_driver::SendWrenchJoystickCommand
  :super ros::object
  :slots ())

(setf (get kortex_driver::SendWrenchJoystickCommand :md5sum-) "f2e3d158b1d1ef2944d97f7820b907ad")
(setf (get kortex_driver::SendWrenchJoystickCommand :datatype-) "kortex_driver/SendWrenchJoystickCommand")
(setf (get kortex_driver::SendWrenchJoystickCommand :request) kortex_driver::SendWrenchJoystickCommandRequest)
(setf (get kortex_driver::SendWrenchJoystickCommand :response) kortex_driver::SendWrenchJoystickCommandResponse)

(defmethod kortex_driver::SendWrenchJoystickCommandRequest
  (:response () (instance kortex_driver::SendWrenchJoystickCommandResponse :init)))

(setf (get kortex_driver::SendWrenchJoystickCommandRequest :md5sum-) "f2e3d158b1d1ef2944d97f7820b907ad")
(setf (get kortex_driver::SendWrenchJoystickCommandRequest :datatype-) "kortex_driver/SendWrenchJoystickCommandRequest")
(setf (get kortex_driver::SendWrenchJoystickCommandRequest :definition-)
      "WrenchCommand input

================================================================================
MSG: kortex_driver/WrenchCommand

uint32 reference_frame
uint32 mode
Wrench wrench
uint32 duration
================================================================================
MSG: kortex_driver/Wrench

float32 force_x
float32 force_y
float32 force_z
float32 torque_x
float32 torque_y
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SendWrenchJoystickCommandResponse :md5sum-) "f2e3d158b1d1ef2944d97f7820b907ad")
(setf (get kortex_driver::SendWrenchJoystickCommandResponse :datatype-) "kortex_driver/SendWrenchJoystickCommandResponse")
(setf (get kortex_driver::SendWrenchJoystickCommandResponse :definition-)
      "WrenchCommand input

================================================================================
MSG: kortex_driver/WrenchCommand

uint32 reference_frame
uint32 mode
Wrench wrench
uint32 duration
================================================================================
MSG: kortex_driver/Wrench

float32 force_x
float32 force_y
float32 force_z
float32 torque_x
float32 torque_y
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SendWrenchJoystickCommand "f2e3d158b1d1ef2944d97f7820b907ad")


