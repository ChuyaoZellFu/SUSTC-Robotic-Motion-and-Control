;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SendGripperCommand)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SendGripperCommand (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SENDGRIPPERCOMMAND")
  (make-package "KORTEX_DRIVER::SENDGRIPPERCOMMAND"))
(unless (find-package "KORTEX_DRIVER::SENDGRIPPERCOMMANDREQUEST")
  (make-package "KORTEX_DRIVER::SENDGRIPPERCOMMANDREQUEST"))
(unless (find-package "KORTEX_DRIVER::SENDGRIPPERCOMMANDRESPONSE")
  (make-package "KORTEX_DRIVER::SENDGRIPPERCOMMANDRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SendGripperCommandRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SendGripperCommandRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::GripperCommand :init))
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
    ;; kortex_driver/GripperCommand _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/GripperCommand _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/GripperCommand _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SendGripperCommandResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SendGripperCommandResponse
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

(defclass kortex_driver::SendGripperCommand
  :super ros::object
  :slots ())

(setf (get kortex_driver::SendGripperCommand :md5sum-) "d750c71a9686ff834d5687f20aaad330")
(setf (get kortex_driver::SendGripperCommand :datatype-) "kortex_driver/SendGripperCommand")
(setf (get kortex_driver::SendGripperCommand :request) kortex_driver::SendGripperCommandRequest)
(setf (get kortex_driver::SendGripperCommand :response) kortex_driver::SendGripperCommandResponse)

(defmethod kortex_driver::SendGripperCommandRequest
  (:response () (instance kortex_driver::SendGripperCommandResponse :init)))

(setf (get kortex_driver::SendGripperCommandRequest :md5sum-) "d750c71a9686ff834d5687f20aaad330")
(setf (get kortex_driver::SendGripperCommandRequest :datatype-) "kortex_driver/SendGripperCommandRequest")
(setf (get kortex_driver::SendGripperCommandRequest :definition-)
      "GripperCommand input

================================================================================
MSG: kortex_driver/GripperCommand

uint32 mode
Gripper gripper
uint32 duration
================================================================================
MSG: kortex_driver/Gripper

Finger[] finger
================================================================================
MSG: kortex_driver/Finger

uint32 finger_identifier
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SendGripperCommandResponse :md5sum-) "d750c71a9686ff834d5687f20aaad330")
(setf (get kortex_driver::SendGripperCommandResponse :datatype-) "kortex_driver/SendGripperCommandResponse")
(setf (get kortex_driver::SendGripperCommandResponse :definition-)
      "GripperCommand input

================================================================================
MSG: kortex_driver/GripperCommand

uint32 mode
Gripper gripper
uint32 duration
================================================================================
MSG: kortex_driver/Gripper

Finger[] finger
================================================================================
MSG: kortex_driver/Finger

uint32 finger_identifier
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SendGripperCommand "d750c71a9686ff834d5687f20aaad330")


