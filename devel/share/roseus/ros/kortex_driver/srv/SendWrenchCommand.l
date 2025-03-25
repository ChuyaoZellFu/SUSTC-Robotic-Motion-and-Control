;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SendWrenchCommand)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SendWrenchCommand (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SENDWRENCHCOMMAND")
  (make-package "KORTEX_DRIVER::SENDWRENCHCOMMAND"))
(unless (find-package "KORTEX_DRIVER::SENDWRENCHCOMMANDREQUEST")
  (make-package "KORTEX_DRIVER::SENDWRENCHCOMMANDREQUEST"))
(unless (find-package "KORTEX_DRIVER::SENDWRENCHCOMMANDRESPONSE")
  (make-package "KORTEX_DRIVER::SENDWRENCHCOMMANDRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SendWrenchCommandRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SendWrenchCommandRequest
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

(defclass kortex_driver::SendWrenchCommandResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SendWrenchCommandResponse
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

(defclass kortex_driver::SendWrenchCommand
  :super ros::object
  :slots ())

(setf (get kortex_driver::SendWrenchCommand :md5sum-) "f2e3d158b1d1ef2944d97f7820b907ad")
(setf (get kortex_driver::SendWrenchCommand :datatype-) "kortex_driver/SendWrenchCommand")
(setf (get kortex_driver::SendWrenchCommand :request) kortex_driver::SendWrenchCommandRequest)
(setf (get kortex_driver::SendWrenchCommand :response) kortex_driver::SendWrenchCommandResponse)

(defmethod kortex_driver::SendWrenchCommandRequest
  (:response () (instance kortex_driver::SendWrenchCommandResponse :init)))

(setf (get kortex_driver::SendWrenchCommandRequest :md5sum-) "f2e3d158b1d1ef2944d97f7820b907ad")
(setf (get kortex_driver::SendWrenchCommandRequest :datatype-) "kortex_driver/SendWrenchCommandRequest")
(setf (get kortex_driver::SendWrenchCommandRequest :definition-)
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

(setf (get kortex_driver::SendWrenchCommandResponse :md5sum-) "f2e3d158b1d1ef2944d97f7820b907ad")
(setf (get kortex_driver::SendWrenchCommandResponse :datatype-) "kortex_driver/SendWrenchCommandResponse")
(setf (get kortex_driver::SendWrenchCommandResponse :definition-)
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



(provide :kortex_driver/SendWrenchCommand "f2e3d158b1d1ef2944d97f7820b907ad")


