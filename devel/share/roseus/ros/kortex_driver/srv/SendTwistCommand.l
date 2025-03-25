;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SendTwistCommand)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SendTwistCommand (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SENDTWISTCOMMAND")
  (make-package "KORTEX_DRIVER::SENDTWISTCOMMAND"))
(unless (find-package "KORTEX_DRIVER::SENDTWISTCOMMANDREQUEST")
  (make-package "KORTEX_DRIVER::SENDTWISTCOMMANDREQUEST"))
(unless (find-package "KORTEX_DRIVER::SENDTWISTCOMMANDRESPONSE")
  (make-package "KORTEX_DRIVER::SENDTWISTCOMMANDRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SendTwistCommandRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SendTwistCommandRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::TwistCommand :init))
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
    ;; kortex_driver/TwistCommand _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/TwistCommand _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/TwistCommand _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SendTwistCommandResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SendTwistCommandResponse
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

(defclass kortex_driver::SendTwistCommand
  :super ros::object
  :slots ())

(setf (get kortex_driver::SendTwistCommand :md5sum-) "1d982d6ab027b7ae362cacd16bfb339f")
(setf (get kortex_driver::SendTwistCommand :datatype-) "kortex_driver/SendTwistCommand")
(setf (get kortex_driver::SendTwistCommand :request) kortex_driver::SendTwistCommandRequest)
(setf (get kortex_driver::SendTwistCommand :response) kortex_driver::SendTwistCommandResponse)

(defmethod kortex_driver::SendTwistCommandRequest
  (:response () (instance kortex_driver::SendTwistCommandResponse :init)))

(setf (get kortex_driver::SendTwistCommandRequest :md5sum-) "1d982d6ab027b7ae362cacd16bfb339f")
(setf (get kortex_driver::SendTwistCommandRequest :datatype-) "kortex_driver/SendTwistCommandRequest")
(setf (get kortex_driver::SendTwistCommandRequest :definition-)
      "TwistCommand input

================================================================================
MSG: kortex_driver/TwistCommand

uint32 reference_frame
Twist twist
uint32 duration
================================================================================
MSG: kortex_driver/Twist

float32 linear_x
float32 linear_y
float32 linear_z
float32 angular_x
float32 angular_y
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SendTwistCommandResponse :md5sum-) "1d982d6ab027b7ae362cacd16bfb339f")
(setf (get kortex_driver::SendTwistCommandResponse :datatype-) "kortex_driver/SendTwistCommandResponse")
(setf (get kortex_driver::SendTwistCommandResponse :definition-)
      "TwistCommand input

================================================================================
MSG: kortex_driver/TwistCommand

uint32 reference_frame
Twist twist
uint32 duration
================================================================================
MSG: kortex_driver/Twist

float32 linear_x
float32 linear_y
float32 linear_z
float32 angular_x
float32 angular_y
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SendTwistCommand "1d982d6ab027b7ae362cacd16bfb339f")


