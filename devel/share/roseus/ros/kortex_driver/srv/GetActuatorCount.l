;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetActuatorCount)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetActuatorCount (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETACTUATORCOUNT")
  (make-package "KORTEX_DRIVER::GETACTUATORCOUNT"))
(unless (find-package "KORTEX_DRIVER::GETACTUATORCOUNTREQUEST")
  (make-package "KORTEX_DRIVER::GETACTUATORCOUNTREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETACTUATORCOUNTRESPONSE")
  (make-package "KORTEX_DRIVER::GETACTUATORCOUNTRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetActuatorCountRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetActuatorCountRequest
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

(defclass kortex_driver::GetActuatorCountResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetActuatorCountResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::ActuatorInformation :init))
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
    ;; kortex_driver/ActuatorInformation _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ActuatorInformation _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ActuatorInformation _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetActuatorCount
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetActuatorCount :md5sum-) "e8b096678f65c6ea2fbcbfdbbf77d42f")
(setf (get kortex_driver::GetActuatorCount :datatype-) "kortex_driver/GetActuatorCount")
(setf (get kortex_driver::GetActuatorCount :request) kortex_driver::GetActuatorCountRequest)
(setf (get kortex_driver::GetActuatorCount :response) kortex_driver::GetActuatorCountResponse)

(defmethod kortex_driver::GetActuatorCountRequest
  (:response () (instance kortex_driver::GetActuatorCountResponse :init)))

(setf (get kortex_driver::GetActuatorCountRequest :md5sum-) "e8b096678f65c6ea2fbcbfdbbf77d42f")
(setf (get kortex_driver::GetActuatorCountRequest :datatype-) "kortex_driver/GetActuatorCountRequest")
(setf (get kortex_driver::GetActuatorCountRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
ActuatorInformation output

================================================================================
MSG: kortex_driver/ActuatorInformation

")

(setf (get kortex_driver::GetActuatorCountResponse :md5sum-) "e8b096678f65c6ea2fbcbfdbbf77d42f")
(setf (get kortex_driver::GetActuatorCountResponse :datatype-) "kortex_driver/GetActuatorCountResponse")
(setf (get kortex_driver::GetActuatorCountResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
ActuatorInformation output

================================================================================
MSG: kortex_driver/ActuatorInformation

")



(provide :kortex_driver/GetActuatorCount "e8b096678f65c6ea2fbcbfdbbf77d42f")


