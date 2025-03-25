;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetOptionValue)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetOptionValue (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETOPTIONVALUE")
  (make-package "KORTEX_DRIVER::SETOPTIONVALUE"))
(unless (find-package "KORTEX_DRIVER::SETOPTIONVALUEREQUEST")
  (make-package "KORTEX_DRIVER::SETOPTIONVALUEREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETOPTIONVALUERESPONSE")
  (make-package "KORTEX_DRIVER::SETOPTIONVALUERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetOptionValueRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetOptionValueRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::OptionValue :init))
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
    ;; kortex_driver/OptionValue _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/OptionValue _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/OptionValue _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetOptionValueResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetOptionValueResponse
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

(defclass kortex_driver::SetOptionValue
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetOptionValue :md5sum-) "c37d6886e6fd73ac7a80b3ff3d2909fa")
(setf (get kortex_driver::SetOptionValue :datatype-) "kortex_driver/SetOptionValue")
(setf (get kortex_driver::SetOptionValue :request) kortex_driver::SetOptionValueRequest)
(setf (get kortex_driver::SetOptionValue :response) kortex_driver::SetOptionValueResponse)

(defmethod kortex_driver::SetOptionValueRequest
  (:response () (instance kortex_driver::SetOptionValueResponse :init)))

(setf (get kortex_driver::SetOptionValueRequest :md5sum-) "c37d6886e6fd73ac7a80b3ff3d2909fa")
(setf (get kortex_driver::SetOptionValueRequest :datatype-) "kortex_driver/SetOptionValueRequest")
(setf (get kortex_driver::SetOptionValueRequest :definition-)
      "OptionValue input

================================================================================
MSG: kortex_driver/OptionValue

uint32 sensor
uint32 option
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetOptionValueResponse :md5sum-) "c37d6886e6fd73ac7a80b3ff3d2909fa")
(setf (get kortex_driver::SetOptionValueResponse :datatype-) "kortex_driver/SetOptionValueResponse")
(setf (get kortex_driver::SetOptionValueResponse :definition-)
      "OptionValue input

================================================================================
MSG: kortex_driver/OptionValue

uint32 sensor
uint32 option
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetOptionValue "c37d6886e6fd73ac7a80b3ff3d2909fa")


