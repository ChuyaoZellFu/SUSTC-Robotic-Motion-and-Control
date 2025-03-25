;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetPayloadInformation)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetPayloadInformation (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETPAYLOADINFORMATION")
  (make-package "KORTEX_DRIVER::SETPAYLOADINFORMATION"))
(unless (find-package "KORTEX_DRIVER::SETPAYLOADINFORMATIONREQUEST")
  (make-package "KORTEX_DRIVER::SETPAYLOADINFORMATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETPAYLOADINFORMATIONRESPONSE")
  (make-package "KORTEX_DRIVER::SETPAYLOADINFORMATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetPayloadInformationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetPayloadInformationRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::PayloadInformation :init))
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
    ;; kortex_driver/PayloadInformation _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/PayloadInformation _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/PayloadInformation _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetPayloadInformationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetPayloadInformationResponse
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

(defclass kortex_driver::SetPayloadInformation
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetPayloadInformation :md5sum-) "132bdc38d85ff3aec89e76f693ad5e54")
(setf (get kortex_driver::SetPayloadInformation :datatype-) "kortex_driver/SetPayloadInformation")
(setf (get kortex_driver::SetPayloadInformation :request) kortex_driver::SetPayloadInformationRequest)
(setf (get kortex_driver::SetPayloadInformation :response) kortex_driver::SetPayloadInformationResponse)

(defmethod kortex_driver::SetPayloadInformationRequest
  (:response () (instance kortex_driver::SetPayloadInformationResponse :init)))

(setf (get kortex_driver::SetPayloadInformationRequest :md5sum-) "132bdc38d85ff3aec89e76f693ad5e54")
(setf (get kortex_driver::SetPayloadInformationRequest :datatype-) "kortex_driver/SetPayloadInformationRequest")
(setf (get kortex_driver::SetPayloadInformationRequest :definition-)
      "PayloadInformation input

================================================================================
MSG: kortex_driver/PayloadInformation

float32 payload_mass
ControlConfig_Position payload_mass_center
================================================================================
MSG: kortex_driver/ControlConfig_Position

float32 x
float32 y
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetPayloadInformationResponse :md5sum-) "132bdc38d85ff3aec89e76f693ad5e54")
(setf (get kortex_driver::SetPayloadInformationResponse :datatype-) "kortex_driver/SetPayloadInformationResponse")
(setf (get kortex_driver::SetPayloadInformationResponse :definition-)
      "PayloadInformation input

================================================================================
MSG: kortex_driver/PayloadInformation

float32 payload_mass
ControlConfig_Position payload_mass_center
================================================================================
MSG: kortex_driver/ControlConfig_Position

float32 x
float32 y
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetPayloadInformation "132bdc38d85ff3aec89e76f693ad5e54")


