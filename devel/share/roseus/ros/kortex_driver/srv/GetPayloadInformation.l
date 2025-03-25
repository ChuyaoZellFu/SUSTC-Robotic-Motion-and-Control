;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetPayloadInformation)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetPayloadInformation (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETPAYLOADINFORMATION")
  (make-package "KORTEX_DRIVER::GETPAYLOADINFORMATION"))
(unless (find-package "KORTEX_DRIVER::GETPAYLOADINFORMATIONREQUEST")
  (make-package "KORTEX_DRIVER::GETPAYLOADINFORMATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETPAYLOADINFORMATIONRESPONSE")
  (make-package "KORTEX_DRIVER::GETPAYLOADINFORMATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetPayloadInformationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetPayloadInformationRequest
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

(defclass kortex_driver::GetPayloadInformationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetPayloadInformationResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::PayloadInformation :init))
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
    ;; kortex_driver/PayloadInformation _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/PayloadInformation _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/PayloadInformation _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetPayloadInformation
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetPayloadInformation :md5sum-) "38a744b19ddbb71fb4d7e8724de570f6")
(setf (get kortex_driver::GetPayloadInformation :datatype-) "kortex_driver/GetPayloadInformation")
(setf (get kortex_driver::GetPayloadInformation :request) kortex_driver::GetPayloadInformationRequest)
(setf (get kortex_driver::GetPayloadInformation :response) kortex_driver::GetPayloadInformationResponse)

(defmethod kortex_driver::GetPayloadInformationRequest
  (:response () (instance kortex_driver::GetPayloadInformationResponse :init)))

(setf (get kortex_driver::GetPayloadInformationRequest :md5sum-) "38a744b19ddbb71fb4d7e8724de570f6")
(setf (get kortex_driver::GetPayloadInformationRequest :datatype-) "kortex_driver/GetPayloadInformationRequest")
(setf (get kortex_driver::GetPayloadInformationRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
PayloadInformation output

================================================================================
MSG: kortex_driver/PayloadInformation

float32 payload_mass
ControlConfig_Position payload_mass_center
================================================================================
MSG: kortex_driver/ControlConfig_Position

float32 x
float32 y
")

(setf (get kortex_driver::GetPayloadInformationResponse :md5sum-) "38a744b19ddbb71fb4d7e8724de570f6")
(setf (get kortex_driver::GetPayloadInformationResponse :datatype-) "kortex_driver/GetPayloadInformationResponse")
(setf (get kortex_driver::GetPayloadInformationResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
PayloadInformation output

================================================================================
MSG: kortex_driver/PayloadInformation

float32 payload_mass
ControlConfig_Position payload_mass_center
================================================================================
MSG: kortex_driver/ControlConfig_Position

float32 x
float32 y
")



(provide :kortex_driver/GetPayloadInformation "38a744b19ddbb71fb4d7e8724de570f6")


