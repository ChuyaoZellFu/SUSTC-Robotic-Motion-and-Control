;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ResetPayloadInformation)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ResetPayloadInformation (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::RESETPAYLOADINFORMATION")
  (make-package "KORTEX_DRIVER::RESETPAYLOADINFORMATION"))
(unless (find-package "KORTEX_DRIVER::RESETPAYLOADINFORMATIONREQUEST")
  (make-package "KORTEX_DRIVER::RESETPAYLOADINFORMATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::RESETPAYLOADINFORMATIONRESPONSE")
  (make-package "KORTEX_DRIVER::RESETPAYLOADINFORMATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::ResetPayloadInformationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::ResetPayloadInformationRequest
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

(defclass kortex_driver::ResetPayloadInformationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::ResetPayloadInformationResponse
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

(defclass kortex_driver::ResetPayloadInformation
  :super ros::object
  :slots ())

(setf (get kortex_driver::ResetPayloadInformation :md5sum-) "38a744b19ddbb71fb4d7e8724de570f6")
(setf (get kortex_driver::ResetPayloadInformation :datatype-) "kortex_driver/ResetPayloadInformation")
(setf (get kortex_driver::ResetPayloadInformation :request) kortex_driver::ResetPayloadInformationRequest)
(setf (get kortex_driver::ResetPayloadInformation :response) kortex_driver::ResetPayloadInformationResponse)

(defmethod kortex_driver::ResetPayloadInformationRequest
  (:response () (instance kortex_driver::ResetPayloadInformationResponse :init)))

(setf (get kortex_driver::ResetPayloadInformationRequest :md5sum-) "38a744b19ddbb71fb4d7e8724de570f6")
(setf (get kortex_driver::ResetPayloadInformationRequest :datatype-) "kortex_driver/ResetPayloadInformationRequest")
(setf (get kortex_driver::ResetPayloadInformationRequest :definition-)
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

(setf (get kortex_driver::ResetPayloadInformationResponse :md5sum-) "38a744b19ddbb71fb4d7e8724de570f6")
(setf (get kortex_driver::ResetPayloadInformationResponse :datatype-) "kortex_driver/ResetPayloadInformationResponse")
(setf (get kortex_driver::ResetPayloadInformationResponse :definition-)
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



(provide :kortex_driver/ResetPayloadInformation "38a744b19ddbb71fb4d7e8724de570f6")


