;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetAllSafetyInformation)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetAllSafetyInformation (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETALLSAFETYINFORMATION")
  (make-package "KORTEX_DRIVER::GETALLSAFETYINFORMATION"))
(unless (find-package "KORTEX_DRIVER::GETALLSAFETYINFORMATIONREQUEST")
  (make-package "KORTEX_DRIVER::GETALLSAFETYINFORMATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETALLSAFETYINFORMATIONRESPONSE")
  (make-package "KORTEX_DRIVER::GETALLSAFETYINFORMATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetAllSafetyInformationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetAllSafetyInformationRequest
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

(defclass kortex_driver::GetAllSafetyInformationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetAllSafetyInformationResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::SafetyInformationList :init))
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
    ;; kortex_driver/SafetyInformationList _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/SafetyInformationList _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/SafetyInformationList _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetAllSafetyInformation
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetAllSafetyInformation :md5sum-) "6cd621443d851423fb32151d65f5f576")
(setf (get kortex_driver::GetAllSafetyInformation :datatype-) "kortex_driver/GetAllSafetyInformation")
(setf (get kortex_driver::GetAllSafetyInformation :request) kortex_driver::GetAllSafetyInformationRequest)
(setf (get kortex_driver::GetAllSafetyInformation :response) kortex_driver::GetAllSafetyInformationResponse)

(defmethod kortex_driver::GetAllSafetyInformationRequest
  (:response () (instance kortex_driver::GetAllSafetyInformationResponse :init)))

(setf (get kortex_driver::GetAllSafetyInformationRequest :md5sum-) "6cd621443d851423fb32151d65f5f576")
(setf (get kortex_driver::GetAllSafetyInformationRequest :datatype-) "kortex_driver/GetAllSafetyInformationRequest")
(setf (get kortex_driver::GetAllSafetyInformationRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
SafetyInformationList output

================================================================================
MSG: kortex_driver/SafetyInformationList

SafetyInformation[] information
================================================================================
MSG: kortex_driver/SafetyInformation

SafetyHandle handle
bool can_change_safety_state
bool has_warning_threshold
bool has_error_threshold
uint32 limit_type
float32 default_warning_threshold
float32 default_error_threshold
float32 upper_hard_limit
float32 lower_hard_limit
uint32 status
uint32 unit
================================================================================
MSG: kortex_driver/SafetyHandle

")

(setf (get kortex_driver::GetAllSafetyInformationResponse :md5sum-) "6cd621443d851423fb32151d65f5f576")
(setf (get kortex_driver::GetAllSafetyInformationResponse :datatype-) "kortex_driver/GetAllSafetyInformationResponse")
(setf (get kortex_driver::GetAllSafetyInformationResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
SafetyInformationList output

================================================================================
MSG: kortex_driver/SafetyInformationList

SafetyInformation[] information
================================================================================
MSG: kortex_driver/SafetyInformation

SafetyHandle handle
bool can_change_safety_state
bool has_warning_threshold
bool has_error_threshold
uint32 limit_type
float32 default_warning_threshold
float32 default_error_threshold
float32 upper_hard_limit
float32 lower_hard_limit
uint32 status
uint32 unit
================================================================================
MSG: kortex_driver/SafetyHandle

")



(provide :kortex_driver/GetAllSafetyInformation "6cd621443d851423fb32151d65f5f576")


