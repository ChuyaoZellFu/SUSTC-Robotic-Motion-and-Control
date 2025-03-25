;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetSafetyInformation)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetSafetyInformation (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETSAFETYINFORMATION")
  (make-package "KORTEX_DRIVER::GETSAFETYINFORMATION"))
(unless (find-package "KORTEX_DRIVER::GETSAFETYINFORMATIONREQUEST")
  (make-package "KORTEX_DRIVER::GETSAFETYINFORMATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETSAFETYINFORMATIONRESPONSE")
  (make-package "KORTEX_DRIVER::GETSAFETYINFORMATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetSafetyInformationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetSafetyInformationRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::SafetyHandle :init))
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
    ;; kortex_driver/SafetyHandle _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/SafetyHandle _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/SafetyHandle _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetSafetyInformationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetSafetyInformationResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::SafetyInformation :init))
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
    ;; kortex_driver/SafetyInformation _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/SafetyInformation _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/SafetyInformation _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetSafetyInformation
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetSafetyInformation :md5sum-) "03aeca1f2d2314fba73be4aeb3606ece")
(setf (get kortex_driver::GetSafetyInformation :datatype-) "kortex_driver/GetSafetyInformation")
(setf (get kortex_driver::GetSafetyInformation :request) kortex_driver::GetSafetyInformationRequest)
(setf (get kortex_driver::GetSafetyInformation :response) kortex_driver::GetSafetyInformationResponse)

(defmethod kortex_driver::GetSafetyInformationRequest
  (:response () (instance kortex_driver::GetSafetyInformationResponse :init)))

(setf (get kortex_driver::GetSafetyInformationRequest :md5sum-) "03aeca1f2d2314fba73be4aeb3606ece")
(setf (get kortex_driver::GetSafetyInformationRequest :datatype-) "kortex_driver/GetSafetyInformationRequest")
(setf (get kortex_driver::GetSafetyInformationRequest :definition-)
      "SafetyHandle input

================================================================================
MSG: kortex_driver/SafetyHandle

---
SafetyInformation output

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

(setf (get kortex_driver::GetSafetyInformationResponse :md5sum-) "03aeca1f2d2314fba73be4aeb3606ece")
(setf (get kortex_driver::GetSafetyInformationResponse :datatype-) "kortex_driver/GetSafetyInformationResponse")
(setf (get kortex_driver::GetSafetyInformationResponse :definition-)
      "SafetyHandle input

================================================================================
MSG: kortex_driver/SafetyHandle

---
SafetyInformation output

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



(provide :kortex_driver/GetSafetyInformation "03aeca1f2d2314fba73be4aeb3606ece")


