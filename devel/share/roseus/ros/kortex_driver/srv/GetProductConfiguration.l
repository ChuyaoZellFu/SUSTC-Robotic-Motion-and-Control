;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetProductConfiguration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetProductConfiguration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETPRODUCTCONFIGURATION")
  (make-package "KORTEX_DRIVER::GETPRODUCTCONFIGURATION"))
(unless (find-package "KORTEX_DRIVER::GETPRODUCTCONFIGURATIONREQUEST")
  (make-package "KORTEX_DRIVER::GETPRODUCTCONFIGURATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETPRODUCTCONFIGURATIONRESPONSE")
  (make-package "KORTEX_DRIVER::GETPRODUCTCONFIGURATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetProductConfigurationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetProductConfigurationRequest
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

(defclass kortex_driver::GetProductConfigurationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetProductConfigurationResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::CompleteProductConfiguration :init))
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
    ;; kortex_driver/CompleteProductConfiguration _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/CompleteProductConfiguration _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/CompleteProductConfiguration _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetProductConfiguration
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetProductConfiguration :md5sum-) "899fbdc53f9306591e652c7c52962785")
(setf (get kortex_driver::GetProductConfiguration :datatype-) "kortex_driver/GetProductConfiguration")
(setf (get kortex_driver::GetProductConfiguration :request) kortex_driver::GetProductConfigurationRequest)
(setf (get kortex_driver::GetProductConfiguration :response) kortex_driver::GetProductConfigurationResponse)

(defmethod kortex_driver::GetProductConfigurationRequest
  (:response () (instance kortex_driver::GetProductConfigurationResponse :init)))

(setf (get kortex_driver::GetProductConfigurationRequest :md5sum-) "899fbdc53f9306591e652c7c52962785")
(setf (get kortex_driver::GetProductConfigurationRequest :datatype-) "kortex_driver/GetProductConfigurationRequest")
(setf (get kortex_driver::GetProductConfigurationRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
CompleteProductConfiguration output

================================================================================
MSG: kortex_driver/CompleteProductConfiguration

string kin
uint32 model
CountryCode country_code
string assembly_plant
string model_year
uint32 degree_of_freedom
uint32 base_type
uint32 end_effector_type
uint32 vision_module_type
uint32 interface_module_type
uint32 arm_laterality
uint32 wrist_type
uint32 brake_type
================================================================================
MSG: kortex_driver/CountryCode

")

(setf (get kortex_driver::GetProductConfigurationResponse :md5sum-) "899fbdc53f9306591e652c7c52962785")
(setf (get kortex_driver::GetProductConfigurationResponse :datatype-) "kortex_driver/GetProductConfigurationResponse")
(setf (get kortex_driver::GetProductConfigurationResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
CompleteProductConfiguration output

================================================================================
MSG: kortex_driver/CompleteProductConfiguration

string kin
uint32 model
CountryCode country_code
string assembly_plant
string model_year
uint32 degree_of_freedom
uint32 base_type
uint32 end_effector_type
uint32 vision_module_type
uint32 interface_module_type
uint32 arm_laterality
uint32 wrist_type
uint32 brake_type
================================================================================
MSG: kortex_driver/CountryCode

")



(provide :kortex_driver/GetProductConfiguration "899fbdc53f9306591e652c7c52962785")


