;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::CompleteProductConfiguration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'CompleteProductConfiguration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::COMPLETEPRODUCTCONFIGURATION")
  (make-package "KORTEX_DRIVER::COMPLETEPRODUCTCONFIGURATION"))

(in-package "ROS")
;;//! \htmlinclude CompleteProductConfiguration.msg.html


(defclass kortex_driver::CompleteProductConfiguration
  :super ros::object
  :slots (_kin _model _country_code _assembly_plant _model_year _degree_of_freedom _base_type _end_effector_type _vision_module_type _interface_module_type _arm_laterality _wrist_type _brake_type ))

(defmethod kortex_driver::CompleteProductConfiguration
  (:init
   (&key
    ((:kin __kin) "")
    ((:model __model) 0)
    ((:country_code __country_code) (instance kortex_driver::CountryCode :init))
    ((:assembly_plant __assembly_plant) "")
    ((:model_year __model_year) "")
    ((:degree_of_freedom __degree_of_freedom) 0)
    ((:base_type __base_type) 0)
    ((:end_effector_type __end_effector_type) 0)
    ((:vision_module_type __vision_module_type) 0)
    ((:interface_module_type __interface_module_type) 0)
    ((:arm_laterality __arm_laterality) 0)
    ((:wrist_type __wrist_type) 0)
    ((:brake_type __brake_type) 0)
    )
   (send-super :init)
   (setq _kin (string __kin))
   (setq _model (round __model))
   (setq _country_code __country_code)
   (setq _assembly_plant (string __assembly_plant))
   (setq _model_year (string __model_year))
   (setq _degree_of_freedom (round __degree_of_freedom))
   (setq _base_type (round __base_type))
   (setq _end_effector_type (round __end_effector_type))
   (setq _vision_module_type (round __vision_module_type))
   (setq _interface_module_type (round __interface_module_type))
   (setq _arm_laterality (round __arm_laterality))
   (setq _wrist_type (round __wrist_type))
   (setq _brake_type (round __brake_type))
   self)
  (:kin
   (&optional __kin)
   (if __kin (setq _kin __kin)) _kin)
  (:model
   (&optional __model)
   (if __model (setq _model __model)) _model)
  (:country_code
   (&rest __country_code)
   (if (keywordp (car __country_code))
       (send* _country_code __country_code)
     (progn
       (if __country_code (setq _country_code (car __country_code)))
       _country_code)))
  (:assembly_plant
   (&optional __assembly_plant)
   (if __assembly_plant (setq _assembly_plant __assembly_plant)) _assembly_plant)
  (:model_year
   (&optional __model_year)
   (if __model_year (setq _model_year __model_year)) _model_year)
  (:degree_of_freedom
   (&optional __degree_of_freedom)
   (if __degree_of_freedom (setq _degree_of_freedom __degree_of_freedom)) _degree_of_freedom)
  (:base_type
   (&optional __base_type)
   (if __base_type (setq _base_type __base_type)) _base_type)
  (:end_effector_type
   (&optional __end_effector_type)
   (if __end_effector_type (setq _end_effector_type __end_effector_type)) _end_effector_type)
  (:vision_module_type
   (&optional __vision_module_type)
   (if __vision_module_type (setq _vision_module_type __vision_module_type)) _vision_module_type)
  (:interface_module_type
   (&optional __interface_module_type)
   (if __interface_module_type (setq _interface_module_type __interface_module_type)) _interface_module_type)
  (:arm_laterality
   (&optional __arm_laterality)
   (if __arm_laterality (setq _arm_laterality __arm_laterality)) _arm_laterality)
  (:wrist_type
   (&optional __wrist_type)
   (if __wrist_type (setq _wrist_type __wrist_type)) _wrist_type)
  (:brake_type
   (&optional __brake_type)
   (if __brake_type (setq _brake_type __brake_type)) _brake_type)
  (:serialization-length
   ()
   (+
    ;; string _kin
    4 (length _kin)
    ;; uint32 _model
    4
    ;; kortex_driver/CountryCode _country_code
    (send _country_code :serialization-length)
    ;; string _assembly_plant
    4 (length _assembly_plant)
    ;; string _model_year
    4 (length _model_year)
    ;; uint32 _degree_of_freedom
    4
    ;; uint32 _base_type
    4
    ;; uint32 _end_effector_type
    4
    ;; uint32 _vision_module_type
    4
    ;; uint32 _interface_module_type
    4
    ;; uint32 _arm_laterality
    4
    ;; uint32 _wrist_type
    4
    ;; uint32 _brake_type
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _kin
       (write-long (length _kin) s) (princ _kin s)
     ;; uint32 _model
       (write-long _model s)
     ;; kortex_driver/CountryCode _country_code
       (send _country_code :serialize s)
     ;; string _assembly_plant
       (write-long (length _assembly_plant) s) (princ _assembly_plant s)
     ;; string _model_year
       (write-long (length _model_year) s) (princ _model_year s)
     ;; uint32 _degree_of_freedom
       (write-long _degree_of_freedom s)
     ;; uint32 _base_type
       (write-long _base_type s)
     ;; uint32 _end_effector_type
       (write-long _end_effector_type s)
     ;; uint32 _vision_module_type
       (write-long _vision_module_type s)
     ;; uint32 _interface_module_type
       (write-long _interface_module_type s)
     ;; uint32 _arm_laterality
       (write-long _arm_laterality s)
     ;; uint32 _wrist_type
       (write-long _wrist_type s)
     ;; uint32 _brake_type
       (write-long _brake_type s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _kin
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _kin (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _model
     (setq _model (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; kortex_driver/CountryCode _country_code
     (send _country_code :deserialize buf ptr-) (incf ptr- (send _country_code :serialization-length))
   ;; string _assembly_plant
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _assembly_plant (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _model_year
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _model_year (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _degree_of_freedom
     (setq _degree_of_freedom (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _base_type
     (setq _base_type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _end_effector_type
     (setq _end_effector_type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _vision_module_type
     (setq _vision_module_type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _interface_module_type
     (setq _interface_module_type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _arm_laterality
     (setq _arm_laterality (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _wrist_type
     (setq _wrist_type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _brake_type
     (setq _brake_type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::CompleteProductConfiguration :md5sum-) "b73bc25bff314a65c4c98c0970043ccf")
(setf (get kortex_driver::CompleteProductConfiguration :datatype-) "kortex_driver/CompleteProductConfiguration")
(setf (get kortex_driver::CompleteProductConfiguration :definition-)
      "
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

uint32 identifier
")



(provide :kortex_driver/CompleteProductConfiguration "b73bc25bff314a65c4c98c0970043ccf")


