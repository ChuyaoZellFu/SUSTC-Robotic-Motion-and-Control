;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ProtectionZoneList)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ProtectionZoneList (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::PROTECTIONZONELIST")
  (make-package "KORTEX_DRIVER::PROTECTIONZONELIST"))

(in-package "ROS")
;;//! \htmlinclude ProtectionZoneList.msg.html


(defclass kortex_driver::ProtectionZoneList
  :super ros::object
  :slots (_protection_zones ))

(defmethod kortex_driver::ProtectionZoneList
  (:init
   (&key
    ((:protection_zones __protection_zones) ())
    )
   (send-super :init)
   (setq _protection_zones __protection_zones)
   self)
  (:protection_zones
   (&rest __protection_zones)
   (if (keywordp (car __protection_zones))
       (send* _protection_zones __protection_zones)
     (progn
       (if __protection_zones (setq _protection_zones (car __protection_zones)))
       _protection_zones)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/ProtectionZone[] _protection_zones
    (apply #'+ (send-all _protection_zones :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ProtectionZone[] _protection_zones
     (write-long (length _protection_zones) s)
     (dolist (elem _protection_zones)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ProtectionZone[] _protection_zones
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _protection_zones (let (r) (dotimes (i n) (push (instance kortex_driver::ProtectionZone :init) r)) r))
     (dolist (elem- _protection_zones)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::ProtectionZoneList :md5sum-) "a08108637290aeb7551436f88ea3c88f")
(setf (get kortex_driver::ProtectionZoneList :datatype-) "kortex_driver/ProtectionZoneList")
(setf (get kortex_driver::ProtectionZoneList :definition-)
      "
ProtectionZone[] protection_zones
================================================================================
MSG: kortex_driver/ProtectionZone

ProtectionZoneHandle handle
string name
string application_data
bool is_enabled
ZoneShape shape
CartesianLimitation[] limitations
CartesianLimitation[] envelope_limitations
================================================================================
MSG: kortex_driver/ProtectionZoneHandle

uint32 identifier
uint32 permission
================================================================================
MSG: kortex_driver/ZoneShape

uint32 shape_type
Point origin
Base_RotationMatrix orientation
float32[] dimensions
float32 envelope_thickness
================================================================================
MSG: kortex_driver/Point

float32 x
float32 y
float32 z
================================================================================
MSG: kortex_driver/Base_RotationMatrix

Base_RotationMatrixRow row1
Base_RotationMatrixRow row2
Base_RotationMatrixRow row3
================================================================================
MSG: kortex_driver/Base_RotationMatrixRow

float32 column1
float32 column2
float32 column3
================================================================================
MSG: kortex_driver/CartesianLimitation

uint32 type
float32 translation
float32 orientation
")



(provide :kortex_driver/ProtectionZoneList "a08108637290aeb7551436f88ea3c88f")


