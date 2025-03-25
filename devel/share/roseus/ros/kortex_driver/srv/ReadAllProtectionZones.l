;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ReadAllProtectionZones)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ReadAllProtectionZones (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::READALLPROTECTIONZONES")
  (make-package "KORTEX_DRIVER::READALLPROTECTIONZONES"))
(unless (find-package "KORTEX_DRIVER::READALLPROTECTIONZONESREQUEST")
  (make-package "KORTEX_DRIVER::READALLPROTECTIONZONESREQUEST"))
(unless (find-package "KORTEX_DRIVER::READALLPROTECTIONZONESRESPONSE")
  (make-package "KORTEX_DRIVER::READALLPROTECTIONZONESRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::ReadAllProtectionZonesRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::ReadAllProtectionZonesRequest
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

(defclass kortex_driver::ReadAllProtectionZonesResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::ReadAllProtectionZonesResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::ProtectionZoneList :init))
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
    ;; kortex_driver/ProtectionZoneList _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ProtectionZoneList _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ProtectionZoneList _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::ReadAllProtectionZones
  :super ros::object
  :slots ())

(setf (get kortex_driver::ReadAllProtectionZones :md5sum-) "26bc18c08c1b8389444ea1d92e6c8745")
(setf (get kortex_driver::ReadAllProtectionZones :datatype-) "kortex_driver/ReadAllProtectionZones")
(setf (get kortex_driver::ReadAllProtectionZones :request) kortex_driver::ReadAllProtectionZonesRequest)
(setf (get kortex_driver::ReadAllProtectionZones :response) kortex_driver::ReadAllProtectionZonesResponse)

(defmethod kortex_driver::ReadAllProtectionZonesRequest
  (:response () (instance kortex_driver::ReadAllProtectionZonesResponse :init)))

(setf (get kortex_driver::ReadAllProtectionZonesRequest :md5sum-) "26bc18c08c1b8389444ea1d92e6c8745")
(setf (get kortex_driver::ReadAllProtectionZonesRequest :datatype-) "kortex_driver/ReadAllProtectionZonesRequest")
(setf (get kortex_driver::ReadAllProtectionZonesRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
ProtectionZoneList output

================================================================================
MSG: kortex_driver/ProtectionZoneList

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
")

(setf (get kortex_driver::ReadAllProtectionZonesResponse :md5sum-) "26bc18c08c1b8389444ea1d92e6c8745")
(setf (get kortex_driver::ReadAllProtectionZonesResponse :datatype-) "kortex_driver/ReadAllProtectionZonesResponse")
(setf (get kortex_driver::ReadAllProtectionZonesResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
ProtectionZoneList output

================================================================================
MSG: kortex_driver/ProtectionZoneList

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
")



(provide :kortex_driver/ReadAllProtectionZones "26bc18c08c1b8389444ea1d92e6c8745")


