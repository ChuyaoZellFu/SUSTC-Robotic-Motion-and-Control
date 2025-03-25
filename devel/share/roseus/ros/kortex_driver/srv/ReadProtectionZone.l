;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ReadProtectionZone)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ReadProtectionZone (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::READPROTECTIONZONE")
  (make-package "KORTEX_DRIVER::READPROTECTIONZONE"))
(unless (find-package "KORTEX_DRIVER::READPROTECTIONZONEREQUEST")
  (make-package "KORTEX_DRIVER::READPROTECTIONZONEREQUEST"))
(unless (find-package "KORTEX_DRIVER::READPROTECTIONZONERESPONSE")
  (make-package "KORTEX_DRIVER::READPROTECTIONZONERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::ReadProtectionZoneRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::ReadProtectionZoneRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::ProtectionZoneHandle :init))
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
    ;; kortex_driver/ProtectionZoneHandle _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ProtectionZoneHandle _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ProtectionZoneHandle _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::ReadProtectionZoneResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::ReadProtectionZoneResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::ProtectionZone :init))
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
    ;; kortex_driver/ProtectionZone _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ProtectionZone _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ProtectionZone _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::ReadProtectionZone
  :super ros::object
  :slots ())

(setf (get kortex_driver::ReadProtectionZone :md5sum-) "fd53b25b5f35cd926da1c19403c70bc5")
(setf (get kortex_driver::ReadProtectionZone :datatype-) "kortex_driver/ReadProtectionZone")
(setf (get kortex_driver::ReadProtectionZone :request) kortex_driver::ReadProtectionZoneRequest)
(setf (get kortex_driver::ReadProtectionZone :response) kortex_driver::ReadProtectionZoneResponse)

(defmethod kortex_driver::ReadProtectionZoneRequest
  (:response () (instance kortex_driver::ReadProtectionZoneResponse :init)))

(setf (get kortex_driver::ReadProtectionZoneRequest :md5sum-) "fd53b25b5f35cd926da1c19403c70bc5")
(setf (get kortex_driver::ReadProtectionZoneRequest :datatype-) "kortex_driver/ReadProtectionZoneRequest")
(setf (get kortex_driver::ReadProtectionZoneRequest :definition-)
      "ProtectionZoneHandle input

================================================================================
MSG: kortex_driver/ProtectionZoneHandle

uint32 identifier
---
ProtectionZone output

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

(setf (get kortex_driver::ReadProtectionZoneResponse :md5sum-) "fd53b25b5f35cd926da1c19403c70bc5")
(setf (get kortex_driver::ReadProtectionZoneResponse :datatype-) "kortex_driver/ReadProtectionZoneResponse")
(setf (get kortex_driver::ReadProtectionZoneResponse :definition-)
      "ProtectionZoneHandle input

================================================================================
MSG: kortex_driver/ProtectionZoneHandle

uint32 identifier
---
ProtectionZone output

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



(provide :kortex_driver/ReadProtectionZone "fd53b25b5f35cd926da1c19403c70bc5")


