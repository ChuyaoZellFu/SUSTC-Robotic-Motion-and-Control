;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::CreateProtectionZone)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'CreateProtectionZone (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CREATEPROTECTIONZONE")
  (make-package "KORTEX_DRIVER::CREATEPROTECTIONZONE"))
(unless (find-package "KORTEX_DRIVER::CREATEPROTECTIONZONEREQUEST")
  (make-package "KORTEX_DRIVER::CREATEPROTECTIONZONEREQUEST"))
(unless (find-package "KORTEX_DRIVER::CREATEPROTECTIONZONERESPONSE")
  (make-package "KORTEX_DRIVER::CREATEPROTECTIONZONERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::CreateProtectionZoneRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::CreateProtectionZoneRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::ProtectionZone :init))
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
    ;; kortex_driver/ProtectionZone _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ProtectionZone _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ProtectionZone _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::CreateProtectionZoneResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::CreateProtectionZoneResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::ProtectionZoneHandle :init))
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
    ;; kortex_driver/ProtectionZoneHandle _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ProtectionZoneHandle _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ProtectionZoneHandle _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::CreateProtectionZone
  :super ros::object
  :slots ())

(setf (get kortex_driver::CreateProtectionZone :md5sum-) "e5380d610764b507278413b8ea5bd27f")
(setf (get kortex_driver::CreateProtectionZone :datatype-) "kortex_driver/CreateProtectionZone")
(setf (get kortex_driver::CreateProtectionZone :request) kortex_driver::CreateProtectionZoneRequest)
(setf (get kortex_driver::CreateProtectionZone :response) kortex_driver::CreateProtectionZoneResponse)

(defmethod kortex_driver::CreateProtectionZoneRequest
  (:response () (instance kortex_driver::CreateProtectionZoneResponse :init)))

(setf (get kortex_driver::CreateProtectionZoneRequest :md5sum-) "e5380d610764b507278413b8ea5bd27f")
(setf (get kortex_driver::CreateProtectionZoneRequest :datatype-) "kortex_driver/CreateProtectionZoneRequest")
(setf (get kortex_driver::CreateProtectionZoneRequest :definition-)
      "ProtectionZone input

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
---
ProtectionZoneHandle output

================================================================================
MSG: kortex_driver/ProtectionZoneHandle

uint32 identifier
")

(setf (get kortex_driver::CreateProtectionZoneResponse :md5sum-) "e5380d610764b507278413b8ea5bd27f")
(setf (get kortex_driver::CreateProtectionZoneResponse :datatype-) "kortex_driver/CreateProtectionZoneResponse")
(setf (get kortex_driver::CreateProtectionZoneResponse :definition-)
      "ProtectionZone input

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
---
ProtectionZoneHandle output

================================================================================
MSG: kortex_driver/ProtectionZoneHandle

uint32 identifier
")



(provide :kortex_driver/CreateProtectionZone "e5380d610764b507278413b8ea5bd27f")


