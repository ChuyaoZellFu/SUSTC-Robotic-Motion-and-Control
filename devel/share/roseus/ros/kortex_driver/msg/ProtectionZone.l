;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ProtectionZone)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ProtectionZone (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::PROTECTIONZONE")
  (make-package "KORTEX_DRIVER::PROTECTIONZONE"))

(in-package "ROS")
;;//! \htmlinclude ProtectionZone.msg.html


(defclass kortex_driver::ProtectionZone
  :super ros::object
  :slots (_handle _name _application_data _is_enabled _shape _limitations _envelope_limitations ))

(defmethod kortex_driver::ProtectionZone
  (:init
   (&key
    ((:handle __handle) (instance kortex_driver::ProtectionZoneHandle :init))
    ((:name __name) "")
    ((:application_data __application_data) "")
    ((:is_enabled __is_enabled) nil)
    ((:shape __shape) (instance kortex_driver::ZoneShape :init))
    ((:limitations __limitations) ())
    ((:envelope_limitations __envelope_limitations) ())
    )
   (send-super :init)
   (setq _handle __handle)
   (setq _name (string __name))
   (setq _application_data (string __application_data))
   (setq _is_enabled __is_enabled)
   (setq _shape __shape)
   (setq _limitations __limitations)
   (setq _envelope_limitations __envelope_limitations)
   self)
  (:handle
   (&rest __handle)
   (if (keywordp (car __handle))
       (send* _handle __handle)
     (progn
       (if __handle (setq _handle (car __handle)))
       _handle)))
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:application_data
   (&optional __application_data)
   (if __application_data (setq _application_data __application_data)) _application_data)
  (:is_enabled
   (&optional (__is_enabled :null))
   (if (not (eq __is_enabled :null)) (setq _is_enabled __is_enabled)) _is_enabled)
  (:shape
   (&rest __shape)
   (if (keywordp (car __shape))
       (send* _shape __shape)
     (progn
       (if __shape (setq _shape (car __shape)))
       _shape)))
  (:limitations
   (&rest __limitations)
   (if (keywordp (car __limitations))
       (send* _limitations __limitations)
     (progn
       (if __limitations (setq _limitations (car __limitations)))
       _limitations)))
  (:envelope_limitations
   (&rest __envelope_limitations)
   (if (keywordp (car __envelope_limitations))
       (send* _envelope_limitations __envelope_limitations)
     (progn
       (if __envelope_limitations (setq _envelope_limitations (car __envelope_limitations)))
       _envelope_limitations)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/ProtectionZoneHandle _handle
    (send _handle :serialization-length)
    ;; string _name
    4 (length _name)
    ;; string _application_data
    4 (length _application_data)
    ;; bool _is_enabled
    1
    ;; kortex_driver/ZoneShape _shape
    (send _shape :serialization-length)
    ;; kortex_driver/CartesianLimitation[] _limitations
    (apply #'+ (send-all _limitations :serialization-length)) 4
    ;; kortex_driver/CartesianLimitation[] _envelope_limitations
    (apply #'+ (send-all _envelope_limitations :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ProtectionZoneHandle _handle
       (send _handle :serialize s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; string _application_data
       (write-long (length _application_data) s) (princ _application_data s)
     ;; bool _is_enabled
       (if _is_enabled (write-byte -1 s) (write-byte 0 s))
     ;; kortex_driver/ZoneShape _shape
       (send _shape :serialize s)
     ;; kortex_driver/CartesianLimitation[] _limitations
     (write-long (length _limitations) s)
     (dolist (elem _limitations)
       (send elem :serialize s)
       )
     ;; kortex_driver/CartesianLimitation[] _envelope_limitations
     (write-long (length _envelope_limitations) s)
     (dolist (elem _envelope_limitations)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ProtectionZoneHandle _handle
     (send _handle :deserialize buf ptr-) (incf ptr- (send _handle :serialization-length))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _application_data
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _application_data (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _is_enabled
     (setq _is_enabled (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; kortex_driver/ZoneShape _shape
     (send _shape :deserialize buf ptr-) (incf ptr- (send _shape :serialization-length))
   ;; kortex_driver/CartesianLimitation[] _limitations
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _limitations (let (r) (dotimes (i n) (push (instance kortex_driver::CartesianLimitation :init) r)) r))
     (dolist (elem- _limitations)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kortex_driver/CartesianLimitation[] _envelope_limitations
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _envelope_limitations (let (r) (dotimes (i n) (push (instance kortex_driver::CartesianLimitation :init) r)) r))
     (dolist (elem- _envelope_limitations)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::ProtectionZone :md5sum-) "9648b3590ddbe012236cdc588d3f6ef8")
(setf (get kortex_driver::ProtectionZone :datatype-) "kortex_driver/ProtectionZone")
(setf (get kortex_driver::ProtectionZone :definition-)
      "
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



(provide :kortex_driver/ProtectionZone "9648b3590ddbe012236cdc588d3f6ef8")


