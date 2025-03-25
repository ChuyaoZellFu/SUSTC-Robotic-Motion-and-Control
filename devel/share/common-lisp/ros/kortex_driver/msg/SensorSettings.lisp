; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude SensorSettings.msg.html

(cl:defclass <SensorSettings> (roslisp-msg-protocol:ros-message)
  ((sensor
    :reader sensor
    :initarg :sensor
    :type cl:integer
    :initform 0)
   (resolution
    :reader resolution
    :initarg :resolution
    :type cl:integer
    :initform 0)
   (frame_rate
    :reader frame_rate
    :initarg :frame_rate
    :type cl:integer
    :initform 0)
   (bit_rate
    :reader bit_rate
    :initarg :bit_rate
    :type cl:integer
    :initform 0))
)

(cl:defclass SensorSettings (<SensorSettings>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SensorSettings>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SensorSettings)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<SensorSettings> is deprecated: use kortex_driver-msg:SensorSettings instead.")))

(cl:ensure-generic-function 'sensor-val :lambda-list '(m))
(cl:defmethod sensor-val ((m <SensorSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:sensor-val is deprecated.  Use kortex_driver-msg:sensor instead.")
  (sensor m))

(cl:ensure-generic-function 'resolution-val :lambda-list '(m))
(cl:defmethod resolution-val ((m <SensorSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:resolution-val is deprecated.  Use kortex_driver-msg:resolution instead.")
  (resolution m))

(cl:ensure-generic-function 'frame_rate-val :lambda-list '(m))
(cl:defmethod frame_rate-val ((m <SensorSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:frame_rate-val is deprecated.  Use kortex_driver-msg:frame_rate instead.")
  (frame_rate m))

(cl:ensure-generic-function 'bit_rate-val :lambda-list '(m))
(cl:defmethod bit_rate-val ((m <SensorSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:bit_rate-val is deprecated.  Use kortex_driver-msg:bit_rate instead.")
  (bit_rate m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SensorSettings>) ostream)
  "Serializes a message object of type '<SensorSettings>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sensor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sensor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sensor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sensor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'resolution)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'resolution)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'resolution)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'resolution)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'frame_rate)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'frame_rate)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'frame_rate)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'frame_rate)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bit_rate)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'bit_rate)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'bit_rate)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'bit_rate)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SensorSettings>) istream)
  "Deserializes a message object of type '<SensorSettings>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sensor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sensor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sensor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sensor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'resolution)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'resolution)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'resolution)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'resolution)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'frame_rate)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'frame_rate)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'frame_rate)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'frame_rate)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bit_rate)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'bit_rate)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'bit_rate)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'bit_rate)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SensorSettings>)))
  "Returns string type for a message object of type '<SensorSettings>"
  "kortex_driver/SensorSettings")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SensorSettings)))
  "Returns string type for a message object of type 'SensorSettings"
  "kortex_driver/SensorSettings")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SensorSettings>)))
  "Returns md5sum for a message object of type '<SensorSettings>"
  "1b1f6cd3d6eb9aa0d6912b66a0f489bd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SensorSettings)))
  "Returns md5sum for a message object of type 'SensorSettings"
  "1b1f6cd3d6eb9aa0d6912b66a0f489bd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SensorSettings>)))
  "Returns full string definition for message of type '<SensorSettings>"
  (cl:format cl:nil "~%uint32 sensor~%uint32 resolution~%uint32 frame_rate~%uint32 bit_rate~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SensorSettings)))
  "Returns full string definition for message of type 'SensorSettings"
  (cl:format cl:nil "~%uint32 sensor~%uint32 resolution~%uint32 frame_rate~%uint32 bit_rate~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SensorSettings>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SensorSettings>))
  "Converts a ROS message object to a list"
  (cl:list 'SensorSettings
    (cl:cons ':sensor (sensor msg))
    (cl:cons ':resolution (resolution msg))
    (cl:cons ':frame_rate (frame_rate msg))
    (cl:cons ':bit_rate (bit_rate msg))
))
