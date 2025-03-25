; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude FirmwareVersion.msg.html

(cl:defclass <FirmwareVersion> (roslisp-msg-protocol:ros-message)
  ((firmware_version
    :reader firmware_version
    :initarg :firmware_version
    :type cl:integer
    :initform 0))
)

(cl:defclass FirmwareVersion (<FirmwareVersion>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FirmwareVersion>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FirmwareVersion)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<FirmwareVersion> is deprecated: use kortex_driver-msg:FirmwareVersion instead.")))

(cl:ensure-generic-function 'firmware_version-val :lambda-list '(m))
(cl:defmethod firmware_version-val ((m <FirmwareVersion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:firmware_version-val is deprecated.  Use kortex_driver-msg:firmware_version instead.")
  (firmware_version m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FirmwareVersion>) ostream)
  "Serializes a message object of type '<FirmwareVersion>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'firmware_version)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'firmware_version)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'firmware_version)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'firmware_version)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FirmwareVersion>) istream)
  "Deserializes a message object of type '<FirmwareVersion>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'firmware_version)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'firmware_version)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'firmware_version)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'firmware_version)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FirmwareVersion>)))
  "Returns string type for a message object of type '<FirmwareVersion>"
  "kortex_driver/FirmwareVersion")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FirmwareVersion)))
  "Returns string type for a message object of type 'FirmwareVersion"
  "kortex_driver/FirmwareVersion")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FirmwareVersion>)))
  "Returns md5sum for a message object of type '<FirmwareVersion>"
  "f0cdf3021edc70c033ace598cc19b3d0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FirmwareVersion)))
  "Returns md5sum for a message object of type 'FirmwareVersion"
  "f0cdf3021edc70c033ace598cc19b3d0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FirmwareVersion>)))
  "Returns full string definition for message of type '<FirmwareVersion>"
  (cl:format cl:nil "~%uint32 firmware_version~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FirmwareVersion)))
  "Returns full string definition for message of type 'FirmwareVersion"
  (cl:format cl:nil "~%uint32 firmware_version~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FirmwareVersion>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FirmwareVersion>))
  "Converts a ROS message object to a list"
  (cl:list 'FirmwareVersion
    (cl:cons ':firmware_version (firmware_version msg))
))
