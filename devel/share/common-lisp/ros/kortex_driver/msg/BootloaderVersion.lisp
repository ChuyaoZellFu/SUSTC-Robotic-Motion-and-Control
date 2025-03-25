; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude BootloaderVersion.msg.html

(cl:defclass <BootloaderVersion> (roslisp-msg-protocol:ros-message)
  ((bootloader_version
    :reader bootloader_version
    :initarg :bootloader_version
    :type cl:integer
    :initform 0))
)

(cl:defclass BootloaderVersion (<BootloaderVersion>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BootloaderVersion>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BootloaderVersion)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<BootloaderVersion> is deprecated: use kortex_driver-msg:BootloaderVersion instead.")))

(cl:ensure-generic-function 'bootloader_version-val :lambda-list '(m))
(cl:defmethod bootloader_version-val ((m <BootloaderVersion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:bootloader_version-val is deprecated.  Use kortex_driver-msg:bootloader_version instead.")
  (bootloader_version m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BootloaderVersion>) ostream)
  "Serializes a message object of type '<BootloaderVersion>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bootloader_version)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'bootloader_version)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'bootloader_version)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'bootloader_version)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BootloaderVersion>) istream)
  "Deserializes a message object of type '<BootloaderVersion>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bootloader_version)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'bootloader_version)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'bootloader_version)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'bootloader_version)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BootloaderVersion>)))
  "Returns string type for a message object of type '<BootloaderVersion>"
  "kortex_driver/BootloaderVersion")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BootloaderVersion)))
  "Returns string type for a message object of type 'BootloaderVersion"
  "kortex_driver/BootloaderVersion")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BootloaderVersion>)))
  "Returns md5sum for a message object of type '<BootloaderVersion>"
  "ec0c056f9a04b0432415c72e0d38271e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BootloaderVersion)))
  "Returns md5sum for a message object of type 'BootloaderVersion"
  "ec0c056f9a04b0432415c72e0d38271e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BootloaderVersion>)))
  "Returns full string definition for message of type '<BootloaderVersion>"
  (cl:format cl:nil "~%uint32 bootloader_version~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BootloaderVersion)))
  "Returns full string definition for message of type 'BootloaderVersion"
  (cl:format cl:nil "~%uint32 bootloader_version~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BootloaderVersion>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BootloaderVersion>))
  "Converts a ROS message object to a list"
  (cl:list 'BootloaderVersion
    (cl:cons ':bootloader_version (bootloader_version msg))
))
