; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude FirmwareComponentVersion.msg.html

(cl:defclass <FirmwareComponentVersion> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (version
    :reader version
    :initarg :version
    :type cl:string
    :initform "")
   (device_id
    :reader device_id
    :initarg :device_id
    :type cl:integer
    :initform 0))
)

(cl:defclass FirmwareComponentVersion (<FirmwareComponentVersion>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FirmwareComponentVersion>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FirmwareComponentVersion)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<FirmwareComponentVersion> is deprecated: use kortex_driver-msg:FirmwareComponentVersion instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <FirmwareComponentVersion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:name-val is deprecated.  Use kortex_driver-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'version-val :lambda-list '(m))
(cl:defmethod version-val ((m <FirmwareComponentVersion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:version-val is deprecated.  Use kortex_driver-msg:version instead.")
  (version m))

(cl:ensure-generic-function 'device_id-val :lambda-list '(m))
(cl:defmethod device_id-val ((m <FirmwareComponentVersion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:device_id-val is deprecated.  Use kortex_driver-msg:device_id instead.")
  (device_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FirmwareComponentVersion>) ostream)
  "Serializes a message object of type '<FirmwareComponentVersion>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'version))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'version))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'device_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'device_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'device_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'device_id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FirmwareComponentVersion>) istream)
  "Deserializes a message object of type '<FirmwareComponentVersion>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'version) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'version) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'device_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'device_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'device_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'device_id)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FirmwareComponentVersion>)))
  "Returns string type for a message object of type '<FirmwareComponentVersion>"
  "kortex_driver/FirmwareComponentVersion")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FirmwareComponentVersion)))
  "Returns string type for a message object of type 'FirmwareComponentVersion"
  "kortex_driver/FirmwareComponentVersion")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FirmwareComponentVersion>)))
  "Returns md5sum for a message object of type '<FirmwareComponentVersion>"
  "33679c8f8b46a7fffb7ad238111a28bb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FirmwareComponentVersion)))
  "Returns md5sum for a message object of type 'FirmwareComponentVersion"
  "33679c8f8b46a7fffb7ad238111a28bb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FirmwareComponentVersion>)))
  "Returns full string definition for message of type '<FirmwareComponentVersion>"
  (cl:format cl:nil "~%string name~%string version~%uint32 device_id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FirmwareComponentVersion)))
  "Returns full string definition for message of type 'FirmwareComponentVersion"
  (cl:format cl:nil "~%string name~%string version~%uint32 device_id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FirmwareComponentVersion>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4 (cl:length (cl:slot-value msg 'version))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FirmwareComponentVersion>))
  "Converts a ROS message object to a list"
  (cl:list 'FirmwareComponentVersion
    (cl:cons ':name (name msg))
    (cl:cons ':version (version msg))
    (cl:cons ':device_id (device_id msg))
))
