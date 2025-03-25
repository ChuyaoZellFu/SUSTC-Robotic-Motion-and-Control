; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude DeviceHandle.msg.html

(cl:defclass <DeviceHandle> (roslisp-msg-protocol:ros-message)
  ((device_type
    :reader device_type
    :initarg :device_type
    :type cl:integer
    :initform 0)
   (device_identifier
    :reader device_identifier
    :initarg :device_identifier
    :type cl:integer
    :initform 0)
   (order
    :reader order
    :initarg :order
    :type cl:integer
    :initform 0))
)

(cl:defclass DeviceHandle (<DeviceHandle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeviceHandle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeviceHandle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<DeviceHandle> is deprecated: use kortex_driver-msg:DeviceHandle instead.")))

(cl:ensure-generic-function 'device_type-val :lambda-list '(m))
(cl:defmethod device_type-val ((m <DeviceHandle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:device_type-val is deprecated.  Use kortex_driver-msg:device_type instead.")
  (device_type m))

(cl:ensure-generic-function 'device_identifier-val :lambda-list '(m))
(cl:defmethod device_identifier-val ((m <DeviceHandle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:device_identifier-val is deprecated.  Use kortex_driver-msg:device_identifier instead.")
  (device_identifier m))

(cl:ensure-generic-function 'order-val :lambda-list '(m))
(cl:defmethod order-val ((m <DeviceHandle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:order-val is deprecated.  Use kortex_driver-msg:order instead.")
  (order m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeviceHandle>) ostream)
  "Serializes a message object of type '<DeviceHandle>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'device_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'device_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'device_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'device_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'device_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'device_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'device_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'device_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'order)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'order)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'order)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'order)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeviceHandle>) istream)
  "Deserializes a message object of type '<DeviceHandle>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'device_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'device_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'device_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'device_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'device_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'device_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'device_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'device_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'order)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'order)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'order)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'order)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeviceHandle>)))
  "Returns string type for a message object of type '<DeviceHandle>"
  "kortex_driver/DeviceHandle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeviceHandle)))
  "Returns string type for a message object of type 'DeviceHandle"
  "kortex_driver/DeviceHandle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeviceHandle>)))
  "Returns md5sum for a message object of type '<DeviceHandle>"
  "a89d226be38807777119fd9acd7acb3b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeviceHandle)))
  "Returns md5sum for a message object of type 'DeviceHandle"
  "a89d226be38807777119fd9acd7acb3b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeviceHandle>)))
  "Returns full string definition for message of type '<DeviceHandle>"
  (cl:format cl:nil "~%uint32 device_type~%uint32 device_identifier~%uint32 order~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeviceHandle)))
  "Returns full string definition for message of type 'DeviceHandle"
  (cl:format cl:nil "~%uint32 device_type~%uint32 device_identifier~%uint32 order~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeviceHandle>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeviceHandle>))
  "Converts a ROS message object to a list"
  (cl:list 'DeviceHandle
    (cl:cons ':device_type (device_type msg))
    (cl:cons ':device_identifier (device_identifier msg))
    (cl:cons ':order (order msg))
))
