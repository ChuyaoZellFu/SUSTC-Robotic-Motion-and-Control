; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude I2CReadParameter.msg.html

(cl:defclass <I2CReadParameter> (roslisp-msg-protocol:ros-message)
  ((device
    :reader device
    :initarg :device
    :type cl:integer
    :initform 0)
   (device_address
    :reader device_address
    :initarg :device_address
    :type cl:integer
    :initform 0)
   (size
    :reader size
    :initarg :size
    :type cl:integer
    :initform 0)
   (timeout
    :reader timeout
    :initarg :timeout
    :type cl:integer
    :initform 0))
)

(cl:defclass I2CReadParameter (<I2CReadParameter>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <I2CReadParameter>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'I2CReadParameter)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<I2CReadParameter> is deprecated: use kortex_driver-msg:I2CReadParameter instead.")))

(cl:ensure-generic-function 'device-val :lambda-list '(m))
(cl:defmethod device-val ((m <I2CReadParameter>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:device-val is deprecated.  Use kortex_driver-msg:device instead.")
  (device m))

(cl:ensure-generic-function 'device_address-val :lambda-list '(m))
(cl:defmethod device_address-val ((m <I2CReadParameter>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:device_address-val is deprecated.  Use kortex_driver-msg:device_address instead.")
  (device_address m))

(cl:ensure-generic-function 'size-val :lambda-list '(m))
(cl:defmethod size-val ((m <I2CReadParameter>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:size-val is deprecated.  Use kortex_driver-msg:size instead.")
  (size m))

(cl:ensure-generic-function 'timeout-val :lambda-list '(m))
(cl:defmethod timeout-val ((m <I2CReadParameter>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:timeout-val is deprecated.  Use kortex_driver-msg:timeout instead.")
  (timeout m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <I2CReadParameter>) ostream)
  "Serializes a message object of type '<I2CReadParameter>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'device)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'device)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'device)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'device)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'device_address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'device_address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'device_address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'device_address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'timeout)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'timeout)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'timeout)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'timeout)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <I2CReadParameter>) istream)
  "Deserializes a message object of type '<I2CReadParameter>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'device)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'device)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'device)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'device)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'device_address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'device_address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'device_address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'device_address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'timeout)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'timeout)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'timeout)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'timeout)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<I2CReadParameter>)))
  "Returns string type for a message object of type '<I2CReadParameter>"
  "kortex_driver/I2CReadParameter")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'I2CReadParameter)))
  "Returns string type for a message object of type 'I2CReadParameter"
  "kortex_driver/I2CReadParameter")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<I2CReadParameter>)))
  "Returns md5sum for a message object of type '<I2CReadParameter>"
  "66188a52c1e1bb7e3a095189a7bded0c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'I2CReadParameter)))
  "Returns md5sum for a message object of type 'I2CReadParameter"
  "66188a52c1e1bb7e3a095189a7bded0c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<I2CReadParameter>)))
  "Returns full string definition for message of type '<I2CReadParameter>"
  (cl:format cl:nil "~%uint32 device~%uint32 device_address~%uint32 size~%uint32 timeout~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'I2CReadParameter)))
  "Returns full string definition for message of type 'I2CReadParameter"
  (cl:format cl:nil "~%uint32 device~%uint32 device_address~%uint32 size~%uint32 timeout~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <I2CReadParameter>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <I2CReadParameter>))
  "Converts a ROS message object to a list"
  (cl:list 'I2CReadParameter
    (cl:cons ':device (device msg))
    (cl:cons ':device_address (device_address msg))
    (cl:cons ':size (size msg))
    (cl:cons ':timeout (timeout msg))
))
