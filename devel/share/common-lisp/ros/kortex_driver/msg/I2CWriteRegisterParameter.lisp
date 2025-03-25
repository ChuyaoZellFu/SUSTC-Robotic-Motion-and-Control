; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude I2CWriteRegisterParameter.msg.html

(cl:defclass <I2CWriteRegisterParameter> (roslisp-msg-protocol:ros-message)
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
   (register_address
    :reader register_address
    :initarg :register_address
    :type cl:integer
    :initform 0)
   (register_address_size
    :reader register_address_size
    :initarg :register_address_size
    :type cl:integer
    :initform 0)
   (timeout
    :reader timeout
    :initarg :timeout
    :type cl:integer
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type kortex_driver-msg:I2CData
    :initform (cl:make-instance 'kortex_driver-msg:I2CData)))
)

(cl:defclass I2CWriteRegisterParameter (<I2CWriteRegisterParameter>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <I2CWriteRegisterParameter>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'I2CWriteRegisterParameter)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<I2CWriteRegisterParameter> is deprecated: use kortex_driver-msg:I2CWriteRegisterParameter instead.")))

(cl:ensure-generic-function 'device-val :lambda-list '(m))
(cl:defmethod device-val ((m <I2CWriteRegisterParameter>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:device-val is deprecated.  Use kortex_driver-msg:device instead.")
  (device m))

(cl:ensure-generic-function 'device_address-val :lambda-list '(m))
(cl:defmethod device_address-val ((m <I2CWriteRegisterParameter>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:device_address-val is deprecated.  Use kortex_driver-msg:device_address instead.")
  (device_address m))

(cl:ensure-generic-function 'register_address-val :lambda-list '(m))
(cl:defmethod register_address-val ((m <I2CWriteRegisterParameter>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:register_address-val is deprecated.  Use kortex_driver-msg:register_address instead.")
  (register_address m))

(cl:ensure-generic-function 'register_address_size-val :lambda-list '(m))
(cl:defmethod register_address_size-val ((m <I2CWriteRegisterParameter>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:register_address_size-val is deprecated.  Use kortex_driver-msg:register_address_size instead.")
  (register_address_size m))

(cl:ensure-generic-function 'timeout-val :lambda-list '(m))
(cl:defmethod timeout-val ((m <I2CWriteRegisterParameter>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:timeout-val is deprecated.  Use kortex_driver-msg:timeout instead.")
  (timeout m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <I2CWriteRegisterParameter>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:data-val is deprecated.  Use kortex_driver-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <I2CWriteRegisterParameter>) ostream)
  "Serializes a message object of type '<I2CWriteRegisterParameter>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'device)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'device)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'device)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'device)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'device_address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'device_address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'device_address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'device_address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'register_address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'register_address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'register_address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'register_address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'register_address_size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'register_address_size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'register_address_size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'register_address_size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'timeout)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'timeout)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'timeout)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'timeout)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'data) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <I2CWriteRegisterParameter>) istream)
  "Deserializes a message object of type '<I2CWriteRegisterParameter>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'device)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'device)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'device)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'device)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'device_address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'device_address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'device_address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'device_address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'register_address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'register_address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'register_address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'register_address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'register_address_size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'register_address_size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'register_address_size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'register_address_size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'timeout)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'timeout)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'timeout)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'timeout)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'data) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<I2CWriteRegisterParameter>)))
  "Returns string type for a message object of type '<I2CWriteRegisterParameter>"
  "kortex_driver/I2CWriteRegisterParameter")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'I2CWriteRegisterParameter)))
  "Returns string type for a message object of type 'I2CWriteRegisterParameter"
  "kortex_driver/I2CWriteRegisterParameter")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<I2CWriteRegisterParameter>)))
  "Returns md5sum for a message object of type '<I2CWriteRegisterParameter>"
  "347168899c8760463d3fc21c4d811648")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'I2CWriteRegisterParameter)))
  "Returns md5sum for a message object of type 'I2CWriteRegisterParameter"
  "347168899c8760463d3fc21c4d811648")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<I2CWriteRegisterParameter>)))
  "Returns full string definition for message of type '<I2CWriteRegisterParameter>"
  (cl:format cl:nil "~%uint32 device~%uint32 device_address~%uint32 register_address~%uint32 register_address_size~%uint32 timeout~%I2CData data~%================================================================================~%MSG: kortex_driver/I2CData~%~%uint8[] data~%uint32 size~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'I2CWriteRegisterParameter)))
  "Returns full string definition for message of type 'I2CWriteRegisterParameter"
  (cl:format cl:nil "~%uint32 device~%uint32 device_address~%uint32 register_address~%uint32 register_address_size~%uint32 timeout~%I2CData data~%================================================================================~%MSG: kortex_driver/I2CData~%~%uint8[] data~%uint32 size~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <I2CWriteRegisterParameter>))
  (cl:+ 0
     4
     4
     4
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <I2CWriteRegisterParameter>))
  "Converts a ROS message object to a list"
  (cl:list 'I2CWriteRegisterParameter
    (cl:cons ':device (device msg))
    (cl:cons ':device_address (device_address msg))
    (cl:cons ':register_address (register_address msg))
    (cl:cons ':register_address_size (register_address_size msg))
    (cl:cons ':timeout (timeout msg))
    (cl:cons ':data (data msg))
))
