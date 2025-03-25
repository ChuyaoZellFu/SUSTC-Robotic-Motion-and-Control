; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude I2CWriteParameter.msg.html

(cl:defclass <I2CWriteParameter> (roslisp-msg-protocol:ros-message)
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

(cl:defclass I2CWriteParameter (<I2CWriteParameter>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <I2CWriteParameter>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'I2CWriteParameter)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<I2CWriteParameter> is deprecated: use kortex_driver-msg:I2CWriteParameter instead.")))

(cl:ensure-generic-function 'device-val :lambda-list '(m))
(cl:defmethod device-val ((m <I2CWriteParameter>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:device-val is deprecated.  Use kortex_driver-msg:device instead.")
  (device m))

(cl:ensure-generic-function 'device_address-val :lambda-list '(m))
(cl:defmethod device_address-val ((m <I2CWriteParameter>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:device_address-val is deprecated.  Use kortex_driver-msg:device_address instead.")
  (device_address m))

(cl:ensure-generic-function 'timeout-val :lambda-list '(m))
(cl:defmethod timeout-val ((m <I2CWriteParameter>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:timeout-val is deprecated.  Use kortex_driver-msg:timeout instead.")
  (timeout m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <I2CWriteParameter>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:data-val is deprecated.  Use kortex_driver-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <I2CWriteParameter>) ostream)
  "Serializes a message object of type '<I2CWriteParameter>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'device)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'device)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'device)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'device)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'device_address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'device_address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'device_address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'device_address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'timeout)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'timeout)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'timeout)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'timeout)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'data) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <I2CWriteParameter>) istream)
  "Deserializes a message object of type '<I2CWriteParameter>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'device)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'device)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'device)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'device)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'device_address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'device_address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'device_address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'device_address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'timeout)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'timeout)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'timeout)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'timeout)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'data) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<I2CWriteParameter>)))
  "Returns string type for a message object of type '<I2CWriteParameter>"
  "kortex_driver/I2CWriteParameter")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'I2CWriteParameter)))
  "Returns string type for a message object of type 'I2CWriteParameter"
  "kortex_driver/I2CWriteParameter")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<I2CWriteParameter>)))
  "Returns md5sum for a message object of type '<I2CWriteParameter>"
  "619677bad5497ce3141f386d2779fe02")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'I2CWriteParameter)))
  "Returns md5sum for a message object of type 'I2CWriteParameter"
  "619677bad5497ce3141f386d2779fe02")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<I2CWriteParameter>)))
  "Returns full string definition for message of type '<I2CWriteParameter>"
  (cl:format cl:nil "~%uint32 device~%uint32 device_address~%uint32 timeout~%I2CData data~%================================================================================~%MSG: kortex_driver/I2CData~%~%uint8[] data~%uint32 size~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'I2CWriteParameter)))
  "Returns full string definition for message of type 'I2CWriteParameter"
  (cl:format cl:nil "~%uint32 device~%uint32 device_address~%uint32 timeout~%I2CData data~%================================================================================~%MSG: kortex_driver/I2CData~%~%uint8[] data~%uint32 size~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <I2CWriteParameter>))
  (cl:+ 0
     4
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <I2CWriteParameter>))
  "Converts a ROS message object to a list"
  (cl:list 'I2CWriteParameter
    (cl:cons ':device (device msg))
    (cl:cons ':device_address (device_address msg))
    (cl:cons ':timeout (timeout msg))
    (cl:cons ':data (data msg))
))
