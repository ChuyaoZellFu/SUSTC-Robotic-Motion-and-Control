; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude OptionIdentifier.msg.html

(cl:defclass <OptionIdentifier> (roslisp-msg-protocol:ros-message)
  ((sensor
    :reader sensor
    :initarg :sensor
    :type cl:integer
    :initform 0)
   (option
    :reader option
    :initarg :option
    :type cl:integer
    :initform 0))
)

(cl:defclass OptionIdentifier (<OptionIdentifier>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OptionIdentifier>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OptionIdentifier)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<OptionIdentifier> is deprecated: use kortex_driver-msg:OptionIdentifier instead.")))

(cl:ensure-generic-function 'sensor-val :lambda-list '(m))
(cl:defmethod sensor-val ((m <OptionIdentifier>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:sensor-val is deprecated.  Use kortex_driver-msg:sensor instead.")
  (sensor m))

(cl:ensure-generic-function 'option-val :lambda-list '(m))
(cl:defmethod option-val ((m <OptionIdentifier>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:option-val is deprecated.  Use kortex_driver-msg:option instead.")
  (option m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OptionIdentifier>) ostream)
  "Serializes a message object of type '<OptionIdentifier>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sensor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sensor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sensor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sensor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'option)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'option)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'option)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'option)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OptionIdentifier>) istream)
  "Deserializes a message object of type '<OptionIdentifier>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sensor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sensor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sensor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sensor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'option)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'option)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'option)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'option)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OptionIdentifier>)))
  "Returns string type for a message object of type '<OptionIdentifier>"
  "kortex_driver/OptionIdentifier")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OptionIdentifier)))
  "Returns string type for a message object of type 'OptionIdentifier"
  "kortex_driver/OptionIdentifier")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OptionIdentifier>)))
  "Returns md5sum for a message object of type '<OptionIdentifier>"
  "23d53d4ecb95c9409f8e27d3bfff4aca")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OptionIdentifier)))
  "Returns md5sum for a message object of type 'OptionIdentifier"
  "23d53d4ecb95c9409f8e27d3bfff4aca")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OptionIdentifier>)))
  "Returns full string definition for message of type '<OptionIdentifier>"
  (cl:format cl:nil "~%uint32 sensor~%uint32 option~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OptionIdentifier)))
  "Returns full string definition for message of type 'OptionIdentifier"
  (cl:format cl:nil "~%uint32 sensor~%uint32 option~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OptionIdentifier>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OptionIdentifier>))
  "Converts a ROS message object to a list"
  (cl:list 'OptionIdentifier
    (cl:cons ':sensor (sensor msg))
    (cl:cons ':option (option msg))
))
