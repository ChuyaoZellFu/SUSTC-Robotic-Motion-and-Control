; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude SensorIdentifier.msg.html

(cl:defclass <SensorIdentifier> (roslisp-msg-protocol:ros-message)
  ((sensor
    :reader sensor
    :initarg :sensor
    :type cl:integer
    :initform 0))
)

(cl:defclass SensorIdentifier (<SensorIdentifier>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SensorIdentifier>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SensorIdentifier)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<SensorIdentifier> is deprecated: use kortex_driver-msg:SensorIdentifier instead.")))

(cl:ensure-generic-function 'sensor-val :lambda-list '(m))
(cl:defmethod sensor-val ((m <SensorIdentifier>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:sensor-val is deprecated.  Use kortex_driver-msg:sensor instead.")
  (sensor m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SensorIdentifier>) ostream)
  "Serializes a message object of type '<SensorIdentifier>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sensor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sensor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sensor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sensor)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SensorIdentifier>) istream)
  "Deserializes a message object of type '<SensorIdentifier>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sensor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sensor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sensor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sensor)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SensorIdentifier>)))
  "Returns string type for a message object of type '<SensorIdentifier>"
  "kortex_driver/SensorIdentifier")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SensorIdentifier)))
  "Returns string type for a message object of type 'SensorIdentifier"
  "kortex_driver/SensorIdentifier")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SensorIdentifier>)))
  "Returns md5sum for a message object of type '<SensorIdentifier>"
  "b9fe73419757e1a59473fcd91b011ce9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SensorIdentifier)))
  "Returns md5sum for a message object of type 'SensorIdentifier"
  "b9fe73419757e1a59473fcd91b011ce9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SensorIdentifier>)))
  "Returns full string definition for message of type '<SensorIdentifier>"
  (cl:format cl:nil "~%uint32 sensor~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SensorIdentifier)))
  "Returns full string definition for message of type 'SensorIdentifier"
  (cl:format cl:nil "~%uint32 sensor~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SensorIdentifier>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SensorIdentifier>))
  "Converts a ROS message object to a list"
  (cl:list 'SensorIdentifier
    (cl:cons ':sensor (sensor msg))
))
