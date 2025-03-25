; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude UARTDeviceIdentification.msg.html

(cl:defclass <UARTDeviceIdentification> (roslisp-msg-protocol:ros-message)
  ((port_id
    :reader port_id
    :initarg :port_id
    :type cl:integer
    :initform 0))
)

(cl:defclass UARTDeviceIdentification (<UARTDeviceIdentification>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UARTDeviceIdentification>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UARTDeviceIdentification)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<UARTDeviceIdentification> is deprecated: use kortex_driver-msg:UARTDeviceIdentification instead.")))

(cl:ensure-generic-function 'port_id-val :lambda-list '(m))
(cl:defmethod port_id-val ((m <UARTDeviceIdentification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:port_id-val is deprecated.  Use kortex_driver-msg:port_id instead.")
  (port_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UARTDeviceIdentification>) ostream)
  "Serializes a message object of type '<UARTDeviceIdentification>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'port_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'port_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'port_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'port_id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UARTDeviceIdentification>) istream)
  "Deserializes a message object of type '<UARTDeviceIdentification>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'port_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'port_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'port_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'port_id)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UARTDeviceIdentification>)))
  "Returns string type for a message object of type '<UARTDeviceIdentification>"
  "kortex_driver/UARTDeviceIdentification")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UARTDeviceIdentification)))
  "Returns string type for a message object of type 'UARTDeviceIdentification"
  "kortex_driver/UARTDeviceIdentification")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UARTDeviceIdentification>)))
  "Returns md5sum for a message object of type '<UARTDeviceIdentification>"
  "c688e9b638ff5d4cab989955c0b36eb8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UARTDeviceIdentification)))
  "Returns md5sum for a message object of type 'UARTDeviceIdentification"
  "c688e9b638ff5d4cab989955c0b36eb8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UARTDeviceIdentification>)))
  "Returns full string definition for message of type '<UARTDeviceIdentification>"
  (cl:format cl:nil "~%uint32 port_id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UARTDeviceIdentification)))
  "Returns full string definition for message of type 'UARTDeviceIdentification"
  (cl:format cl:nil "~%uint32 port_id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UARTDeviceIdentification>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UARTDeviceIdentification>))
  "Converts a ROS message object to a list"
  (cl:list 'UARTDeviceIdentification
    (cl:cons ':port_id (port_id msg))
))
