; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude EthernetDeviceIdentification.msg.html

(cl:defclass <EthernetDeviceIdentification> (roslisp-msg-protocol:ros-message)
  ((device
    :reader device
    :initarg :device
    :type cl:integer
    :initform 0))
)

(cl:defclass EthernetDeviceIdentification (<EthernetDeviceIdentification>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EthernetDeviceIdentification>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EthernetDeviceIdentification)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<EthernetDeviceIdentification> is deprecated: use kortex_driver-msg:EthernetDeviceIdentification instead.")))

(cl:ensure-generic-function 'device-val :lambda-list '(m))
(cl:defmethod device-val ((m <EthernetDeviceIdentification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:device-val is deprecated.  Use kortex_driver-msg:device instead.")
  (device m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EthernetDeviceIdentification>) ostream)
  "Serializes a message object of type '<EthernetDeviceIdentification>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'device)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'device)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'device)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'device)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EthernetDeviceIdentification>) istream)
  "Deserializes a message object of type '<EthernetDeviceIdentification>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'device)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'device)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'device)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'device)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EthernetDeviceIdentification>)))
  "Returns string type for a message object of type '<EthernetDeviceIdentification>"
  "kortex_driver/EthernetDeviceIdentification")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EthernetDeviceIdentification)))
  "Returns string type for a message object of type 'EthernetDeviceIdentification"
  "kortex_driver/EthernetDeviceIdentification")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EthernetDeviceIdentification>)))
  "Returns md5sum for a message object of type '<EthernetDeviceIdentification>"
  "e97054e4ab2eb3665270f077c46482df")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EthernetDeviceIdentification)))
  "Returns md5sum for a message object of type 'EthernetDeviceIdentification"
  "e97054e4ab2eb3665270f077c46482df")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EthernetDeviceIdentification>)))
  "Returns full string definition for message of type '<EthernetDeviceIdentification>"
  (cl:format cl:nil "~%uint32 device~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EthernetDeviceIdentification)))
  "Returns full string definition for message of type 'EthernetDeviceIdentification"
  (cl:format cl:nil "~%uint32 device~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EthernetDeviceIdentification>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EthernetDeviceIdentification>))
  "Converts a ROS message object to a list"
  (cl:list 'EthernetDeviceIdentification
    (cl:cons ':device (device msg))
))
