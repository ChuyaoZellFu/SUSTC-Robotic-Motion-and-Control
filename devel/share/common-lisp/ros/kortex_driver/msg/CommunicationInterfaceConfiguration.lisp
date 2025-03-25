; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude CommunicationInterfaceConfiguration.msg.html

(cl:defclass <CommunicationInterfaceConfiguration> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:integer
    :initform 0)
   (enable
    :reader enable
    :initarg :enable
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass CommunicationInterfaceConfiguration (<CommunicationInterfaceConfiguration>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CommunicationInterfaceConfiguration>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CommunicationInterfaceConfiguration)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<CommunicationInterfaceConfiguration> is deprecated: use kortex_driver-msg:CommunicationInterfaceConfiguration instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <CommunicationInterfaceConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:type-val is deprecated.  Use kortex_driver-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'enable-val :lambda-list '(m))
(cl:defmethod enable-val ((m <CommunicationInterfaceConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:enable-val is deprecated.  Use kortex_driver-msg:enable instead.")
  (enable m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CommunicationInterfaceConfiguration>) ostream)
  "Serializes a message object of type '<CommunicationInterfaceConfiguration>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enable) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CommunicationInterfaceConfiguration>) istream)
  "Deserializes a message object of type '<CommunicationInterfaceConfiguration>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'enable) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CommunicationInterfaceConfiguration>)))
  "Returns string type for a message object of type '<CommunicationInterfaceConfiguration>"
  "kortex_driver/CommunicationInterfaceConfiguration")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CommunicationInterfaceConfiguration)))
  "Returns string type for a message object of type 'CommunicationInterfaceConfiguration"
  "kortex_driver/CommunicationInterfaceConfiguration")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CommunicationInterfaceConfiguration>)))
  "Returns md5sum for a message object of type '<CommunicationInterfaceConfiguration>"
  "5a1523f2cb0646f5be12fd2c14b0a2a7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CommunicationInterfaceConfiguration)))
  "Returns md5sum for a message object of type 'CommunicationInterfaceConfiguration"
  "5a1523f2cb0646f5be12fd2c14b0a2a7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CommunicationInterfaceConfiguration>)))
  "Returns full string definition for message of type '<CommunicationInterfaceConfiguration>"
  (cl:format cl:nil "~%uint32 type~%bool enable~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CommunicationInterfaceConfiguration)))
  "Returns full string definition for message of type 'CommunicationInterfaceConfiguration"
  (cl:format cl:nil "~%uint32 type~%bool enable~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CommunicationInterfaceConfiguration>))
  (cl:+ 0
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CommunicationInterfaceConfiguration>))
  "Converts a ROS message object to a list"
  (cl:list 'CommunicationInterfaceConfiguration
    (cl:cons ':type (type msg))
    (cl:cons ':enable (enable msg))
))
