; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude GpioCommand.msg.html

(cl:defclass <GpioCommand> (roslisp-msg-protocol:ros-message)
  ((port_identifier
    :reader port_identifier
    :initarg :port_identifier
    :type cl:integer
    :initform 0)
   (pin_identifier
    :reader pin_identifier
    :initarg :pin_identifier
    :type cl:integer
    :initform 0)
   (action
    :reader action
    :initarg :action
    :type cl:integer
    :initform 0)
   (period
    :reader period
    :initarg :period
    :type cl:integer
    :initform 0))
)

(cl:defclass GpioCommand (<GpioCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GpioCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GpioCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<GpioCommand> is deprecated: use kortex_driver-msg:GpioCommand instead.")))

(cl:ensure-generic-function 'port_identifier-val :lambda-list '(m))
(cl:defmethod port_identifier-val ((m <GpioCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:port_identifier-val is deprecated.  Use kortex_driver-msg:port_identifier instead.")
  (port_identifier m))

(cl:ensure-generic-function 'pin_identifier-val :lambda-list '(m))
(cl:defmethod pin_identifier-val ((m <GpioCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:pin_identifier-val is deprecated.  Use kortex_driver-msg:pin_identifier instead.")
  (pin_identifier m))

(cl:ensure-generic-function 'action-val :lambda-list '(m))
(cl:defmethod action-val ((m <GpioCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:action-val is deprecated.  Use kortex_driver-msg:action instead.")
  (action m))

(cl:ensure-generic-function 'period-val :lambda-list '(m))
(cl:defmethod period-val ((m <GpioCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:period-val is deprecated.  Use kortex_driver-msg:period instead.")
  (period m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GpioCommand>) ostream)
  "Serializes a message object of type '<GpioCommand>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'port_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'port_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'port_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'port_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pin_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'pin_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'pin_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'pin_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'action)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'action)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'action)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'action)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'period)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'period)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'period)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'period)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GpioCommand>) istream)
  "Deserializes a message object of type '<GpioCommand>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'port_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'port_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'port_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'port_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pin_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'pin_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'pin_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'pin_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'action)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'action)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'action)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'action)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'period)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'period)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'period)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'period)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GpioCommand>)))
  "Returns string type for a message object of type '<GpioCommand>"
  "kortex_driver/GpioCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GpioCommand)))
  "Returns string type for a message object of type 'GpioCommand"
  "kortex_driver/GpioCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GpioCommand>)))
  "Returns md5sum for a message object of type '<GpioCommand>"
  "407fd312655ca86f1a6125a8f767d374")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GpioCommand)))
  "Returns md5sum for a message object of type 'GpioCommand"
  "407fd312655ca86f1a6125a8f767d374")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GpioCommand>)))
  "Returns full string definition for message of type '<GpioCommand>"
  (cl:format cl:nil "~%uint32 port_identifier~%uint32 pin_identifier~%uint32 action~%uint32 period~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GpioCommand)))
  "Returns full string definition for message of type 'GpioCommand"
  (cl:format cl:nil "~%uint32 port_identifier~%uint32 pin_identifier~%uint32 action~%uint32 period~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GpioCommand>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GpioCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'GpioCommand
    (cl:cons ':port_identifier (port_identifier msg))
    (cl:cons ':pin_identifier (pin_identifier msg))
    (cl:cons ':action (action msg))
    (cl:cons ':period (period msg))
))
