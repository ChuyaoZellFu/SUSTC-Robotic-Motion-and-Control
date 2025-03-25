; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude BridgePortConfig.msg.html

(cl:defclass <BridgePortConfig> (roslisp-msg-protocol:ros-message)
  ((target_port
    :reader target_port
    :initarg :target_port
    :type cl:integer
    :initform 0)
   (out_port
    :reader out_port
    :initarg :out_port
    :type cl:integer
    :initform 0))
)

(cl:defclass BridgePortConfig (<BridgePortConfig>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BridgePortConfig>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BridgePortConfig)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<BridgePortConfig> is deprecated: use kortex_driver-msg:BridgePortConfig instead.")))

(cl:ensure-generic-function 'target_port-val :lambda-list '(m))
(cl:defmethod target_port-val ((m <BridgePortConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:target_port-val is deprecated.  Use kortex_driver-msg:target_port instead.")
  (target_port m))

(cl:ensure-generic-function 'out_port-val :lambda-list '(m))
(cl:defmethod out_port-val ((m <BridgePortConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:out_port-val is deprecated.  Use kortex_driver-msg:out_port instead.")
  (out_port m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BridgePortConfig>) ostream)
  "Serializes a message object of type '<BridgePortConfig>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'target_port)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'target_port)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'target_port)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'target_port)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'out_port)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'out_port)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'out_port)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'out_port)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BridgePortConfig>) istream)
  "Deserializes a message object of type '<BridgePortConfig>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'target_port)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'target_port)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'target_port)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'target_port)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'out_port)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'out_port)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'out_port)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'out_port)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BridgePortConfig>)))
  "Returns string type for a message object of type '<BridgePortConfig>"
  "kortex_driver/BridgePortConfig")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BridgePortConfig)))
  "Returns string type for a message object of type 'BridgePortConfig"
  "kortex_driver/BridgePortConfig")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BridgePortConfig>)))
  "Returns md5sum for a message object of type '<BridgePortConfig>"
  "c2591c5a923ec65c5c9c9b1eb43de273")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BridgePortConfig)))
  "Returns md5sum for a message object of type 'BridgePortConfig"
  "c2591c5a923ec65c5c9c9b1eb43de273")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BridgePortConfig>)))
  "Returns full string definition for message of type '<BridgePortConfig>"
  (cl:format cl:nil "~%uint32 target_port~%uint32 out_port~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BridgePortConfig)))
  "Returns full string definition for message of type 'BridgePortConfig"
  (cl:format cl:nil "~%uint32 target_port~%uint32 out_port~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BridgePortConfig>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BridgePortConfig>))
  "Converts a ROS message object to a list"
  (cl:list 'BridgePortConfig
    (cl:cons ':target_port (target_port msg))
    (cl:cons ':out_port (out_port msg))
))
