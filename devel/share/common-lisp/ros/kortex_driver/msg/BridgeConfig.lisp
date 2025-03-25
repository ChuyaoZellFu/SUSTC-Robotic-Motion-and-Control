; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude BridgeConfig.msg.html

(cl:defclass <BridgeConfig> (roslisp-msg-protocol:ros-message)
  ((device_identifier
    :reader device_identifier
    :initarg :device_identifier
    :type cl:integer
    :initform 0)
   (bridgetype
    :reader bridgetype
    :initarg :bridgetype
    :type cl:integer
    :initform 0)
   (port_config
    :reader port_config
    :initarg :port_config
    :type kortex_driver-msg:BridgePortConfig
    :initform (cl:make-instance 'kortex_driver-msg:BridgePortConfig))
   (bridge_id
    :reader bridge_id
    :initarg :bridge_id
    :type kortex_driver-msg:BridgeIdentifier
    :initform (cl:make-instance 'kortex_driver-msg:BridgeIdentifier)))
)

(cl:defclass BridgeConfig (<BridgeConfig>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BridgeConfig>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BridgeConfig)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<BridgeConfig> is deprecated: use kortex_driver-msg:BridgeConfig instead.")))

(cl:ensure-generic-function 'device_identifier-val :lambda-list '(m))
(cl:defmethod device_identifier-val ((m <BridgeConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:device_identifier-val is deprecated.  Use kortex_driver-msg:device_identifier instead.")
  (device_identifier m))

(cl:ensure-generic-function 'bridgetype-val :lambda-list '(m))
(cl:defmethod bridgetype-val ((m <BridgeConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:bridgetype-val is deprecated.  Use kortex_driver-msg:bridgetype instead.")
  (bridgetype m))

(cl:ensure-generic-function 'port_config-val :lambda-list '(m))
(cl:defmethod port_config-val ((m <BridgeConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:port_config-val is deprecated.  Use kortex_driver-msg:port_config instead.")
  (port_config m))

(cl:ensure-generic-function 'bridge_id-val :lambda-list '(m))
(cl:defmethod bridge_id-val ((m <BridgeConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:bridge_id-val is deprecated.  Use kortex_driver-msg:bridge_id instead.")
  (bridge_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BridgeConfig>) ostream)
  "Serializes a message object of type '<BridgeConfig>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'device_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'device_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'device_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'device_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bridgetype)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'bridgetype)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'bridgetype)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'bridgetype)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'port_config) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'bridge_id) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BridgeConfig>) istream)
  "Deserializes a message object of type '<BridgeConfig>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'device_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'device_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'device_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'device_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bridgetype)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'bridgetype)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'bridgetype)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'bridgetype)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'port_config) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'bridge_id) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BridgeConfig>)))
  "Returns string type for a message object of type '<BridgeConfig>"
  "kortex_driver/BridgeConfig")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BridgeConfig)))
  "Returns string type for a message object of type 'BridgeConfig"
  "kortex_driver/BridgeConfig")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BridgeConfig>)))
  "Returns md5sum for a message object of type '<BridgeConfig>"
  "967c7a78caf96ff069310456674faf6a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BridgeConfig)))
  "Returns md5sum for a message object of type 'BridgeConfig"
  "967c7a78caf96ff069310456674faf6a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BridgeConfig>)))
  "Returns full string definition for message of type '<BridgeConfig>"
  (cl:format cl:nil "~%uint32 device_identifier~%uint32 bridgetype~%BridgePortConfig port_config~%BridgeIdentifier bridge_id~%================================================================================~%MSG: kortex_driver/BridgePortConfig~%~%uint32 target_port~%uint32 out_port~%================================================================================~%MSG: kortex_driver/BridgeIdentifier~%~%uint32 bridge_id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BridgeConfig)))
  "Returns full string definition for message of type 'BridgeConfig"
  (cl:format cl:nil "~%uint32 device_identifier~%uint32 bridgetype~%BridgePortConfig port_config~%BridgeIdentifier bridge_id~%================================================================================~%MSG: kortex_driver/BridgePortConfig~%~%uint32 target_port~%uint32 out_port~%================================================================================~%MSG: kortex_driver/BridgeIdentifier~%~%uint32 bridge_id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BridgeConfig>))
  (cl:+ 0
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'port_config))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'bridge_id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BridgeConfig>))
  "Converts a ROS message object to a list"
  (cl:list 'BridgeConfig
    (cl:cons ':device_identifier (device_identifier msg))
    (cl:cons ':bridgetype (bridgetype msg))
    (cl:cons ':port_config (port_config msg))
    (cl:cons ':bridge_id (bridge_id msg))
))
