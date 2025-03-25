; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude BridgeIdentifier.msg.html

(cl:defclass <BridgeIdentifier> (roslisp-msg-protocol:ros-message)
  ((bridge_id
    :reader bridge_id
    :initarg :bridge_id
    :type cl:integer
    :initform 0))
)

(cl:defclass BridgeIdentifier (<BridgeIdentifier>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BridgeIdentifier>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BridgeIdentifier)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<BridgeIdentifier> is deprecated: use kortex_driver-msg:BridgeIdentifier instead.")))

(cl:ensure-generic-function 'bridge_id-val :lambda-list '(m))
(cl:defmethod bridge_id-val ((m <BridgeIdentifier>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:bridge_id-val is deprecated.  Use kortex_driver-msg:bridge_id instead.")
  (bridge_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BridgeIdentifier>) ostream)
  "Serializes a message object of type '<BridgeIdentifier>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bridge_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'bridge_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'bridge_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'bridge_id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BridgeIdentifier>) istream)
  "Deserializes a message object of type '<BridgeIdentifier>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bridge_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'bridge_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'bridge_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'bridge_id)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BridgeIdentifier>)))
  "Returns string type for a message object of type '<BridgeIdentifier>"
  "kortex_driver/BridgeIdentifier")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BridgeIdentifier)))
  "Returns string type for a message object of type 'BridgeIdentifier"
  "kortex_driver/BridgeIdentifier")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BridgeIdentifier>)))
  "Returns md5sum for a message object of type '<BridgeIdentifier>"
  "24fa834626ec2b7a0c103b00131d6015")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BridgeIdentifier)))
  "Returns md5sum for a message object of type 'BridgeIdentifier"
  "24fa834626ec2b7a0c103b00131d6015")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BridgeIdentifier>)))
  "Returns full string definition for message of type '<BridgeIdentifier>"
  (cl:format cl:nil "~%uint32 bridge_id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BridgeIdentifier)))
  "Returns full string definition for message of type 'BridgeIdentifier"
  (cl:format cl:nil "~%uint32 bridge_id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BridgeIdentifier>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BridgeIdentifier>))
  "Converts a ROS message object to a list"
  (cl:list 'BridgeIdentifier
    (cl:cons ':bridge_id (bridge_id msg))
))
