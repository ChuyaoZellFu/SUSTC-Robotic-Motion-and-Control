; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude BridgeResult.msg.html

(cl:defclass <BridgeResult> (roslisp-msg-protocol:ros-message)
  ((bridge_id
    :reader bridge_id
    :initarg :bridge_id
    :type kortex_driver-msg:BridgeIdentifier
    :initform (cl:make-instance 'kortex_driver-msg:BridgeIdentifier))
   (status
    :reader status
    :initarg :status
    :type cl:integer
    :initform 0))
)

(cl:defclass BridgeResult (<BridgeResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BridgeResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BridgeResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<BridgeResult> is deprecated: use kortex_driver-msg:BridgeResult instead.")))

(cl:ensure-generic-function 'bridge_id-val :lambda-list '(m))
(cl:defmethod bridge_id-val ((m <BridgeResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:bridge_id-val is deprecated.  Use kortex_driver-msg:bridge_id instead.")
  (bridge_id m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <BridgeResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:status-val is deprecated.  Use kortex_driver-msg:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BridgeResult>) ostream)
  "Serializes a message object of type '<BridgeResult>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'bridge_id) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'status)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'status)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'status)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BridgeResult>) istream)
  "Deserializes a message object of type '<BridgeResult>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'bridge_id) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'status)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'status)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'status)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BridgeResult>)))
  "Returns string type for a message object of type '<BridgeResult>"
  "kortex_driver/BridgeResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BridgeResult)))
  "Returns string type for a message object of type 'BridgeResult"
  "kortex_driver/BridgeResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BridgeResult>)))
  "Returns md5sum for a message object of type '<BridgeResult>"
  "c86e99efcef2f596ae80430e65714cbc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BridgeResult)))
  "Returns md5sum for a message object of type 'BridgeResult"
  "c86e99efcef2f596ae80430e65714cbc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BridgeResult>)))
  "Returns full string definition for message of type '<BridgeResult>"
  (cl:format cl:nil "~%BridgeIdentifier bridge_id~%uint32 status~%================================================================================~%MSG: kortex_driver/BridgeIdentifier~%~%uint32 bridge_id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BridgeResult)))
  "Returns full string definition for message of type 'BridgeResult"
  (cl:format cl:nil "~%BridgeIdentifier bridge_id~%uint32 status~%================================================================================~%MSG: kortex_driver/BridgeIdentifier~%~%uint32 bridge_id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BridgeResult>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'bridge_id))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BridgeResult>))
  "Converts a ROS message object to a list"
  (cl:list 'BridgeResult
    (cl:cons ':bridge_id (bridge_id msg))
    (cl:cons ':status (status msg))
))
