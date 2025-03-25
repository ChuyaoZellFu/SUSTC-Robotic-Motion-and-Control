; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetBridgeList-request.msg.html

(cl:defclass <GetBridgeList-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetBridgeList-request (<GetBridgeList-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetBridgeList-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetBridgeList-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetBridgeList-request> is deprecated: use kortex_driver-srv:GetBridgeList-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetBridgeList-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetBridgeList-request>) ostream)
  "Serializes a message object of type '<GetBridgeList-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetBridgeList-request>) istream)
  "Deserializes a message object of type '<GetBridgeList-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetBridgeList-request>)))
  "Returns string type for a service object of type '<GetBridgeList-request>"
  "kortex_driver/GetBridgeListRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetBridgeList-request)))
  "Returns string type for a service object of type 'GetBridgeList-request"
  "kortex_driver/GetBridgeListRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetBridgeList-request>)))
  "Returns md5sum for a message object of type '<GetBridgeList-request>"
  "866f5d10e4d60286569bdfe7c65c5078")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetBridgeList-request)))
  "Returns md5sum for a message object of type 'GetBridgeList-request"
  "866f5d10e4d60286569bdfe7c65c5078")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetBridgeList-request>)))
  "Returns full string definition for message of type '<GetBridgeList-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetBridgeList-request)))
  "Returns full string definition for message of type 'GetBridgeList-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetBridgeList-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetBridgeList-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetBridgeList-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetBridgeList-response.msg.html

(cl:defclass <GetBridgeList-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:BridgeList
    :initform (cl:make-instance 'kortex_driver-msg:BridgeList)))
)

(cl:defclass GetBridgeList-response (<GetBridgeList-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetBridgeList-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetBridgeList-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetBridgeList-response> is deprecated: use kortex_driver-srv:GetBridgeList-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetBridgeList-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetBridgeList-response>) ostream)
  "Serializes a message object of type '<GetBridgeList-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetBridgeList-response>) istream)
  "Deserializes a message object of type '<GetBridgeList-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetBridgeList-response>)))
  "Returns string type for a service object of type '<GetBridgeList-response>"
  "kortex_driver/GetBridgeListResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetBridgeList-response)))
  "Returns string type for a service object of type 'GetBridgeList-response"
  "kortex_driver/GetBridgeListResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetBridgeList-response>)))
  "Returns md5sum for a message object of type '<GetBridgeList-response>"
  "866f5d10e4d60286569bdfe7c65c5078")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetBridgeList-response)))
  "Returns md5sum for a message object of type 'GetBridgeList-response"
  "866f5d10e4d60286569bdfe7c65c5078")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetBridgeList-response>)))
  "Returns full string definition for message of type '<GetBridgeList-response>"
  (cl:format cl:nil "BridgeList output~%~%================================================================================~%MSG: kortex_driver/BridgeList~%~%BridgeConfig[] bridgeConfig~%================================================================================~%MSG: kortex_driver/BridgeConfig~%~%uint32 device_identifier~%uint32 bridgetype~%BridgePortConfig port_config~%BridgeIdentifier bridge_id~%================================================================================~%MSG: kortex_driver/BridgePortConfig~%~%uint32 target_port~%uint32 out_port~%================================================================================~%MSG: kortex_driver/BridgeIdentifier~%~%uint32 bridge_id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetBridgeList-response)))
  "Returns full string definition for message of type 'GetBridgeList-response"
  (cl:format cl:nil "BridgeList output~%~%================================================================================~%MSG: kortex_driver/BridgeList~%~%BridgeConfig[] bridgeConfig~%================================================================================~%MSG: kortex_driver/BridgeConfig~%~%uint32 device_identifier~%uint32 bridgetype~%BridgePortConfig port_config~%BridgeIdentifier bridge_id~%================================================================================~%MSG: kortex_driver/BridgePortConfig~%~%uint32 target_port~%uint32 out_port~%================================================================================~%MSG: kortex_driver/BridgeIdentifier~%~%uint32 bridge_id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetBridgeList-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetBridgeList-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetBridgeList-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetBridgeList)))
  'GetBridgeList-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetBridgeList)))
  'GetBridgeList-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetBridgeList)))
  "Returns string type for a service object of type '<GetBridgeList>"
  "kortex_driver/GetBridgeList")