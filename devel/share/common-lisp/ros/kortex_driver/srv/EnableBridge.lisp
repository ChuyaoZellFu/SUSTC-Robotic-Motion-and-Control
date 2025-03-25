; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude EnableBridge-request.msg.html

(cl:defclass <EnableBridge-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:BridgeConfig
    :initform (cl:make-instance 'kortex_driver-msg:BridgeConfig)))
)

(cl:defclass EnableBridge-request (<EnableBridge-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EnableBridge-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EnableBridge-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<EnableBridge-request> is deprecated: use kortex_driver-srv:EnableBridge-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <EnableBridge-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EnableBridge-request>) ostream)
  "Serializes a message object of type '<EnableBridge-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EnableBridge-request>) istream)
  "Deserializes a message object of type '<EnableBridge-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EnableBridge-request>)))
  "Returns string type for a service object of type '<EnableBridge-request>"
  "kortex_driver/EnableBridgeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EnableBridge-request)))
  "Returns string type for a service object of type 'EnableBridge-request"
  "kortex_driver/EnableBridgeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EnableBridge-request>)))
  "Returns md5sum for a message object of type '<EnableBridge-request>"
  "be9e21afaa7e92c6bf00b8eed13a695e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EnableBridge-request)))
  "Returns md5sum for a message object of type 'EnableBridge-request"
  "be9e21afaa7e92c6bf00b8eed13a695e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EnableBridge-request>)))
  "Returns full string definition for message of type '<EnableBridge-request>"
  (cl:format cl:nil "BridgeConfig input~%~%================================================================================~%MSG: kortex_driver/BridgeConfig~%~%uint32 device_identifier~%uint32 bridgetype~%BridgePortConfig port_config~%BridgeIdentifier bridge_id~%================================================================================~%MSG: kortex_driver/BridgePortConfig~%~%uint32 target_port~%uint32 out_port~%================================================================================~%MSG: kortex_driver/BridgeIdentifier~%~%uint32 bridge_id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EnableBridge-request)))
  "Returns full string definition for message of type 'EnableBridge-request"
  (cl:format cl:nil "BridgeConfig input~%~%================================================================================~%MSG: kortex_driver/BridgeConfig~%~%uint32 device_identifier~%uint32 bridgetype~%BridgePortConfig port_config~%BridgeIdentifier bridge_id~%================================================================================~%MSG: kortex_driver/BridgePortConfig~%~%uint32 target_port~%uint32 out_port~%================================================================================~%MSG: kortex_driver/BridgeIdentifier~%~%uint32 bridge_id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EnableBridge-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EnableBridge-request>))
  "Converts a ROS message object to a list"
  (cl:list 'EnableBridge-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude EnableBridge-response.msg.html

(cl:defclass <EnableBridge-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:BridgeResult
    :initform (cl:make-instance 'kortex_driver-msg:BridgeResult)))
)

(cl:defclass EnableBridge-response (<EnableBridge-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EnableBridge-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EnableBridge-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<EnableBridge-response> is deprecated: use kortex_driver-srv:EnableBridge-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <EnableBridge-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EnableBridge-response>) ostream)
  "Serializes a message object of type '<EnableBridge-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EnableBridge-response>) istream)
  "Deserializes a message object of type '<EnableBridge-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EnableBridge-response>)))
  "Returns string type for a service object of type '<EnableBridge-response>"
  "kortex_driver/EnableBridgeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EnableBridge-response)))
  "Returns string type for a service object of type 'EnableBridge-response"
  "kortex_driver/EnableBridgeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EnableBridge-response>)))
  "Returns md5sum for a message object of type '<EnableBridge-response>"
  "be9e21afaa7e92c6bf00b8eed13a695e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EnableBridge-response)))
  "Returns md5sum for a message object of type 'EnableBridge-response"
  "be9e21afaa7e92c6bf00b8eed13a695e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EnableBridge-response>)))
  "Returns full string definition for message of type '<EnableBridge-response>"
  (cl:format cl:nil "BridgeResult output~%~%================================================================================~%MSG: kortex_driver/BridgeResult~%~%BridgeIdentifier bridge_id~%uint32 status~%================================================================================~%MSG: kortex_driver/BridgeIdentifier~%~%uint32 bridge_id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EnableBridge-response)))
  "Returns full string definition for message of type 'EnableBridge-response"
  (cl:format cl:nil "BridgeResult output~%~%================================================================================~%MSG: kortex_driver/BridgeResult~%~%BridgeIdentifier bridge_id~%uint32 status~%================================================================================~%MSG: kortex_driver/BridgeIdentifier~%~%uint32 bridge_id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EnableBridge-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EnableBridge-response>))
  "Converts a ROS message object to a list"
  (cl:list 'EnableBridge-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'EnableBridge)))
  'EnableBridge-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'EnableBridge)))
  'EnableBridge-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EnableBridge)))
  "Returns string type for a service object of type '<EnableBridge>"
  "kortex_driver/EnableBridge")