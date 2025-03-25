; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude DisableBridge-request.msg.html

(cl:defclass <DisableBridge-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:BridgeIdentifier
    :initform (cl:make-instance 'kortex_driver-msg:BridgeIdentifier)))
)

(cl:defclass DisableBridge-request (<DisableBridge-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DisableBridge-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DisableBridge-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<DisableBridge-request> is deprecated: use kortex_driver-srv:DisableBridge-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <DisableBridge-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DisableBridge-request>) ostream)
  "Serializes a message object of type '<DisableBridge-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DisableBridge-request>) istream)
  "Deserializes a message object of type '<DisableBridge-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DisableBridge-request>)))
  "Returns string type for a service object of type '<DisableBridge-request>"
  "kortex_driver/DisableBridgeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DisableBridge-request)))
  "Returns string type for a service object of type 'DisableBridge-request"
  "kortex_driver/DisableBridgeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DisableBridge-request>)))
  "Returns md5sum for a message object of type '<DisableBridge-request>"
  "5d03590e711a693b2b6ee914ec9cc8b3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DisableBridge-request)))
  "Returns md5sum for a message object of type 'DisableBridge-request"
  "5d03590e711a693b2b6ee914ec9cc8b3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DisableBridge-request>)))
  "Returns full string definition for message of type '<DisableBridge-request>"
  (cl:format cl:nil "BridgeIdentifier input~%~%================================================================================~%MSG: kortex_driver/BridgeIdentifier~%~%uint32 bridge_id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DisableBridge-request)))
  "Returns full string definition for message of type 'DisableBridge-request"
  (cl:format cl:nil "BridgeIdentifier input~%~%================================================================================~%MSG: kortex_driver/BridgeIdentifier~%~%uint32 bridge_id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DisableBridge-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DisableBridge-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DisableBridge-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude DisableBridge-response.msg.html

(cl:defclass <DisableBridge-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:BridgeResult
    :initform (cl:make-instance 'kortex_driver-msg:BridgeResult)))
)

(cl:defclass DisableBridge-response (<DisableBridge-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DisableBridge-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DisableBridge-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<DisableBridge-response> is deprecated: use kortex_driver-srv:DisableBridge-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <DisableBridge-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DisableBridge-response>) ostream)
  "Serializes a message object of type '<DisableBridge-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DisableBridge-response>) istream)
  "Deserializes a message object of type '<DisableBridge-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DisableBridge-response>)))
  "Returns string type for a service object of type '<DisableBridge-response>"
  "kortex_driver/DisableBridgeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DisableBridge-response)))
  "Returns string type for a service object of type 'DisableBridge-response"
  "kortex_driver/DisableBridgeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DisableBridge-response>)))
  "Returns md5sum for a message object of type '<DisableBridge-response>"
  "5d03590e711a693b2b6ee914ec9cc8b3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DisableBridge-response)))
  "Returns md5sum for a message object of type 'DisableBridge-response"
  "5d03590e711a693b2b6ee914ec9cc8b3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DisableBridge-response>)))
  "Returns full string definition for message of type '<DisableBridge-response>"
  (cl:format cl:nil "BridgeResult output~%~%================================================================================~%MSG: kortex_driver/BridgeResult~%~%BridgeIdentifier bridge_id~%uint32 status~%================================================================================~%MSG: kortex_driver/BridgeIdentifier~%~%uint32 bridge_id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DisableBridge-response)))
  "Returns full string definition for message of type 'DisableBridge-response"
  (cl:format cl:nil "BridgeResult output~%~%================================================================================~%MSG: kortex_driver/BridgeResult~%~%BridgeIdentifier bridge_id~%uint32 status~%================================================================================~%MSG: kortex_driver/BridgeIdentifier~%~%uint32 bridge_id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DisableBridge-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DisableBridge-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DisableBridge-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DisableBridge)))
  'DisableBridge-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DisableBridge)))
  'DisableBridge-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DisableBridge)))
  "Returns string type for a service object of type '<DisableBridge>"
  "kortex_driver/DisableBridge")