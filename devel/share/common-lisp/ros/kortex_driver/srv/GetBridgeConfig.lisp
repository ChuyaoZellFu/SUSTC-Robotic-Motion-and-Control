; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetBridgeConfig-request.msg.html

(cl:defclass <GetBridgeConfig-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:BridgeIdentifier
    :initform (cl:make-instance 'kortex_driver-msg:BridgeIdentifier)))
)

(cl:defclass GetBridgeConfig-request (<GetBridgeConfig-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetBridgeConfig-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetBridgeConfig-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetBridgeConfig-request> is deprecated: use kortex_driver-srv:GetBridgeConfig-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetBridgeConfig-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetBridgeConfig-request>) ostream)
  "Serializes a message object of type '<GetBridgeConfig-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetBridgeConfig-request>) istream)
  "Deserializes a message object of type '<GetBridgeConfig-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetBridgeConfig-request>)))
  "Returns string type for a service object of type '<GetBridgeConfig-request>"
  "kortex_driver/GetBridgeConfigRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetBridgeConfig-request)))
  "Returns string type for a service object of type 'GetBridgeConfig-request"
  "kortex_driver/GetBridgeConfigRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetBridgeConfig-request>)))
  "Returns md5sum for a message object of type '<GetBridgeConfig-request>"
  "9f7aa46e9ca64f4d1fa2fe2894e0afa6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetBridgeConfig-request)))
  "Returns md5sum for a message object of type 'GetBridgeConfig-request"
  "9f7aa46e9ca64f4d1fa2fe2894e0afa6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetBridgeConfig-request>)))
  "Returns full string definition for message of type '<GetBridgeConfig-request>"
  (cl:format cl:nil "BridgeIdentifier input~%~%================================================================================~%MSG: kortex_driver/BridgeIdentifier~%~%uint32 bridge_id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetBridgeConfig-request)))
  "Returns full string definition for message of type 'GetBridgeConfig-request"
  (cl:format cl:nil "BridgeIdentifier input~%~%================================================================================~%MSG: kortex_driver/BridgeIdentifier~%~%uint32 bridge_id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetBridgeConfig-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetBridgeConfig-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetBridgeConfig-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetBridgeConfig-response.msg.html

(cl:defclass <GetBridgeConfig-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:BridgeConfig
    :initform (cl:make-instance 'kortex_driver-msg:BridgeConfig)))
)

(cl:defclass GetBridgeConfig-response (<GetBridgeConfig-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetBridgeConfig-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetBridgeConfig-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetBridgeConfig-response> is deprecated: use kortex_driver-srv:GetBridgeConfig-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetBridgeConfig-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetBridgeConfig-response>) ostream)
  "Serializes a message object of type '<GetBridgeConfig-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetBridgeConfig-response>) istream)
  "Deserializes a message object of type '<GetBridgeConfig-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetBridgeConfig-response>)))
  "Returns string type for a service object of type '<GetBridgeConfig-response>"
  "kortex_driver/GetBridgeConfigResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetBridgeConfig-response)))
  "Returns string type for a service object of type 'GetBridgeConfig-response"
  "kortex_driver/GetBridgeConfigResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetBridgeConfig-response>)))
  "Returns md5sum for a message object of type '<GetBridgeConfig-response>"
  "9f7aa46e9ca64f4d1fa2fe2894e0afa6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetBridgeConfig-response)))
  "Returns md5sum for a message object of type 'GetBridgeConfig-response"
  "9f7aa46e9ca64f4d1fa2fe2894e0afa6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetBridgeConfig-response>)))
  "Returns full string definition for message of type '<GetBridgeConfig-response>"
  (cl:format cl:nil "BridgeConfig output~%~%================================================================================~%MSG: kortex_driver/BridgeConfig~%~%uint32 device_identifier~%uint32 bridgetype~%BridgePortConfig port_config~%BridgeIdentifier bridge_id~%================================================================================~%MSG: kortex_driver/BridgePortConfig~%~%uint32 target_port~%uint32 out_port~%================================================================================~%MSG: kortex_driver/BridgeIdentifier~%~%uint32 bridge_id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetBridgeConfig-response)))
  "Returns full string definition for message of type 'GetBridgeConfig-response"
  (cl:format cl:nil "BridgeConfig output~%~%================================================================================~%MSG: kortex_driver/BridgeConfig~%~%uint32 device_identifier~%uint32 bridgetype~%BridgePortConfig port_config~%BridgeIdentifier bridge_id~%================================================================================~%MSG: kortex_driver/BridgePortConfig~%~%uint32 target_port~%uint32 out_port~%================================================================================~%MSG: kortex_driver/BridgeIdentifier~%~%uint32 bridge_id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetBridgeConfig-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetBridgeConfig-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetBridgeConfig-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetBridgeConfig)))
  'GetBridgeConfig-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetBridgeConfig)))
  'GetBridgeConfig-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetBridgeConfig)))
  "Returns string type for a service object of type '<GetBridgeConfig>"
  "kortex_driver/GetBridgeConfig")