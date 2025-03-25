; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude ActuatorConfig_ClearFaults-request.msg.html

(cl:defclass <ActuatorConfig_ClearFaults-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass ActuatorConfig_ClearFaults-request (<ActuatorConfig_ClearFaults-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ActuatorConfig_ClearFaults-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ActuatorConfig_ClearFaults-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ActuatorConfig_ClearFaults-request> is deprecated: use kortex_driver-srv:ActuatorConfig_ClearFaults-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <ActuatorConfig_ClearFaults-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ActuatorConfig_ClearFaults-request>) ostream)
  "Serializes a message object of type '<ActuatorConfig_ClearFaults-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ActuatorConfig_ClearFaults-request>) istream)
  "Deserializes a message object of type '<ActuatorConfig_ClearFaults-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ActuatorConfig_ClearFaults-request>)))
  "Returns string type for a service object of type '<ActuatorConfig_ClearFaults-request>"
  "kortex_driver/ActuatorConfig_ClearFaultsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ActuatorConfig_ClearFaults-request)))
  "Returns string type for a service object of type 'ActuatorConfig_ClearFaults-request"
  "kortex_driver/ActuatorConfig_ClearFaultsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ActuatorConfig_ClearFaults-request>)))
  "Returns md5sum for a message object of type '<ActuatorConfig_ClearFaults-request>"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ActuatorConfig_ClearFaults-request)))
  "Returns md5sum for a message object of type 'ActuatorConfig_ClearFaults-request"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ActuatorConfig_ClearFaults-request>)))
  "Returns full string definition for message of type '<ActuatorConfig_ClearFaults-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ActuatorConfig_ClearFaults-request)))
  "Returns full string definition for message of type 'ActuatorConfig_ClearFaults-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ActuatorConfig_ClearFaults-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ActuatorConfig_ClearFaults-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ActuatorConfig_ClearFaults-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude ActuatorConfig_ClearFaults-response.msg.html

(cl:defclass <ActuatorConfig_ClearFaults-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass ActuatorConfig_ClearFaults-response (<ActuatorConfig_ClearFaults-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ActuatorConfig_ClearFaults-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ActuatorConfig_ClearFaults-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ActuatorConfig_ClearFaults-response> is deprecated: use kortex_driver-srv:ActuatorConfig_ClearFaults-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <ActuatorConfig_ClearFaults-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ActuatorConfig_ClearFaults-response>) ostream)
  "Serializes a message object of type '<ActuatorConfig_ClearFaults-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ActuatorConfig_ClearFaults-response>) istream)
  "Deserializes a message object of type '<ActuatorConfig_ClearFaults-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ActuatorConfig_ClearFaults-response>)))
  "Returns string type for a service object of type '<ActuatorConfig_ClearFaults-response>"
  "kortex_driver/ActuatorConfig_ClearFaultsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ActuatorConfig_ClearFaults-response)))
  "Returns string type for a service object of type 'ActuatorConfig_ClearFaults-response"
  "kortex_driver/ActuatorConfig_ClearFaultsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ActuatorConfig_ClearFaults-response>)))
  "Returns md5sum for a message object of type '<ActuatorConfig_ClearFaults-response>"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ActuatorConfig_ClearFaults-response)))
  "Returns md5sum for a message object of type 'ActuatorConfig_ClearFaults-response"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ActuatorConfig_ClearFaults-response>)))
  "Returns full string definition for message of type '<ActuatorConfig_ClearFaults-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ActuatorConfig_ClearFaults-response)))
  "Returns full string definition for message of type 'ActuatorConfig_ClearFaults-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ActuatorConfig_ClearFaults-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ActuatorConfig_ClearFaults-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ActuatorConfig_ClearFaults-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ActuatorConfig_ClearFaults)))
  'ActuatorConfig_ClearFaults-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ActuatorConfig_ClearFaults)))
  'ActuatorConfig_ClearFaults-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ActuatorConfig_ClearFaults)))
  "Returns string type for a service object of type '<ActuatorConfig_ClearFaults>"
  "kortex_driver/ActuatorConfig_ClearFaults")