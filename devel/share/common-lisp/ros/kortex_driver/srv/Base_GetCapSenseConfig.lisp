; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude Base_GetCapSenseConfig-request.msg.html

(cl:defclass <Base_GetCapSenseConfig-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass Base_GetCapSenseConfig-request (<Base_GetCapSenseConfig-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Base_GetCapSenseConfig-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Base_GetCapSenseConfig-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<Base_GetCapSenseConfig-request> is deprecated: use kortex_driver-srv:Base_GetCapSenseConfig-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <Base_GetCapSenseConfig-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Base_GetCapSenseConfig-request>) ostream)
  "Serializes a message object of type '<Base_GetCapSenseConfig-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Base_GetCapSenseConfig-request>) istream)
  "Deserializes a message object of type '<Base_GetCapSenseConfig-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Base_GetCapSenseConfig-request>)))
  "Returns string type for a service object of type '<Base_GetCapSenseConfig-request>"
  "kortex_driver/Base_GetCapSenseConfigRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Base_GetCapSenseConfig-request)))
  "Returns string type for a service object of type 'Base_GetCapSenseConfig-request"
  "kortex_driver/Base_GetCapSenseConfigRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Base_GetCapSenseConfig-request>)))
  "Returns md5sum for a message object of type '<Base_GetCapSenseConfig-request>"
  "c1e479a50a0c96ec49d735b61e1b4a43")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Base_GetCapSenseConfig-request)))
  "Returns md5sum for a message object of type 'Base_GetCapSenseConfig-request"
  "c1e479a50a0c96ec49d735b61e1b4a43")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Base_GetCapSenseConfig-request>)))
  "Returns full string definition for message of type '<Base_GetCapSenseConfig-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Base_GetCapSenseConfig-request)))
  "Returns full string definition for message of type 'Base_GetCapSenseConfig-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Base_GetCapSenseConfig-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Base_GetCapSenseConfig-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Base_GetCapSenseConfig-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude Base_GetCapSenseConfig-response.msg.html

(cl:defclass <Base_GetCapSenseConfig-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Base_CapSenseConfig
    :initform (cl:make-instance 'kortex_driver-msg:Base_CapSenseConfig)))
)

(cl:defclass Base_GetCapSenseConfig-response (<Base_GetCapSenseConfig-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Base_GetCapSenseConfig-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Base_GetCapSenseConfig-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<Base_GetCapSenseConfig-response> is deprecated: use kortex_driver-srv:Base_GetCapSenseConfig-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <Base_GetCapSenseConfig-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Base_GetCapSenseConfig-response>) ostream)
  "Serializes a message object of type '<Base_GetCapSenseConfig-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Base_GetCapSenseConfig-response>) istream)
  "Deserializes a message object of type '<Base_GetCapSenseConfig-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Base_GetCapSenseConfig-response>)))
  "Returns string type for a service object of type '<Base_GetCapSenseConfig-response>"
  "kortex_driver/Base_GetCapSenseConfigResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Base_GetCapSenseConfig-response)))
  "Returns string type for a service object of type 'Base_GetCapSenseConfig-response"
  "kortex_driver/Base_GetCapSenseConfigResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Base_GetCapSenseConfig-response>)))
  "Returns md5sum for a message object of type '<Base_GetCapSenseConfig-response>"
  "c1e479a50a0c96ec49d735b61e1b4a43")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Base_GetCapSenseConfig-response)))
  "Returns md5sum for a message object of type 'Base_GetCapSenseConfig-response"
  "c1e479a50a0c96ec49d735b61e1b4a43")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Base_GetCapSenseConfig-response>)))
  "Returns full string definition for message of type '<Base_GetCapSenseConfig-response>"
  (cl:format cl:nil "Base_CapSenseConfig output~%~%================================================================================~%MSG: kortex_driver/Base_CapSenseConfig~%~%uint32 identifier~%uint32 mode~%float32 threshold_a~%float32 threshold_b~%float32 sensitivity_a~%float32 sensitivity_b~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Base_GetCapSenseConfig-response)))
  "Returns full string definition for message of type 'Base_GetCapSenseConfig-response"
  (cl:format cl:nil "Base_CapSenseConfig output~%~%================================================================================~%MSG: kortex_driver/Base_CapSenseConfig~%~%uint32 identifier~%uint32 mode~%float32 threshold_a~%float32 threshold_b~%float32 sensitivity_a~%float32 sensitivity_b~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Base_GetCapSenseConfig-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Base_GetCapSenseConfig-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Base_GetCapSenseConfig-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Base_GetCapSenseConfig)))
  'Base_GetCapSenseConfig-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Base_GetCapSenseConfig)))
  'Base_GetCapSenseConfig-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Base_GetCapSenseConfig)))
  "Returns string type for a service object of type '<Base_GetCapSenseConfig>"
  "kortex_driver/Base_GetCapSenseConfig")