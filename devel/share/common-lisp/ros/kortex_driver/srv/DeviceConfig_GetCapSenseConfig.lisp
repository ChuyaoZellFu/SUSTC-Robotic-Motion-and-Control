; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude DeviceConfig_GetCapSenseConfig-request.msg.html

(cl:defclass <DeviceConfig_GetCapSenseConfig-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass DeviceConfig_GetCapSenseConfig-request (<DeviceConfig_GetCapSenseConfig-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeviceConfig_GetCapSenseConfig-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeviceConfig_GetCapSenseConfig-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<DeviceConfig_GetCapSenseConfig-request> is deprecated: use kortex_driver-srv:DeviceConfig_GetCapSenseConfig-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <DeviceConfig_GetCapSenseConfig-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeviceConfig_GetCapSenseConfig-request>) ostream)
  "Serializes a message object of type '<DeviceConfig_GetCapSenseConfig-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeviceConfig_GetCapSenseConfig-request>) istream)
  "Deserializes a message object of type '<DeviceConfig_GetCapSenseConfig-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeviceConfig_GetCapSenseConfig-request>)))
  "Returns string type for a service object of type '<DeviceConfig_GetCapSenseConfig-request>"
  "kortex_driver/DeviceConfig_GetCapSenseConfigRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeviceConfig_GetCapSenseConfig-request)))
  "Returns string type for a service object of type 'DeviceConfig_GetCapSenseConfig-request"
  "kortex_driver/DeviceConfig_GetCapSenseConfigRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeviceConfig_GetCapSenseConfig-request>)))
  "Returns md5sum for a message object of type '<DeviceConfig_GetCapSenseConfig-request>"
  "067bae44ab43b02bd4decf501211ef0c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeviceConfig_GetCapSenseConfig-request)))
  "Returns md5sum for a message object of type 'DeviceConfig_GetCapSenseConfig-request"
  "067bae44ab43b02bd4decf501211ef0c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeviceConfig_GetCapSenseConfig-request>)))
  "Returns full string definition for message of type '<DeviceConfig_GetCapSenseConfig-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeviceConfig_GetCapSenseConfig-request)))
  "Returns full string definition for message of type 'DeviceConfig_GetCapSenseConfig-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeviceConfig_GetCapSenseConfig-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeviceConfig_GetCapSenseConfig-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DeviceConfig_GetCapSenseConfig-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude DeviceConfig_GetCapSenseConfig-response.msg.html

(cl:defclass <DeviceConfig_GetCapSenseConfig-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:DeviceConfig_CapSenseConfig
    :initform (cl:make-instance 'kortex_driver-msg:DeviceConfig_CapSenseConfig)))
)

(cl:defclass DeviceConfig_GetCapSenseConfig-response (<DeviceConfig_GetCapSenseConfig-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeviceConfig_GetCapSenseConfig-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeviceConfig_GetCapSenseConfig-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<DeviceConfig_GetCapSenseConfig-response> is deprecated: use kortex_driver-srv:DeviceConfig_GetCapSenseConfig-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <DeviceConfig_GetCapSenseConfig-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeviceConfig_GetCapSenseConfig-response>) ostream)
  "Serializes a message object of type '<DeviceConfig_GetCapSenseConfig-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeviceConfig_GetCapSenseConfig-response>) istream)
  "Deserializes a message object of type '<DeviceConfig_GetCapSenseConfig-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeviceConfig_GetCapSenseConfig-response>)))
  "Returns string type for a service object of type '<DeviceConfig_GetCapSenseConfig-response>"
  "kortex_driver/DeviceConfig_GetCapSenseConfigResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeviceConfig_GetCapSenseConfig-response)))
  "Returns string type for a service object of type 'DeviceConfig_GetCapSenseConfig-response"
  "kortex_driver/DeviceConfig_GetCapSenseConfigResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeviceConfig_GetCapSenseConfig-response>)))
  "Returns md5sum for a message object of type '<DeviceConfig_GetCapSenseConfig-response>"
  "067bae44ab43b02bd4decf501211ef0c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeviceConfig_GetCapSenseConfig-response)))
  "Returns md5sum for a message object of type 'DeviceConfig_GetCapSenseConfig-response"
  "067bae44ab43b02bd4decf501211ef0c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeviceConfig_GetCapSenseConfig-response>)))
  "Returns full string definition for message of type '<DeviceConfig_GetCapSenseConfig-response>"
  (cl:format cl:nil "DeviceConfig_CapSenseConfig output~%~%================================================================================~%MSG: kortex_driver/DeviceConfig_CapSenseConfig~%~%uint32 mode~%float32 threshold_a~%float32 threshold_b~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeviceConfig_GetCapSenseConfig-response)))
  "Returns full string definition for message of type 'DeviceConfig_GetCapSenseConfig-response"
  (cl:format cl:nil "DeviceConfig_CapSenseConfig output~%~%================================================================================~%MSG: kortex_driver/DeviceConfig_CapSenseConfig~%~%uint32 mode~%float32 threshold_a~%float32 threshold_b~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeviceConfig_GetCapSenseConfig-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeviceConfig_GetCapSenseConfig-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DeviceConfig_GetCapSenseConfig-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DeviceConfig_GetCapSenseConfig)))
  'DeviceConfig_GetCapSenseConfig-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DeviceConfig_GetCapSenseConfig)))
  'DeviceConfig_GetCapSenseConfig-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeviceConfig_GetCapSenseConfig)))
  "Returns string type for a service object of type '<DeviceConfig_GetCapSenseConfig>"
  "kortex_driver/DeviceConfig_GetCapSenseConfig")