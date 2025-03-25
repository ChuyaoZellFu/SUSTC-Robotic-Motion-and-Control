; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude DeviceConfig_SetCapSenseConfig-request.msg.html

(cl:defclass <DeviceConfig_SetCapSenseConfig-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:DeviceConfig_CapSenseConfig
    :initform (cl:make-instance 'kortex_driver-msg:DeviceConfig_CapSenseConfig)))
)

(cl:defclass DeviceConfig_SetCapSenseConfig-request (<DeviceConfig_SetCapSenseConfig-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeviceConfig_SetCapSenseConfig-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeviceConfig_SetCapSenseConfig-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<DeviceConfig_SetCapSenseConfig-request> is deprecated: use kortex_driver-srv:DeviceConfig_SetCapSenseConfig-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <DeviceConfig_SetCapSenseConfig-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeviceConfig_SetCapSenseConfig-request>) ostream)
  "Serializes a message object of type '<DeviceConfig_SetCapSenseConfig-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeviceConfig_SetCapSenseConfig-request>) istream)
  "Deserializes a message object of type '<DeviceConfig_SetCapSenseConfig-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeviceConfig_SetCapSenseConfig-request>)))
  "Returns string type for a service object of type '<DeviceConfig_SetCapSenseConfig-request>"
  "kortex_driver/DeviceConfig_SetCapSenseConfigRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeviceConfig_SetCapSenseConfig-request)))
  "Returns string type for a service object of type 'DeviceConfig_SetCapSenseConfig-request"
  "kortex_driver/DeviceConfig_SetCapSenseConfigRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeviceConfig_SetCapSenseConfig-request>)))
  "Returns md5sum for a message object of type '<DeviceConfig_SetCapSenseConfig-request>"
  "af529d9737e7a99888f758db0a779b87")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeviceConfig_SetCapSenseConfig-request)))
  "Returns md5sum for a message object of type 'DeviceConfig_SetCapSenseConfig-request"
  "af529d9737e7a99888f758db0a779b87")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeviceConfig_SetCapSenseConfig-request>)))
  "Returns full string definition for message of type '<DeviceConfig_SetCapSenseConfig-request>"
  (cl:format cl:nil "DeviceConfig_CapSenseConfig input~%~%================================================================================~%MSG: kortex_driver/DeviceConfig_CapSenseConfig~%~%uint32 mode~%float32 threshold_a~%float32 threshold_b~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeviceConfig_SetCapSenseConfig-request)))
  "Returns full string definition for message of type 'DeviceConfig_SetCapSenseConfig-request"
  (cl:format cl:nil "DeviceConfig_CapSenseConfig input~%~%================================================================================~%MSG: kortex_driver/DeviceConfig_CapSenseConfig~%~%uint32 mode~%float32 threshold_a~%float32 threshold_b~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeviceConfig_SetCapSenseConfig-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeviceConfig_SetCapSenseConfig-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DeviceConfig_SetCapSenseConfig-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude DeviceConfig_SetCapSenseConfig-response.msg.html

(cl:defclass <DeviceConfig_SetCapSenseConfig-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass DeviceConfig_SetCapSenseConfig-response (<DeviceConfig_SetCapSenseConfig-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeviceConfig_SetCapSenseConfig-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeviceConfig_SetCapSenseConfig-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<DeviceConfig_SetCapSenseConfig-response> is deprecated: use kortex_driver-srv:DeviceConfig_SetCapSenseConfig-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <DeviceConfig_SetCapSenseConfig-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeviceConfig_SetCapSenseConfig-response>) ostream)
  "Serializes a message object of type '<DeviceConfig_SetCapSenseConfig-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeviceConfig_SetCapSenseConfig-response>) istream)
  "Deserializes a message object of type '<DeviceConfig_SetCapSenseConfig-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeviceConfig_SetCapSenseConfig-response>)))
  "Returns string type for a service object of type '<DeviceConfig_SetCapSenseConfig-response>"
  "kortex_driver/DeviceConfig_SetCapSenseConfigResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeviceConfig_SetCapSenseConfig-response)))
  "Returns string type for a service object of type 'DeviceConfig_SetCapSenseConfig-response"
  "kortex_driver/DeviceConfig_SetCapSenseConfigResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeviceConfig_SetCapSenseConfig-response>)))
  "Returns md5sum for a message object of type '<DeviceConfig_SetCapSenseConfig-response>"
  "af529d9737e7a99888f758db0a779b87")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeviceConfig_SetCapSenseConfig-response)))
  "Returns md5sum for a message object of type 'DeviceConfig_SetCapSenseConfig-response"
  "af529d9737e7a99888f758db0a779b87")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeviceConfig_SetCapSenseConfig-response>)))
  "Returns full string definition for message of type '<DeviceConfig_SetCapSenseConfig-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeviceConfig_SetCapSenseConfig-response)))
  "Returns full string definition for message of type 'DeviceConfig_SetCapSenseConfig-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeviceConfig_SetCapSenseConfig-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeviceConfig_SetCapSenseConfig-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DeviceConfig_SetCapSenseConfig-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DeviceConfig_SetCapSenseConfig)))
  'DeviceConfig_SetCapSenseConfig-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DeviceConfig_SetCapSenseConfig)))
  'DeviceConfig_SetCapSenseConfig-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeviceConfig_SetCapSenseConfig)))
  "Returns string type for a service object of type '<DeviceConfig_SetCapSenseConfig>"
  "kortex_driver/DeviceConfig_SetCapSenseConfig")