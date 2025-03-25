; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude DuplicateMapping-request.msg.html

(cl:defclass <DuplicateMapping-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:MappingHandle
    :initform (cl:make-instance 'kortex_driver-msg:MappingHandle)))
)

(cl:defclass DuplicateMapping-request (<DuplicateMapping-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DuplicateMapping-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DuplicateMapping-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<DuplicateMapping-request> is deprecated: use kortex_driver-srv:DuplicateMapping-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <DuplicateMapping-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DuplicateMapping-request>) ostream)
  "Serializes a message object of type '<DuplicateMapping-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DuplicateMapping-request>) istream)
  "Deserializes a message object of type '<DuplicateMapping-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DuplicateMapping-request>)))
  "Returns string type for a service object of type '<DuplicateMapping-request>"
  "kortex_driver/DuplicateMappingRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DuplicateMapping-request)))
  "Returns string type for a service object of type 'DuplicateMapping-request"
  "kortex_driver/DuplicateMappingRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DuplicateMapping-request>)))
  "Returns md5sum for a message object of type '<DuplicateMapping-request>"
  "bfcefbce8997010c78c9f62aadaabcab")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DuplicateMapping-request)))
  "Returns md5sum for a message object of type 'DuplicateMapping-request"
  "bfcefbce8997010c78c9f62aadaabcab")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DuplicateMapping-request>)))
  "Returns full string definition for message of type '<DuplicateMapping-request>"
  (cl:format cl:nil "MappingHandle input~%~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DuplicateMapping-request)))
  "Returns full string definition for message of type 'DuplicateMapping-request"
  (cl:format cl:nil "MappingHandle input~%~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DuplicateMapping-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DuplicateMapping-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DuplicateMapping-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude DuplicateMapping-response.msg.html

(cl:defclass <DuplicateMapping-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:MappingHandle
    :initform (cl:make-instance 'kortex_driver-msg:MappingHandle)))
)

(cl:defclass DuplicateMapping-response (<DuplicateMapping-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DuplicateMapping-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DuplicateMapping-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<DuplicateMapping-response> is deprecated: use kortex_driver-srv:DuplicateMapping-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <DuplicateMapping-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DuplicateMapping-response>) ostream)
  "Serializes a message object of type '<DuplicateMapping-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DuplicateMapping-response>) istream)
  "Deserializes a message object of type '<DuplicateMapping-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DuplicateMapping-response>)))
  "Returns string type for a service object of type '<DuplicateMapping-response>"
  "kortex_driver/DuplicateMappingResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DuplicateMapping-response)))
  "Returns string type for a service object of type 'DuplicateMapping-response"
  "kortex_driver/DuplicateMappingResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DuplicateMapping-response>)))
  "Returns md5sum for a message object of type '<DuplicateMapping-response>"
  "bfcefbce8997010c78c9f62aadaabcab")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DuplicateMapping-response)))
  "Returns md5sum for a message object of type 'DuplicateMapping-response"
  "bfcefbce8997010c78c9f62aadaabcab")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DuplicateMapping-response>)))
  "Returns full string definition for message of type '<DuplicateMapping-response>"
  (cl:format cl:nil "MappingHandle output~%~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DuplicateMapping-response)))
  "Returns full string definition for message of type 'DuplicateMapping-response"
  (cl:format cl:nil "MappingHandle output~%~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DuplicateMapping-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DuplicateMapping-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DuplicateMapping-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DuplicateMapping)))
  'DuplicateMapping-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DuplicateMapping)))
  'DuplicateMapping-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DuplicateMapping)))
  "Returns string type for a service object of type '<DuplicateMapping>"
  "kortex_driver/DuplicateMapping")