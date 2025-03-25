; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude DuplicateMap-request.msg.html

(cl:defclass <DuplicateMap-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:MapHandle
    :initform (cl:make-instance 'kortex_driver-msg:MapHandle)))
)

(cl:defclass DuplicateMap-request (<DuplicateMap-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DuplicateMap-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DuplicateMap-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<DuplicateMap-request> is deprecated: use kortex_driver-srv:DuplicateMap-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <DuplicateMap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DuplicateMap-request>) ostream)
  "Serializes a message object of type '<DuplicateMap-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DuplicateMap-request>) istream)
  "Deserializes a message object of type '<DuplicateMap-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DuplicateMap-request>)))
  "Returns string type for a service object of type '<DuplicateMap-request>"
  "kortex_driver/DuplicateMapRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DuplicateMap-request)))
  "Returns string type for a service object of type 'DuplicateMap-request"
  "kortex_driver/DuplicateMapRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DuplicateMap-request>)))
  "Returns md5sum for a message object of type '<DuplicateMap-request>"
  "bfcefbce8997010c78c9f62aadaabcab")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DuplicateMap-request)))
  "Returns md5sum for a message object of type 'DuplicateMap-request"
  "bfcefbce8997010c78c9f62aadaabcab")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DuplicateMap-request>)))
  "Returns full string definition for message of type '<DuplicateMap-request>"
  (cl:format cl:nil "MapHandle input~%~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DuplicateMap-request)))
  "Returns full string definition for message of type 'DuplicateMap-request"
  (cl:format cl:nil "MapHandle input~%~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DuplicateMap-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DuplicateMap-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DuplicateMap-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude DuplicateMap-response.msg.html

(cl:defclass <DuplicateMap-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:MapHandle
    :initform (cl:make-instance 'kortex_driver-msg:MapHandle)))
)

(cl:defclass DuplicateMap-response (<DuplicateMap-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DuplicateMap-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DuplicateMap-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<DuplicateMap-response> is deprecated: use kortex_driver-srv:DuplicateMap-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <DuplicateMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DuplicateMap-response>) ostream)
  "Serializes a message object of type '<DuplicateMap-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DuplicateMap-response>) istream)
  "Deserializes a message object of type '<DuplicateMap-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DuplicateMap-response>)))
  "Returns string type for a service object of type '<DuplicateMap-response>"
  "kortex_driver/DuplicateMapResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DuplicateMap-response)))
  "Returns string type for a service object of type 'DuplicateMap-response"
  "kortex_driver/DuplicateMapResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DuplicateMap-response>)))
  "Returns md5sum for a message object of type '<DuplicateMap-response>"
  "bfcefbce8997010c78c9f62aadaabcab")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DuplicateMap-response)))
  "Returns md5sum for a message object of type 'DuplicateMap-response"
  "bfcefbce8997010c78c9f62aadaabcab")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DuplicateMap-response>)))
  "Returns full string definition for message of type '<DuplicateMap-response>"
  (cl:format cl:nil "MapHandle output~%~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DuplicateMap-response)))
  "Returns full string definition for message of type 'DuplicateMap-response"
  (cl:format cl:nil "MapHandle output~%~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DuplicateMap-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DuplicateMap-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DuplicateMap-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DuplicateMap)))
  'DuplicateMap-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DuplicateMap)))
  'DuplicateMap-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DuplicateMap)))
  "Returns string type for a service object of type '<DuplicateMap>"
  "kortex_driver/DuplicateMap")