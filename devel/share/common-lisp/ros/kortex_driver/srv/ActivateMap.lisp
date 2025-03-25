; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude ActivateMap-request.msg.html

(cl:defclass <ActivateMap-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:ActivateMapHandle
    :initform (cl:make-instance 'kortex_driver-msg:ActivateMapHandle)))
)

(cl:defclass ActivateMap-request (<ActivateMap-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ActivateMap-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ActivateMap-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ActivateMap-request> is deprecated: use kortex_driver-srv:ActivateMap-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <ActivateMap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ActivateMap-request>) ostream)
  "Serializes a message object of type '<ActivateMap-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ActivateMap-request>) istream)
  "Deserializes a message object of type '<ActivateMap-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ActivateMap-request>)))
  "Returns string type for a service object of type '<ActivateMap-request>"
  "kortex_driver/ActivateMapRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ActivateMap-request)))
  "Returns string type for a service object of type 'ActivateMap-request"
  "kortex_driver/ActivateMapRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ActivateMap-request>)))
  "Returns md5sum for a message object of type '<ActivateMap-request>"
  "48d813b9a4f09a106ff67b248afca7e9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ActivateMap-request)))
  "Returns md5sum for a message object of type 'ActivateMap-request"
  "48d813b9a4f09a106ff67b248afca7e9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ActivateMap-request>)))
  "Returns full string definition for message of type '<ActivateMap-request>"
  (cl:format cl:nil "ActivateMapHandle input~%~%================================================================================~%MSG: kortex_driver/ActivateMapHandle~%~%MappingHandle mapping_handle~%MapGroupHandle map_group_handle~%MapHandle map_handle~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapGroupHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ActivateMap-request)))
  "Returns full string definition for message of type 'ActivateMap-request"
  (cl:format cl:nil "ActivateMapHandle input~%~%================================================================================~%MSG: kortex_driver/ActivateMapHandle~%~%MappingHandle mapping_handle~%MapGroupHandle map_group_handle~%MapHandle map_handle~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapGroupHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ActivateMap-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ActivateMap-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ActivateMap-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude ActivateMap-response.msg.html

(cl:defclass <ActivateMap-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass ActivateMap-response (<ActivateMap-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ActivateMap-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ActivateMap-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ActivateMap-response> is deprecated: use kortex_driver-srv:ActivateMap-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <ActivateMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ActivateMap-response>) ostream)
  "Serializes a message object of type '<ActivateMap-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ActivateMap-response>) istream)
  "Deserializes a message object of type '<ActivateMap-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ActivateMap-response>)))
  "Returns string type for a service object of type '<ActivateMap-response>"
  "kortex_driver/ActivateMapResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ActivateMap-response)))
  "Returns string type for a service object of type 'ActivateMap-response"
  "kortex_driver/ActivateMapResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ActivateMap-response>)))
  "Returns md5sum for a message object of type '<ActivateMap-response>"
  "48d813b9a4f09a106ff67b248afca7e9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ActivateMap-response)))
  "Returns md5sum for a message object of type 'ActivateMap-response"
  "48d813b9a4f09a106ff67b248afca7e9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ActivateMap-response>)))
  "Returns full string definition for message of type '<ActivateMap-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ActivateMap-response)))
  "Returns full string definition for message of type 'ActivateMap-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ActivateMap-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ActivateMap-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ActivateMap-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ActivateMap)))
  'ActivateMap-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ActivateMap)))
  'ActivateMap-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ActivateMap)))
  "Returns string type for a service object of type '<ActivateMap>"
  "kortex_driver/ActivateMap")