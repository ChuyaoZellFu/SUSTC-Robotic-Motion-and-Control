; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude ReadMapping-request.msg.html

(cl:defclass <ReadMapping-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:MappingHandle
    :initform (cl:make-instance 'kortex_driver-msg:MappingHandle)))
)

(cl:defclass ReadMapping-request (<ReadMapping-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReadMapping-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReadMapping-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ReadMapping-request> is deprecated: use kortex_driver-srv:ReadMapping-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <ReadMapping-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReadMapping-request>) ostream)
  "Serializes a message object of type '<ReadMapping-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReadMapping-request>) istream)
  "Deserializes a message object of type '<ReadMapping-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReadMapping-request>)))
  "Returns string type for a service object of type '<ReadMapping-request>"
  "kortex_driver/ReadMappingRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadMapping-request)))
  "Returns string type for a service object of type 'ReadMapping-request"
  "kortex_driver/ReadMappingRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReadMapping-request>)))
  "Returns md5sum for a message object of type '<ReadMapping-request>"
  "a098ab486d258af0a3206d3f4ae9d15c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReadMapping-request)))
  "Returns md5sum for a message object of type 'ReadMapping-request"
  "a098ab486d258af0a3206d3f4ae9d15c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReadMapping-request>)))
  "Returns full string definition for message of type '<ReadMapping-request>"
  (cl:format cl:nil "MappingHandle input~%~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReadMapping-request)))
  "Returns full string definition for message of type 'ReadMapping-request"
  (cl:format cl:nil "MappingHandle input~%~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReadMapping-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReadMapping-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ReadMapping-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude ReadMapping-response.msg.html

(cl:defclass <ReadMapping-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Mapping
    :initform (cl:make-instance 'kortex_driver-msg:Mapping)))
)

(cl:defclass ReadMapping-response (<ReadMapping-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReadMapping-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReadMapping-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ReadMapping-response> is deprecated: use kortex_driver-srv:ReadMapping-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <ReadMapping-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReadMapping-response>) ostream)
  "Serializes a message object of type '<ReadMapping-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReadMapping-response>) istream)
  "Deserializes a message object of type '<ReadMapping-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReadMapping-response>)))
  "Returns string type for a service object of type '<ReadMapping-response>"
  "kortex_driver/ReadMappingResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadMapping-response)))
  "Returns string type for a service object of type 'ReadMapping-response"
  "kortex_driver/ReadMappingResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReadMapping-response>)))
  "Returns md5sum for a message object of type '<ReadMapping-response>"
  "a098ab486d258af0a3206d3f4ae9d15c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReadMapping-response)))
  "Returns md5sum for a message object of type 'ReadMapping-response"
  "a098ab486d258af0a3206d3f4ae9d15c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReadMapping-response>)))
  "Returns full string definition for message of type '<ReadMapping-response>"
  (cl:format cl:nil "Mapping output~%~%================================================================================~%MSG: kortex_driver/Mapping~%~%MappingHandle handle~%string name~%uint32 controller_identifier~%MapGroupHandle active_map_group_handle~%MapGroupHandle[] map_group_handles~%MapHandle active_map_handle~%MapHandle[] map_handles~%string application_data~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapGroupHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReadMapping-response)))
  "Returns full string definition for message of type 'ReadMapping-response"
  (cl:format cl:nil "Mapping output~%~%================================================================================~%MSG: kortex_driver/Mapping~%~%MappingHandle handle~%string name~%uint32 controller_identifier~%MapGroupHandle active_map_group_handle~%MapGroupHandle[] map_group_handles~%MapHandle active_map_handle~%MapHandle[] map_handles~%string application_data~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapGroupHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReadMapping-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReadMapping-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ReadMapping-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ReadMapping)))
  'ReadMapping-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ReadMapping)))
  'ReadMapping-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadMapping)))
  "Returns string type for a service object of type '<ReadMapping>"
  "kortex_driver/ReadMapping")