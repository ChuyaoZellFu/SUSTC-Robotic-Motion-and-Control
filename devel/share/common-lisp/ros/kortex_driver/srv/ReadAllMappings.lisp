; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude ReadAllMappings-request.msg.html

(cl:defclass <ReadAllMappings-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass ReadAllMappings-request (<ReadAllMappings-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReadAllMappings-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReadAllMappings-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ReadAllMappings-request> is deprecated: use kortex_driver-srv:ReadAllMappings-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <ReadAllMappings-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReadAllMappings-request>) ostream)
  "Serializes a message object of type '<ReadAllMappings-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReadAllMappings-request>) istream)
  "Deserializes a message object of type '<ReadAllMappings-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReadAllMappings-request>)))
  "Returns string type for a service object of type '<ReadAllMappings-request>"
  "kortex_driver/ReadAllMappingsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadAllMappings-request)))
  "Returns string type for a service object of type 'ReadAllMappings-request"
  "kortex_driver/ReadAllMappingsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReadAllMappings-request>)))
  "Returns md5sum for a message object of type '<ReadAllMappings-request>"
  "b5feb1f35cacb5c11e26808022e25f5c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReadAllMappings-request)))
  "Returns md5sum for a message object of type 'ReadAllMappings-request"
  "b5feb1f35cacb5c11e26808022e25f5c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReadAllMappings-request>)))
  "Returns full string definition for message of type '<ReadAllMappings-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReadAllMappings-request)))
  "Returns full string definition for message of type 'ReadAllMappings-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReadAllMappings-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReadAllMappings-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ReadAllMappings-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude ReadAllMappings-response.msg.html

(cl:defclass <ReadAllMappings-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:MappingList
    :initform (cl:make-instance 'kortex_driver-msg:MappingList)))
)

(cl:defclass ReadAllMappings-response (<ReadAllMappings-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReadAllMappings-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReadAllMappings-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ReadAllMappings-response> is deprecated: use kortex_driver-srv:ReadAllMappings-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <ReadAllMappings-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReadAllMappings-response>) ostream)
  "Serializes a message object of type '<ReadAllMappings-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReadAllMappings-response>) istream)
  "Deserializes a message object of type '<ReadAllMappings-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReadAllMappings-response>)))
  "Returns string type for a service object of type '<ReadAllMappings-response>"
  "kortex_driver/ReadAllMappingsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadAllMappings-response)))
  "Returns string type for a service object of type 'ReadAllMappings-response"
  "kortex_driver/ReadAllMappingsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReadAllMappings-response>)))
  "Returns md5sum for a message object of type '<ReadAllMappings-response>"
  "b5feb1f35cacb5c11e26808022e25f5c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReadAllMappings-response)))
  "Returns md5sum for a message object of type 'ReadAllMappings-response"
  "b5feb1f35cacb5c11e26808022e25f5c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReadAllMappings-response>)))
  "Returns full string definition for message of type '<ReadAllMappings-response>"
  (cl:format cl:nil "MappingList output~%~%================================================================================~%MSG: kortex_driver/MappingList~%~%Mapping[] mappings~%================================================================================~%MSG: kortex_driver/Mapping~%~%MappingHandle handle~%string name~%uint32 controller_identifier~%MapGroupHandle active_map_group_handle~%MapGroupHandle[] map_group_handles~%MapHandle active_map_handle~%MapHandle[] map_handles~%string application_data~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapGroupHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReadAllMappings-response)))
  "Returns full string definition for message of type 'ReadAllMappings-response"
  (cl:format cl:nil "MappingList output~%~%================================================================================~%MSG: kortex_driver/MappingList~%~%Mapping[] mappings~%================================================================================~%MSG: kortex_driver/Mapping~%~%MappingHandle handle~%string name~%uint32 controller_identifier~%MapGroupHandle active_map_group_handle~%MapGroupHandle[] map_group_handles~%MapHandle active_map_handle~%MapHandle[] map_handles~%string application_data~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapGroupHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReadAllMappings-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReadAllMappings-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ReadAllMappings-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ReadAllMappings)))
  'ReadAllMappings-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ReadAllMappings)))
  'ReadAllMappings-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadAllMappings)))
  "Returns string type for a service object of type '<ReadAllMappings>"
  "kortex_driver/ReadAllMappings")