; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude CreateMapping-request.msg.html

(cl:defclass <CreateMapping-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Mapping
    :initform (cl:make-instance 'kortex_driver-msg:Mapping)))
)

(cl:defclass CreateMapping-request (<CreateMapping-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CreateMapping-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CreateMapping-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<CreateMapping-request> is deprecated: use kortex_driver-srv:CreateMapping-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <CreateMapping-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CreateMapping-request>) ostream)
  "Serializes a message object of type '<CreateMapping-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CreateMapping-request>) istream)
  "Deserializes a message object of type '<CreateMapping-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CreateMapping-request>)))
  "Returns string type for a service object of type '<CreateMapping-request>"
  "kortex_driver/CreateMappingRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CreateMapping-request)))
  "Returns string type for a service object of type 'CreateMapping-request"
  "kortex_driver/CreateMappingRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CreateMapping-request>)))
  "Returns md5sum for a message object of type '<CreateMapping-request>"
  "1672eeb2532b125a8c2350096e2628b8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CreateMapping-request)))
  "Returns md5sum for a message object of type 'CreateMapping-request"
  "1672eeb2532b125a8c2350096e2628b8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CreateMapping-request>)))
  "Returns full string definition for message of type '<CreateMapping-request>"
  (cl:format cl:nil "Mapping input~%~%================================================================================~%MSG: kortex_driver/Mapping~%~%MappingHandle handle~%string name~%uint32 controller_identifier~%MapGroupHandle active_map_group_handle~%MapGroupHandle[] map_group_handles~%MapHandle active_map_handle~%MapHandle[] map_handles~%string application_data~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapGroupHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CreateMapping-request)))
  "Returns full string definition for message of type 'CreateMapping-request"
  (cl:format cl:nil "Mapping input~%~%================================================================================~%MSG: kortex_driver/Mapping~%~%MappingHandle handle~%string name~%uint32 controller_identifier~%MapGroupHandle active_map_group_handle~%MapGroupHandle[] map_group_handles~%MapHandle active_map_handle~%MapHandle[] map_handles~%string application_data~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapGroupHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CreateMapping-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CreateMapping-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CreateMapping-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude CreateMapping-response.msg.html

(cl:defclass <CreateMapping-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:MappingHandle
    :initform (cl:make-instance 'kortex_driver-msg:MappingHandle)))
)

(cl:defclass CreateMapping-response (<CreateMapping-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CreateMapping-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CreateMapping-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<CreateMapping-response> is deprecated: use kortex_driver-srv:CreateMapping-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <CreateMapping-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CreateMapping-response>) ostream)
  "Serializes a message object of type '<CreateMapping-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CreateMapping-response>) istream)
  "Deserializes a message object of type '<CreateMapping-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CreateMapping-response>)))
  "Returns string type for a service object of type '<CreateMapping-response>"
  "kortex_driver/CreateMappingResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CreateMapping-response)))
  "Returns string type for a service object of type 'CreateMapping-response"
  "kortex_driver/CreateMappingResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CreateMapping-response>)))
  "Returns md5sum for a message object of type '<CreateMapping-response>"
  "1672eeb2532b125a8c2350096e2628b8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CreateMapping-response)))
  "Returns md5sum for a message object of type 'CreateMapping-response"
  "1672eeb2532b125a8c2350096e2628b8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CreateMapping-response>)))
  "Returns full string definition for message of type '<CreateMapping-response>"
  (cl:format cl:nil "MappingHandle output~%~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CreateMapping-response)))
  "Returns full string definition for message of type 'CreateMapping-response"
  (cl:format cl:nil "MappingHandle output~%~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CreateMapping-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CreateMapping-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CreateMapping-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CreateMapping)))
  'CreateMapping-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CreateMapping)))
  'CreateMapping-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CreateMapping)))
  "Returns string type for a service object of type '<CreateMapping>"
  "kortex_driver/CreateMapping")