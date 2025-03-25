; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude UpdateMapping-request.msg.html

(cl:defclass <UpdateMapping-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Mapping
    :initform (cl:make-instance 'kortex_driver-msg:Mapping)))
)

(cl:defclass UpdateMapping-request (<UpdateMapping-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UpdateMapping-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UpdateMapping-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<UpdateMapping-request> is deprecated: use kortex_driver-srv:UpdateMapping-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <UpdateMapping-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UpdateMapping-request>) ostream)
  "Serializes a message object of type '<UpdateMapping-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UpdateMapping-request>) istream)
  "Deserializes a message object of type '<UpdateMapping-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UpdateMapping-request>)))
  "Returns string type for a service object of type '<UpdateMapping-request>"
  "kortex_driver/UpdateMappingRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdateMapping-request)))
  "Returns string type for a service object of type 'UpdateMapping-request"
  "kortex_driver/UpdateMappingRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UpdateMapping-request>)))
  "Returns md5sum for a message object of type '<UpdateMapping-request>"
  "921c2b05776ccd75721af97241aa399d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UpdateMapping-request)))
  "Returns md5sum for a message object of type 'UpdateMapping-request"
  "921c2b05776ccd75721af97241aa399d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UpdateMapping-request>)))
  "Returns full string definition for message of type '<UpdateMapping-request>"
  (cl:format cl:nil "Mapping input~%~%================================================================================~%MSG: kortex_driver/Mapping~%~%MappingHandle handle~%string name~%uint32 controller_identifier~%MapGroupHandle active_map_group_handle~%MapGroupHandle[] map_group_handles~%MapHandle active_map_handle~%MapHandle[] map_handles~%string application_data~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapGroupHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UpdateMapping-request)))
  "Returns full string definition for message of type 'UpdateMapping-request"
  (cl:format cl:nil "Mapping input~%~%================================================================================~%MSG: kortex_driver/Mapping~%~%MappingHandle handle~%string name~%uint32 controller_identifier~%MapGroupHandle active_map_group_handle~%MapGroupHandle[] map_group_handles~%MapHandle active_map_handle~%MapHandle[] map_handles~%string application_data~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapGroupHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UpdateMapping-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UpdateMapping-request>))
  "Converts a ROS message object to a list"
  (cl:list 'UpdateMapping-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude UpdateMapping-response.msg.html

(cl:defclass <UpdateMapping-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass UpdateMapping-response (<UpdateMapping-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UpdateMapping-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UpdateMapping-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<UpdateMapping-response> is deprecated: use kortex_driver-srv:UpdateMapping-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <UpdateMapping-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UpdateMapping-response>) ostream)
  "Serializes a message object of type '<UpdateMapping-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UpdateMapping-response>) istream)
  "Deserializes a message object of type '<UpdateMapping-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UpdateMapping-response>)))
  "Returns string type for a service object of type '<UpdateMapping-response>"
  "kortex_driver/UpdateMappingResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdateMapping-response)))
  "Returns string type for a service object of type 'UpdateMapping-response"
  "kortex_driver/UpdateMappingResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UpdateMapping-response>)))
  "Returns md5sum for a message object of type '<UpdateMapping-response>"
  "921c2b05776ccd75721af97241aa399d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UpdateMapping-response)))
  "Returns md5sum for a message object of type 'UpdateMapping-response"
  "921c2b05776ccd75721af97241aa399d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UpdateMapping-response>)))
  "Returns full string definition for message of type '<UpdateMapping-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UpdateMapping-response)))
  "Returns full string definition for message of type 'UpdateMapping-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UpdateMapping-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UpdateMapping-response>))
  "Converts a ROS message object to a list"
  (cl:list 'UpdateMapping-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'UpdateMapping)))
  'UpdateMapping-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'UpdateMapping)))
  'UpdateMapping-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdateMapping)))
  "Returns string type for a service object of type '<UpdateMapping>"
  "kortex_driver/UpdateMapping")