; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude DeleteMapping-request.msg.html

(cl:defclass <DeleteMapping-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:MappingHandle
    :initform (cl:make-instance 'kortex_driver-msg:MappingHandle)))
)

(cl:defclass DeleteMapping-request (<DeleteMapping-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeleteMapping-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeleteMapping-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<DeleteMapping-request> is deprecated: use kortex_driver-srv:DeleteMapping-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <DeleteMapping-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeleteMapping-request>) ostream)
  "Serializes a message object of type '<DeleteMapping-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeleteMapping-request>) istream)
  "Deserializes a message object of type '<DeleteMapping-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeleteMapping-request>)))
  "Returns string type for a service object of type '<DeleteMapping-request>"
  "kortex_driver/DeleteMappingRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeleteMapping-request)))
  "Returns string type for a service object of type 'DeleteMapping-request"
  "kortex_driver/DeleteMappingRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeleteMapping-request>)))
  "Returns md5sum for a message object of type '<DeleteMapping-request>"
  "04dfaeca45772f660e0913aa84774e13")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeleteMapping-request)))
  "Returns md5sum for a message object of type 'DeleteMapping-request"
  "04dfaeca45772f660e0913aa84774e13")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeleteMapping-request>)))
  "Returns full string definition for message of type '<DeleteMapping-request>"
  (cl:format cl:nil "MappingHandle input~%~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeleteMapping-request)))
  "Returns full string definition for message of type 'DeleteMapping-request"
  (cl:format cl:nil "MappingHandle input~%~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeleteMapping-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeleteMapping-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DeleteMapping-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude DeleteMapping-response.msg.html

(cl:defclass <DeleteMapping-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass DeleteMapping-response (<DeleteMapping-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeleteMapping-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeleteMapping-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<DeleteMapping-response> is deprecated: use kortex_driver-srv:DeleteMapping-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <DeleteMapping-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeleteMapping-response>) ostream)
  "Serializes a message object of type '<DeleteMapping-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeleteMapping-response>) istream)
  "Deserializes a message object of type '<DeleteMapping-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeleteMapping-response>)))
  "Returns string type for a service object of type '<DeleteMapping-response>"
  "kortex_driver/DeleteMappingResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeleteMapping-response)))
  "Returns string type for a service object of type 'DeleteMapping-response"
  "kortex_driver/DeleteMappingResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeleteMapping-response>)))
  "Returns md5sum for a message object of type '<DeleteMapping-response>"
  "04dfaeca45772f660e0913aa84774e13")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeleteMapping-response)))
  "Returns md5sum for a message object of type 'DeleteMapping-response"
  "04dfaeca45772f660e0913aa84774e13")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeleteMapping-response>)))
  "Returns full string definition for message of type '<DeleteMapping-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeleteMapping-response)))
  "Returns full string definition for message of type 'DeleteMapping-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeleteMapping-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeleteMapping-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DeleteMapping-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DeleteMapping)))
  'DeleteMapping-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DeleteMapping)))
  'DeleteMapping-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeleteMapping)))
  "Returns string type for a service object of type '<DeleteMapping>"
  "kortex_driver/DeleteMapping")