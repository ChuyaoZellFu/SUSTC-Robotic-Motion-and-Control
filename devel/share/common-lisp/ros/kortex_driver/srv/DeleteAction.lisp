; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude DeleteAction-request.msg.html

(cl:defclass <DeleteAction-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:ActionHandle
    :initform (cl:make-instance 'kortex_driver-msg:ActionHandle)))
)

(cl:defclass DeleteAction-request (<DeleteAction-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeleteAction-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeleteAction-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<DeleteAction-request> is deprecated: use kortex_driver-srv:DeleteAction-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <DeleteAction-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeleteAction-request>) ostream)
  "Serializes a message object of type '<DeleteAction-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeleteAction-request>) istream)
  "Deserializes a message object of type '<DeleteAction-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeleteAction-request>)))
  "Returns string type for a service object of type '<DeleteAction-request>"
  "kortex_driver/DeleteActionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeleteAction-request)))
  "Returns string type for a service object of type 'DeleteAction-request"
  "kortex_driver/DeleteActionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeleteAction-request>)))
  "Returns md5sum for a message object of type '<DeleteAction-request>"
  "39696246fa7132aebfa0097dedbf54c1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeleteAction-request)))
  "Returns md5sum for a message object of type 'DeleteAction-request"
  "39696246fa7132aebfa0097dedbf54c1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeleteAction-request>)))
  "Returns full string definition for message of type '<DeleteAction-request>"
  (cl:format cl:nil "ActionHandle input~%~%================================================================================~%MSG: kortex_driver/ActionHandle~%~%uint32 identifier~%uint32 action_type~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeleteAction-request)))
  "Returns full string definition for message of type 'DeleteAction-request"
  (cl:format cl:nil "ActionHandle input~%~%================================================================================~%MSG: kortex_driver/ActionHandle~%~%uint32 identifier~%uint32 action_type~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeleteAction-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeleteAction-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DeleteAction-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude DeleteAction-response.msg.html

(cl:defclass <DeleteAction-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass DeleteAction-response (<DeleteAction-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeleteAction-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeleteAction-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<DeleteAction-response> is deprecated: use kortex_driver-srv:DeleteAction-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <DeleteAction-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeleteAction-response>) ostream)
  "Serializes a message object of type '<DeleteAction-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeleteAction-response>) istream)
  "Deserializes a message object of type '<DeleteAction-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeleteAction-response>)))
  "Returns string type for a service object of type '<DeleteAction-response>"
  "kortex_driver/DeleteActionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeleteAction-response)))
  "Returns string type for a service object of type 'DeleteAction-response"
  "kortex_driver/DeleteActionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeleteAction-response>)))
  "Returns md5sum for a message object of type '<DeleteAction-response>"
  "39696246fa7132aebfa0097dedbf54c1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeleteAction-response)))
  "Returns md5sum for a message object of type 'DeleteAction-response"
  "39696246fa7132aebfa0097dedbf54c1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeleteAction-response>)))
  "Returns full string definition for message of type '<DeleteAction-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeleteAction-response)))
  "Returns full string definition for message of type 'DeleteAction-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeleteAction-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeleteAction-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DeleteAction-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DeleteAction)))
  'DeleteAction-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DeleteAction)))
  'DeleteAction-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeleteAction)))
  "Returns string type for a service object of type '<DeleteAction>"
  "kortex_driver/DeleteAction")