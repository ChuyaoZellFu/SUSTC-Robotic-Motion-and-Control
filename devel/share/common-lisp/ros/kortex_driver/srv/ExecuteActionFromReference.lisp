; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude ExecuteActionFromReference-request.msg.html

(cl:defclass <ExecuteActionFromReference-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:ActionHandle
    :initform (cl:make-instance 'kortex_driver-msg:ActionHandle)))
)

(cl:defclass ExecuteActionFromReference-request (<ExecuteActionFromReference-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExecuteActionFromReference-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExecuteActionFromReference-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ExecuteActionFromReference-request> is deprecated: use kortex_driver-srv:ExecuteActionFromReference-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <ExecuteActionFromReference-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExecuteActionFromReference-request>) ostream)
  "Serializes a message object of type '<ExecuteActionFromReference-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExecuteActionFromReference-request>) istream)
  "Deserializes a message object of type '<ExecuteActionFromReference-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExecuteActionFromReference-request>)))
  "Returns string type for a service object of type '<ExecuteActionFromReference-request>"
  "kortex_driver/ExecuteActionFromReferenceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExecuteActionFromReference-request)))
  "Returns string type for a service object of type 'ExecuteActionFromReference-request"
  "kortex_driver/ExecuteActionFromReferenceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExecuteActionFromReference-request>)))
  "Returns md5sum for a message object of type '<ExecuteActionFromReference-request>"
  "39696246fa7132aebfa0097dedbf54c1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExecuteActionFromReference-request)))
  "Returns md5sum for a message object of type 'ExecuteActionFromReference-request"
  "39696246fa7132aebfa0097dedbf54c1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExecuteActionFromReference-request>)))
  "Returns full string definition for message of type '<ExecuteActionFromReference-request>"
  (cl:format cl:nil "ActionHandle input~%~%================================================================================~%MSG: kortex_driver/ActionHandle~%~%uint32 identifier~%uint32 action_type~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExecuteActionFromReference-request)))
  "Returns full string definition for message of type 'ExecuteActionFromReference-request"
  (cl:format cl:nil "ActionHandle input~%~%================================================================================~%MSG: kortex_driver/ActionHandle~%~%uint32 identifier~%uint32 action_type~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExecuteActionFromReference-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExecuteActionFromReference-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ExecuteActionFromReference-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude ExecuteActionFromReference-response.msg.html

(cl:defclass <ExecuteActionFromReference-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass ExecuteActionFromReference-response (<ExecuteActionFromReference-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExecuteActionFromReference-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExecuteActionFromReference-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ExecuteActionFromReference-response> is deprecated: use kortex_driver-srv:ExecuteActionFromReference-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <ExecuteActionFromReference-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExecuteActionFromReference-response>) ostream)
  "Serializes a message object of type '<ExecuteActionFromReference-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExecuteActionFromReference-response>) istream)
  "Deserializes a message object of type '<ExecuteActionFromReference-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExecuteActionFromReference-response>)))
  "Returns string type for a service object of type '<ExecuteActionFromReference-response>"
  "kortex_driver/ExecuteActionFromReferenceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExecuteActionFromReference-response)))
  "Returns string type for a service object of type 'ExecuteActionFromReference-response"
  "kortex_driver/ExecuteActionFromReferenceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExecuteActionFromReference-response>)))
  "Returns md5sum for a message object of type '<ExecuteActionFromReference-response>"
  "39696246fa7132aebfa0097dedbf54c1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExecuteActionFromReference-response)))
  "Returns md5sum for a message object of type 'ExecuteActionFromReference-response"
  "39696246fa7132aebfa0097dedbf54c1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExecuteActionFromReference-response>)))
  "Returns full string definition for message of type '<ExecuteActionFromReference-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExecuteActionFromReference-response)))
  "Returns full string definition for message of type 'ExecuteActionFromReference-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExecuteActionFromReference-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExecuteActionFromReference-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ExecuteActionFromReference-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ExecuteActionFromReference)))
  'ExecuteActionFromReference-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ExecuteActionFromReference)))
  'ExecuteActionFromReference-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExecuteActionFromReference)))
  "Returns string type for a service object of type '<ExecuteActionFromReference>"
  "kortex_driver/ExecuteActionFromReference")