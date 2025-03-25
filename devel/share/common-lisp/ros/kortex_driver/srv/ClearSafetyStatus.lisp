; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude ClearSafetyStatus-request.msg.html

(cl:defclass <ClearSafetyStatus-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:SafetyHandle
    :initform (cl:make-instance 'kortex_driver-msg:SafetyHandle)))
)

(cl:defclass ClearSafetyStatus-request (<ClearSafetyStatus-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ClearSafetyStatus-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ClearSafetyStatus-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ClearSafetyStatus-request> is deprecated: use kortex_driver-srv:ClearSafetyStatus-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <ClearSafetyStatus-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ClearSafetyStatus-request>) ostream)
  "Serializes a message object of type '<ClearSafetyStatus-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ClearSafetyStatus-request>) istream)
  "Deserializes a message object of type '<ClearSafetyStatus-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ClearSafetyStatus-request>)))
  "Returns string type for a service object of type '<ClearSafetyStatus-request>"
  "kortex_driver/ClearSafetyStatusRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ClearSafetyStatus-request)))
  "Returns string type for a service object of type 'ClearSafetyStatus-request"
  "kortex_driver/ClearSafetyStatusRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ClearSafetyStatus-request>)))
  "Returns md5sum for a message object of type '<ClearSafetyStatus-request>"
  "7960dab80443332660b3bb4bf774c9ab")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ClearSafetyStatus-request)))
  "Returns md5sum for a message object of type 'ClearSafetyStatus-request"
  "7960dab80443332660b3bb4bf774c9ab")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ClearSafetyStatus-request>)))
  "Returns full string definition for message of type '<ClearSafetyStatus-request>"
  (cl:format cl:nil "SafetyHandle input~%~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ClearSafetyStatus-request)))
  "Returns full string definition for message of type 'ClearSafetyStatus-request"
  (cl:format cl:nil "SafetyHandle input~%~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ClearSafetyStatus-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ClearSafetyStatus-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ClearSafetyStatus-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude ClearSafetyStatus-response.msg.html

(cl:defclass <ClearSafetyStatus-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass ClearSafetyStatus-response (<ClearSafetyStatus-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ClearSafetyStatus-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ClearSafetyStatus-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ClearSafetyStatus-response> is deprecated: use kortex_driver-srv:ClearSafetyStatus-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <ClearSafetyStatus-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ClearSafetyStatus-response>) ostream)
  "Serializes a message object of type '<ClearSafetyStatus-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ClearSafetyStatus-response>) istream)
  "Deserializes a message object of type '<ClearSafetyStatus-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ClearSafetyStatus-response>)))
  "Returns string type for a service object of type '<ClearSafetyStatus-response>"
  "kortex_driver/ClearSafetyStatusResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ClearSafetyStatus-response)))
  "Returns string type for a service object of type 'ClearSafetyStatus-response"
  "kortex_driver/ClearSafetyStatusResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ClearSafetyStatus-response>)))
  "Returns md5sum for a message object of type '<ClearSafetyStatus-response>"
  "7960dab80443332660b3bb4bf774c9ab")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ClearSafetyStatus-response)))
  "Returns md5sum for a message object of type 'ClearSafetyStatus-response"
  "7960dab80443332660b3bb4bf774c9ab")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ClearSafetyStatus-response>)))
  "Returns full string definition for message of type '<ClearSafetyStatus-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ClearSafetyStatus-response)))
  "Returns full string definition for message of type 'ClearSafetyStatus-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ClearSafetyStatus-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ClearSafetyStatus-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ClearSafetyStatus-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ClearSafetyStatus)))
  'ClearSafetyStatus-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ClearSafetyStatus)))
  'ClearSafetyStatus-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ClearSafetyStatus)))
  "Returns string type for a service object of type '<ClearSafetyStatus>"
  "kortex_driver/ClearSafetyStatus")