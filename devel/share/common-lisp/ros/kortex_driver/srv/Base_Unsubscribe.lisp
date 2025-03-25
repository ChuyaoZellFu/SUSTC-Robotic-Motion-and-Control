; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude Base_Unsubscribe-request.msg.html

(cl:defclass <Base_Unsubscribe-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:NotificationHandle
    :initform (cl:make-instance 'kortex_driver-msg:NotificationHandle)))
)

(cl:defclass Base_Unsubscribe-request (<Base_Unsubscribe-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Base_Unsubscribe-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Base_Unsubscribe-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<Base_Unsubscribe-request> is deprecated: use kortex_driver-srv:Base_Unsubscribe-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <Base_Unsubscribe-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Base_Unsubscribe-request>) ostream)
  "Serializes a message object of type '<Base_Unsubscribe-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Base_Unsubscribe-request>) istream)
  "Deserializes a message object of type '<Base_Unsubscribe-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Base_Unsubscribe-request>)))
  "Returns string type for a service object of type '<Base_Unsubscribe-request>"
  "kortex_driver/Base_UnsubscribeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Base_Unsubscribe-request)))
  "Returns string type for a service object of type 'Base_Unsubscribe-request"
  "kortex_driver/Base_UnsubscribeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Base_Unsubscribe-request>)))
  "Returns md5sum for a message object of type '<Base_Unsubscribe-request>"
  "7960dab80443332660b3bb4bf774c9ab")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Base_Unsubscribe-request)))
  "Returns md5sum for a message object of type 'Base_Unsubscribe-request"
  "7960dab80443332660b3bb4bf774c9ab")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Base_Unsubscribe-request>)))
  "Returns full string definition for message of type '<Base_Unsubscribe-request>"
  (cl:format cl:nil "NotificationHandle input~%~%================================================================================~%MSG: kortex_driver/NotificationHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Base_Unsubscribe-request)))
  "Returns full string definition for message of type 'Base_Unsubscribe-request"
  (cl:format cl:nil "NotificationHandle input~%~%================================================================================~%MSG: kortex_driver/NotificationHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Base_Unsubscribe-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Base_Unsubscribe-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Base_Unsubscribe-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude Base_Unsubscribe-response.msg.html

(cl:defclass <Base_Unsubscribe-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass Base_Unsubscribe-response (<Base_Unsubscribe-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Base_Unsubscribe-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Base_Unsubscribe-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<Base_Unsubscribe-response> is deprecated: use kortex_driver-srv:Base_Unsubscribe-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <Base_Unsubscribe-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Base_Unsubscribe-response>) ostream)
  "Serializes a message object of type '<Base_Unsubscribe-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Base_Unsubscribe-response>) istream)
  "Deserializes a message object of type '<Base_Unsubscribe-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Base_Unsubscribe-response>)))
  "Returns string type for a service object of type '<Base_Unsubscribe-response>"
  "kortex_driver/Base_UnsubscribeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Base_Unsubscribe-response)))
  "Returns string type for a service object of type 'Base_Unsubscribe-response"
  "kortex_driver/Base_UnsubscribeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Base_Unsubscribe-response>)))
  "Returns md5sum for a message object of type '<Base_Unsubscribe-response>"
  "7960dab80443332660b3bb4bf774c9ab")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Base_Unsubscribe-response)))
  "Returns md5sum for a message object of type 'Base_Unsubscribe-response"
  "7960dab80443332660b3bb4bf774c9ab")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Base_Unsubscribe-response>)))
  "Returns full string definition for message of type '<Base_Unsubscribe-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Base_Unsubscribe-response)))
  "Returns full string definition for message of type 'Base_Unsubscribe-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Base_Unsubscribe-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Base_Unsubscribe-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Base_Unsubscribe-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Base_Unsubscribe)))
  'Base_Unsubscribe-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Base_Unsubscribe)))
  'Base_Unsubscribe-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Base_Unsubscribe)))
  "Returns string type for a service object of type '<Base_Unsubscribe>"
  "kortex_driver/Base_Unsubscribe")