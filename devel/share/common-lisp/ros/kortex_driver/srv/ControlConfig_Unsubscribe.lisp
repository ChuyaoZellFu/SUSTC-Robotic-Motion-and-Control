; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude ControlConfig_Unsubscribe-request.msg.html

(cl:defclass <ControlConfig_Unsubscribe-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:NotificationHandle
    :initform (cl:make-instance 'kortex_driver-msg:NotificationHandle)))
)

(cl:defclass ControlConfig_Unsubscribe-request (<ControlConfig_Unsubscribe-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlConfig_Unsubscribe-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlConfig_Unsubscribe-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ControlConfig_Unsubscribe-request> is deprecated: use kortex_driver-srv:ControlConfig_Unsubscribe-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <ControlConfig_Unsubscribe-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlConfig_Unsubscribe-request>) ostream)
  "Serializes a message object of type '<ControlConfig_Unsubscribe-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlConfig_Unsubscribe-request>) istream)
  "Deserializes a message object of type '<ControlConfig_Unsubscribe-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlConfig_Unsubscribe-request>)))
  "Returns string type for a service object of type '<ControlConfig_Unsubscribe-request>"
  "kortex_driver/ControlConfig_UnsubscribeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlConfig_Unsubscribe-request)))
  "Returns string type for a service object of type 'ControlConfig_Unsubscribe-request"
  "kortex_driver/ControlConfig_UnsubscribeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlConfig_Unsubscribe-request>)))
  "Returns md5sum for a message object of type '<ControlConfig_Unsubscribe-request>"
  "7960dab80443332660b3bb4bf774c9ab")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlConfig_Unsubscribe-request)))
  "Returns md5sum for a message object of type 'ControlConfig_Unsubscribe-request"
  "7960dab80443332660b3bb4bf774c9ab")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlConfig_Unsubscribe-request>)))
  "Returns full string definition for message of type '<ControlConfig_Unsubscribe-request>"
  (cl:format cl:nil "NotificationHandle input~%~%================================================================================~%MSG: kortex_driver/NotificationHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlConfig_Unsubscribe-request)))
  "Returns full string definition for message of type 'ControlConfig_Unsubscribe-request"
  (cl:format cl:nil "NotificationHandle input~%~%================================================================================~%MSG: kortex_driver/NotificationHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlConfig_Unsubscribe-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlConfig_Unsubscribe-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlConfig_Unsubscribe-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude ControlConfig_Unsubscribe-response.msg.html

(cl:defclass <ControlConfig_Unsubscribe-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass ControlConfig_Unsubscribe-response (<ControlConfig_Unsubscribe-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlConfig_Unsubscribe-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlConfig_Unsubscribe-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ControlConfig_Unsubscribe-response> is deprecated: use kortex_driver-srv:ControlConfig_Unsubscribe-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <ControlConfig_Unsubscribe-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlConfig_Unsubscribe-response>) ostream)
  "Serializes a message object of type '<ControlConfig_Unsubscribe-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlConfig_Unsubscribe-response>) istream)
  "Deserializes a message object of type '<ControlConfig_Unsubscribe-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlConfig_Unsubscribe-response>)))
  "Returns string type for a service object of type '<ControlConfig_Unsubscribe-response>"
  "kortex_driver/ControlConfig_UnsubscribeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlConfig_Unsubscribe-response)))
  "Returns string type for a service object of type 'ControlConfig_Unsubscribe-response"
  "kortex_driver/ControlConfig_UnsubscribeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlConfig_Unsubscribe-response>)))
  "Returns md5sum for a message object of type '<ControlConfig_Unsubscribe-response>"
  "7960dab80443332660b3bb4bf774c9ab")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlConfig_Unsubscribe-response)))
  "Returns md5sum for a message object of type 'ControlConfig_Unsubscribe-response"
  "7960dab80443332660b3bb4bf774c9ab")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlConfig_Unsubscribe-response>)))
  "Returns full string definition for message of type '<ControlConfig_Unsubscribe-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlConfig_Unsubscribe-response)))
  "Returns full string definition for message of type 'ControlConfig_Unsubscribe-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlConfig_Unsubscribe-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlConfig_Unsubscribe-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlConfig_Unsubscribe-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ControlConfig_Unsubscribe)))
  'ControlConfig_Unsubscribe-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ControlConfig_Unsubscribe)))
  'ControlConfig_Unsubscribe-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlConfig_Unsubscribe)))
  "Returns string type for a service object of type '<ControlConfig_Unsubscribe>"
  "kortex_driver/ControlConfig_Unsubscribe")