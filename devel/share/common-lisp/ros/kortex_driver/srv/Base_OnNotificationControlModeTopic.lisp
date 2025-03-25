; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude Base_OnNotificationControlModeTopic-request.msg.html

(cl:defclass <Base_OnNotificationControlModeTopic-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:NotificationOptions
    :initform (cl:make-instance 'kortex_driver-msg:NotificationOptions)))
)

(cl:defclass Base_OnNotificationControlModeTopic-request (<Base_OnNotificationControlModeTopic-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Base_OnNotificationControlModeTopic-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Base_OnNotificationControlModeTopic-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<Base_OnNotificationControlModeTopic-request> is deprecated: use kortex_driver-srv:Base_OnNotificationControlModeTopic-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <Base_OnNotificationControlModeTopic-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Base_OnNotificationControlModeTopic-request>) ostream)
  "Serializes a message object of type '<Base_OnNotificationControlModeTopic-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Base_OnNotificationControlModeTopic-request>) istream)
  "Deserializes a message object of type '<Base_OnNotificationControlModeTopic-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Base_OnNotificationControlModeTopic-request>)))
  "Returns string type for a service object of type '<Base_OnNotificationControlModeTopic-request>"
  "kortex_driver/Base_OnNotificationControlModeTopicRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Base_OnNotificationControlModeTopic-request)))
  "Returns string type for a service object of type 'Base_OnNotificationControlModeTopic-request"
  "kortex_driver/Base_OnNotificationControlModeTopicRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Base_OnNotificationControlModeTopic-request>)))
  "Returns md5sum for a message object of type '<Base_OnNotificationControlModeTopic-request>"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Base_OnNotificationControlModeTopic-request)))
  "Returns md5sum for a message object of type 'Base_OnNotificationControlModeTopic-request"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Base_OnNotificationControlModeTopic-request>)))
  "Returns full string definition for message of type '<Base_OnNotificationControlModeTopic-request>"
  (cl:format cl:nil "NotificationOptions input~%~%================================================================================~%MSG: kortex_driver/NotificationOptions~%~%uint32 type~%uint32 rate_m_sec~%float32 threshold_value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Base_OnNotificationControlModeTopic-request)))
  "Returns full string definition for message of type 'Base_OnNotificationControlModeTopic-request"
  (cl:format cl:nil "NotificationOptions input~%~%================================================================================~%MSG: kortex_driver/NotificationOptions~%~%uint32 type~%uint32 rate_m_sec~%float32 threshold_value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Base_OnNotificationControlModeTopic-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Base_OnNotificationControlModeTopic-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Base_OnNotificationControlModeTopic-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude Base_OnNotificationControlModeTopic-response.msg.html

(cl:defclass <Base_OnNotificationControlModeTopic-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:NotificationHandle
    :initform (cl:make-instance 'kortex_driver-msg:NotificationHandle)))
)

(cl:defclass Base_OnNotificationControlModeTopic-response (<Base_OnNotificationControlModeTopic-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Base_OnNotificationControlModeTopic-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Base_OnNotificationControlModeTopic-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<Base_OnNotificationControlModeTopic-response> is deprecated: use kortex_driver-srv:Base_OnNotificationControlModeTopic-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <Base_OnNotificationControlModeTopic-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Base_OnNotificationControlModeTopic-response>) ostream)
  "Serializes a message object of type '<Base_OnNotificationControlModeTopic-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Base_OnNotificationControlModeTopic-response>) istream)
  "Deserializes a message object of type '<Base_OnNotificationControlModeTopic-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Base_OnNotificationControlModeTopic-response>)))
  "Returns string type for a service object of type '<Base_OnNotificationControlModeTopic-response>"
  "kortex_driver/Base_OnNotificationControlModeTopicResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Base_OnNotificationControlModeTopic-response)))
  "Returns string type for a service object of type 'Base_OnNotificationControlModeTopic-response"
  "kortex_driver/Base_OnNotificationControlModeTopicResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Base_OnNotificationControlModeTopic-response>)))
  "Returns md5sum for a message object of type '<Base_OnNotificationControlModeTopic-response>"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Base_OnNotificationControlModeTopic-response)))
  "Returns md5sum for a message object of type 'Base_OnNotificationControlModeTopic-response"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Base_OnNotificationControlModeTopic-response>)))
  "Returns full string definition for message of type '<Base_OnNotificationControlModeTopic-response>"
  (cl:format cl:nil "NotificationHandle output~%~%================================================================================~%MSG: kortex_driver/NotificationHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Base_OnNotificationControlModeTopic-response)))
  "Returns full string definition for message of type 'Base_OnNotificationControlModeTopic-response"
  (cl:format cl:nil "NotificationHandle output~%~%================================================================================~%MSG: kortex_driver/NotificationHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Base_OnNotificationControlModeTopic-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Base_OnNotificationControlModeTopic-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Base_OnNotificationControlModeTopic-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Base_OnNotificationControlModeTopic)))
  'Base_OnNotificationControlModeTopic-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Base_OnNotificationControlModeTopic)))
  'Base_OnNotificationControlModeTopic-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Base_OnNotificationControlModeTopic)))
  "Returns string type for a service object of type '<Base_OnNotificationControlModeTopic>"
  "kortex_driver/Base_OnNotificationControlModeTopic")