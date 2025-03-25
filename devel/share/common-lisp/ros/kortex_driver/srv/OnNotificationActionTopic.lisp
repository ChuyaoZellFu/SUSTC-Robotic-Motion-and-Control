; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude OnNotificationActionTopic-request.msg.html

(cl:defclass <OnNotificationActionTopic-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:NotificationOptions
    :initform (cl:make-instance 'kortex_driver-msg:NotificationOptions)))
)

(cl:defclass OnNotificationActionTopic-request (<OnNotificationActionTopic-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OnNotificationActionTopic-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OnNotificationActionTopic-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<OnNotificationActionTopic-request> is deprecated: use kortex_driver-srv:OnNotificationActionTopic-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <OnNotificationActionTopic-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OnNotificationActionTopic-request>) ostream)
  "Serializes a message object of type '<OnNotificationActionTopic-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OnNotificationActionTopic-request>) istream)
  "Deserializes a message object of type '<OnNotificationActionTopic-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OnNotificationActionTopic-request>)))
  "Returns string type for a service object of type '<OnNotificationActionTopic-request>"
  "kortex_driver/OnNotificationActionTopicRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationActionTopic-request)))
  "Returns string type for a service object of type 'OnNotificationActionTopic-request"
  "kortex_driver/OnNotificationActionTopicRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OnNotificationActionTopic-request>)))
  "Returns md5sum for a message object of type '<OnNotificationActionTopic-request>"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OnNotificationActionTopic-request)))
  "Returns md5sum for a message object of type 'OnNotificationActionTopic-request"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OnNotificationActionTopic-request>)))
  "Returns full string definition for message of type '<OnNotificationActionTopic-request>"
  (cl:format cl:nil "NotificationOptions input~%~%================================================================================~%MSG: kortex_driver/NotificationOptions~%~%uint32 type~%uint32 rate_m_sec~%float32 threshold_value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OnNotificationActionTopic-request)))
  "Returns full string definition for message of type 'OnNotificationActionTopic-request"
  (cl:format cl:nil "NotificationOptions input~%~%================================================================================~%MSG: kortex_driver/NotificationOptions~%~%uint32 type~%uint32 rate_m_sec~%float32 threshold_value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OnNotificationActionTopic-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OnNotificationActionTopic-request>))
  "Converts a ROS message object to a list"
  (cl:list 'OnNotificationActionTopic-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude OnNotificationActionTopic-response.msg.html

(cl:defclass <OnNotificationActionTopic-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:NotificationHandle
    :initform (cl:make-instance 'kortex_driver-msg:NotificationHandle)))
)

(cl:defclass OnNotificationActionTopic-response (<OnNotificationActionTopic-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OnNotificationActionTopic-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OnNotificationActionTopic-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<OnNotificationActionTopic-response> is deprecated: use kortex_driver-srv:OnNotificationActionTopic-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <OnNotificationActionTopic-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OnNotificationActionTopic-response>) ostream)
  "Serializes a message object of type '<OnNotificationActionTopic-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OnNotificationActionTopic-response>) istream)
  "Deserializes a message object of type '<OnNotificationActionTopic-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OnNotificationActionTopic-response>)))
  "Returns string type for a service object of type '<OnNotificationActionTopic-response>"
  "kortex_driver/OnNotificationActionTopicResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationActionTopic-response)))
  "Returns string type for a service object of type 'OnNotificationActionTopic-response"
  "kortex_driver/OnNotificationActionTopicResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OnNotificationActionTopic-response>)))
  "Returns md5sum for a message object of type '<OnNotificationActionTopic-response>"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OnNotificationActionTopic-response)))
  "Returns md5sum for a message object of type 'OnNotificationActionTopic-response"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OnNotificationActionTopic-response>)))
  "Returns full string definition for message of type '<OnNotificationActionTopic-response>"
  (cl:format cl:nil "NotificationHandle output~%~%================================================================================~%MSG: kortex_driver/NotificationHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OnNotificationActionTopic-response)))
  "Returns full string definition for message of type 'OnNotificationActionTopic-response"
  (cl:format cl:nil "NotificationHandle output~%~%================================================================================~%MSG: kortex_driver/NotificationHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OnNotificationActionTopic-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OnNotificationActionTopic-response>))
  "Converts a ROS message object to a list"
  (cl:list 'OnNotificationActionTopic-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'OnNotificationActionTopic)))
  'OnNotificationActionTopic-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'OnNotificationActionTopic)))
  'OnNotificationActionTopic-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationActionTopic)))
  "Returns string type for a service object of type '<OnNotificationActionTopic>"
  "kortex_driver/OnNotificationActionTopic")