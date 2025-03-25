; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude OnNotificationUserTopic-request.msg.html

(cl:defclass <OnNotificationUserTopic-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:NotificationOptions
    :initform (cl:make-instance 'kortex_driver-msg:NotificationOptions)))
)

(cl:defclass OnNotificationUserTopic-request (<OnNotificationUserTopic-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OnNotificationUserTopic-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OnNotificationUserTopic-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<OnNotificationUserTopic-request> is deprecated: use kortex_driver-srv:OnNotificationUserTopic-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <OnNotificationUserTopic-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OnNotificationUserTopic-request>) ostream)
  "Serializes a message object of type '<OnNotificationUserTopic-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OnNotificationUserTopic-request>) istream)
  "Deserializes a message object of type '<OnNotificationUserTopic-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OnNotificationUserTopic-request>)))
  "Returns string type for a service object of type '<OnNotificationUserTopic-request>"
  "kortex_driver/OnNotificationUserTopicRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationUserTopic-request)))
  "Returns string type for a service object of type 'OnNotificationUserTopic-request"
  "kortex_driver/OnNotificationUserTopicRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OnNotificationUserTopic-request>)))
  "Returns md5sum for a message object of type '<OnNotificationUserTopic-request>"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OnNotificationUserTopic-request)))
  "Returns md5sum for a message object of type 'OnNotificationUserTopic-request"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OnNotificationUserTopic-request>)))
  "Returns full string definition for message of type '<OnNotificationUserTopic-request>"
  (cl:format cl:nil "NotificationOptions input~%~%================================================================================~%MSG: kortex_driver/NotificationOptions~%~%uint32 type~%uint32 rate_m_sec~%float32 threshold_value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OnNotificationUserTopic-request)))
  "Returns full string definition for message of type 'OnNotificationUserTopic-request"
  (cl:format cl:nil "NotificationOptions input~%~%================================================================================~%MSG: kortex_driver/NotificationOptions~%~%uint32 type~%uint32 rate_m_sec~%float32 threshold_value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OnNotificationUserTopic-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OnNotificationUserTopic-request>))
  "Converts a ROS message object to a list"
  (cl:list 'OnNotificationUserTopic-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude OnNotificationUserTopic-response.msg.html

(cl:defclass <OnNotificationUserTopic-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:NotificationHandle
    :initform (cl:make-instance 'kortex_driver-msg:NotificationHandle)))
)

(cl:defclass OnNotificationUserTopic-response (<OnNotificationUserTopic-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OnNotificationUserTopic-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OnNotificationUserTopic-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<OnNotificationUserTopic-response> is deprecated: use kortex_driver-srv:OnNotificationUserTopic-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <OnNotificationUserTopic-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OnNotificationUserTopic-response>) ostream)
  "Serializes a message object of type '<OnNotificationUserTopic-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OnNotificationUserTopic-response>) istream)
  "Deserializes a message object of type '<OnNotificationUserTopic-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OnNotificationUserTopic-response>)))
  "Returns string type for a service object of type '<OnNotificationUserTopic-response>"
  "kortex_driver/OnNotificationUserTopicResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationUserTopic-response)))
  "Returns string type for a service object of type 'OnNotificationUserTopic-response"
  "kortex_driver/OnNotificationUserTopicResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OnNotificationUserTopic-response>)))
  "Returns md5sum for a message object of type '<OnNotificationUserTopic-response>"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OnNotificationUserTopic-response)))
  "Returns md5sum for a message object of type 'OnNotificationUserTopic-response"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OnNotificationUserTopic-response>)))
  "Returns full string definition for message of type '<OnNotificationUserTopic-response>"
  (cl:format cl:nil "NotificationHandle output~%~%================================================================================~%MSG: kortex_driver/NotificationHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OnNotificationUserTopic-response)))
  "Returns full string definition for message of type 'OnNotificationUserTopic-response"
  (cl:format cl:nil "NotificationHandle output~%~%================================================================================~%MSG: kortex_driver/NotificationHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OnNotificationUserTopic-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OnNotificationUserTopic-response>))
  "Converts a ROS message object to a list"
  (cl:list 'OnNotificationUserTopic-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'OnNotificationUserTopic)))
  'OnNotificationUserTopic-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'OnNotificationUserTopic)))
  'OnNotificationUserTopic-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationUserTopic)))
  "Returns string type for a service object of type '<OnNotificationUserTopic>"
  "kortex_driver/OnNotificationUserTopic")