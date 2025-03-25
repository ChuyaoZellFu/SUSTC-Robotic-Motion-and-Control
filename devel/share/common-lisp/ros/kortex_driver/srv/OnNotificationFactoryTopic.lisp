; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude OnNotificationFactoryTopic-request.msg.html

(cl:defclass <OnNotificationFactoryTopic-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:NotificationOptions
    :initform (cl:make-instance 'kortex_driver-msg:NotificationOptions)))
)

(cl:defclass OnNotificationFactoryTopic-request (<OnNotificationFactoryTopic-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OnNotificationFactoryTopic-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OnNotificationFactoryTopic-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<OnNotificationFactoryTopic-request> is deprecated: use kortex_driver-srv:OnNotificationFactoryTopic-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <OnNotificationFactoryTopic-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OnNotificationFactoryTopic-request>) ostream)
  "Serializes a message object of type '<OnNotificationFactoryTopic-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OnNotificationFactoryTopic-request>) istream)
  "Deserializes a message object of type '<OnNotificationFactoryTopic-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OnNotificationFactoryTopic-request>)))
  "Returns string type for a service object of type '<OnNotificationFactoryTopic-request>"
  "kortex_driver/OnNotificationFactoryTopicRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationFactoryTopic-request)))
  "Returns string type for a service object of type 'OnNotificationFactoryTopic-request"
  "kortex_driver/OnNotificationFactoryTopicRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OnNotificationFactoryTopic-request>)))
  "Returns md5sum for a message object of type '<OnNotificationFactoryTopic-request>"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OnNotificationFactoryTopic-request)))
  "Returns md5sum for a message object of type 'OnNotificationFactoryTopic-request"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OnNotificationFactoryTopic-request>)))
  "Returns full string definition for message of type '<OnNotificationFactoryTopic-request>"
  (cl:format cl:nil "NotificationOptions input~%~%================================================================================~%MSG: kortex_driver/NotificationOptions~%~%uint32 type~%uint32 rate_m_sec~%float32 threshold_value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OnNotificationFactoryTopic-request)))
  "Returns full string definition for message of type 'OnNotificationFactoryTopic-request"
  (cl:format cl:nil "NotificationOptions input~%~%================================================================================~%MSG: kortex_driver/NotificationOptions~%~%uint32 type~%uint32 rate_m_sec~%float32 threshold_value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OnNotificationFactoryTopic-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OnNotificationFactoryTopic-request>))
  "Converts a ROS message object to a list"
  (cl:list 'OnNotificationFactoryTopic-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude OnNotificationFactoryTopic-response.msg.html

(cl:defclass <OnNotificationFactoryTopic-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:NotificationHandle
    :initform (cl:make-instance 'kortex_driver-msg:NotificationHandle)))
)

(cl:defclass OnNotificationFactoryTopic-response (<OnNotificationFactoryTopic-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OnNotificationFactoryTopic-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OnNotificationFactoryTopic-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<OnNotificationFactoryTopic-response> is deprecated: use kortex_driver-srv:OnNotificationFactoryTopic-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <OnNotificationFactoryTopic-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OnNotificationFactoryTopic-response>) ostream)
  "Serializes a message object of type '<OnNotificationFactoryTopic-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OnNotificationFactoryTopic-response>) istream)
  "Deserializes a message object of type '<OnNotificationFactoryTopic-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OnNotificationFactoryTopic-response>)))
  "Returns string type for a service object of type '<OnNotificationFactoryTopic-response>"
  "kortex_driver/OnNotificationFactoryTopicResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationFactoryTopic-response)))
  "Returns string type for a service object of type 'OnNotificationFactoryTopic-response"
  "kortex_driver/OnNotificationFactoryTopicResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OnNotificationFactoryTopic-response>)))
  "Returns md5sum for a message object of type '<OnNotificationFactoryTopic-response>"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OnNotificationFactoryTopic-response)))
  "Returns md5sum for a message object of type 'OnNotificationFactoryTopic-response"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OnNotificationFactoryTopic-response>)))
  "Returns full string definition for message of type '<OnNotificationFactoryTopic-response>"
  (cl:format cl:nil "NotificationHandle output~%~%================================================================================~%MSG: kortex_driver/NotificationHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OnNotificationFactoryTopic-response)))
  "Returns full string definition for message of type 'OnNotificationFactoryTopic-response"
  (cl:format cl:nil "NotificationHandle output~%~%================================================================================~%MSG: kortex_driver/NotificationHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OnNotificationFactoryTopic-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OnNotificationFactoryTopic-response>))
  "Converts a ROS message object to a list"
  (cl:list 'OnNotificationFactoryTopic-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'OnNotificationFactoryTopic)))
  'OnNotificationFactoryTopic-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'OnNotificationFactoryTopic)))
  'OnNotificationFactoryTopic-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationFactoryTopic)))
  "Returns string type for a service object of type '<OnNotificationFactoryTopic>"
  "kortex_driver/OnNotificationFactoryTopic")