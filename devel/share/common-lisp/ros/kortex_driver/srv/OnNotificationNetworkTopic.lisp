; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude OnNotificationNetworkTopic-request.msg.html

(cl:defclass <OnNotificationNetworkTopic-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:NotificationOptions
    :initform (cl:make-instance 'kortex_driver-msg:NotificationOptions)))
)

(cl:defclass OnNotificationNetworkTopic-request (<OnNotificationNetworkTopic-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OnNotificationNetworkTopic-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OnNotificationNetworkTopic-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<OnNotificationNetworkTopic-request> is deprecated: use kortex_driver-srv:OnNotificationNetworkTopic-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <OnNotificationNetworkTopic-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OnNotificationNetworkTopic-request>) ostream)
  "Serializes a message object of type '<OnNotificationNetworkTopic-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OnNotificationNetworkTopic-request>) istream)
  "Deserializes a message object of type '<OnNotificationNetworkTopic-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OnNotificationNetworkTopic-request>)))
  "Returns string type for a service object of type '<OnNotificationNetworkTopic-request>"
  "kortex_driver/OnNotificationNetworkTopicRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationNetworkTopic-request)))
  "Returns string type for a service object of type 'OnNotificationNetworkTopic-request"
  "kortex_driver/OnNotificationNetworkTopicRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OnNotificationNetworkTopic-request>)))
  "Returns md5sum for a message object of type '<OnNotificationNetworkTopic-request>"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OnNotificationNetworkTopic-request)))
  "Returns md5sum for a message object of type 'OnNotificationNetworkTopic-request"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OnNotificationNetworkTopic-request>)))
  "Returns full string definition for message of type '<OnNotificationNetworkTopic-request>"
  (cl:format cl:nil "NotificationOptions input~%~%================================================================================~%MSG: kortex_driver/NotificationOptions~%~%uint32 type~%uint32 rate_m_sec~%float32 threshold_value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OnNotificationNetworkTopic-request)))
  "Returns full string definition for message of type 'OnNotificationNetworkTopic-request"
  (cl:format cl:nil "NotificationOptions input~%~%================================================================================~%MSG: kortex_driver/NotificationOptions~%~%uint32 type~%uint32 rate_m_sec~%float32 threshold_value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OnNotificationNetworkTopic-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OnNotificationNetworkTopic-request>))
  "Converts a ROS message object to a list"
  (cl:list 'OnNotificationNetworkTopic-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude OnNotificationNetworkTopic-response.msg.html

(cl:defclass <OnNotificationNetworkTopic-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:NotificationHandle
    :initform (cl:make-instance 'kortex_driver-msg:NotificationHandle)))
)

(cl:defclass OnNotificationNetworkTopic-response (<OnNotificationNetworkTopic-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OnNotificationNetworkTopic-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OnNotificationNetworkTopic-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<OnNotificationNetworkTopic-response> is deprecated: use kortex_driver-srv:OnNotificationNetworkTopic-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <OnNotificationNetworkTopic-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OnNotificationNetworkTopic-response>) ostream)
  "Serializes a message object of type '<OnNotificationNetworkTopic-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OnNotificationNetworkTopic-response>) istream)
  "Deserializes a message object of type '<OnNotificationNetworkTopic-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OnNotificationNetworkTopic-response>)))
  "Returns string type for a service object of type '<OnNotificationNetworkTopic-response>"
  "kortex_driver/OnNotificationNetworkTopicResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationNetworkTopic-response)))
  "Returns string type for a service object of type 'OnNotificationNetworkTopic-response"
  "kortex_driver/OnNotificationNetworkTopicResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OnNotificationNetworkTopic-response>)))
  "Returns md5sum for a message object of type '<OnNotificationNetworkTopic-response>"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OnNotificationNetworkTopic-response)))
  "Returns md5sum for a message object of type 'OnNotificationNetworkTopic-response"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OnNotificationNetworkTopic-response>)))
  "Returns full string definition for message of type '<OnNotificationNetworkTopic-response>"
  (cl:format cl:nil "NotificationHandle output~%~%================================================================================~%MSG: kortex_driver/NotificationHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OnNotificationNetworkTopic-response)))
  "Returns full string definition for message of type 'OnNotificationNetworkTopic-response"
  (cl:format cl:nil "NotificationHandle output~%~%================================================================================~%MSG: kortex_driver/NotificationHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OnNotificationNetworkTopic-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OnNotificationNetworkTopic-response>))
  "Converts a ROS message object to a list"
  (cl:list 'OnNotificationNetworkTopic-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'OnNotificationNetworkTopic)))
  'OnNotificationNetworkTopic-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'OnNotificationNetworkTopic)))
  'OnNotificationNetworkTopic-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationNetworkTopic)))
  "Returns string type for a service object of type '<OnNotificationNetworkTopic>"
  "kortex_driver/OnNotificationNetworkTopic")