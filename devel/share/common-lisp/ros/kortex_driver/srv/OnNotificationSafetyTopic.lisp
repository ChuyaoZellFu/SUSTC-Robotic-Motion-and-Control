; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude OnNotificationSafetyTopic-request.msg.html

(cl:defclass <OnNotificationSafetyTopic-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:NotificationOptions
    :initform (cl:make-instance 'kortex_driver-msg:NotificationOptions)))
)

(cl:defclass OnNotificationSafetyTopic-request (<OnNotificationSafetyTopic-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OnNotificationSafetyTopic-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OnNotificationSafetyTopic-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<OnNotificationSafetyTopic-request> is deprecated: use kortex_driver-srv:OnNotificationSafetyTopic-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <OnNotificationSafetyTopic-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OnNotificationSafetyTopic-request>) ostream)
  "Serializes a message object of type '<OnNotificationSafetyTopic-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OnNotificationSafetyTopic-request>) istream)
  "Deserializes a message object of type '<OnNotificationSafetyTopic-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OnNotificationSafetyTopic-request>)))
  "Returns string type for a service object of type '<OnNotificationSafetyTopic-request>"
  "kortex_driver/OnNotificationSafetyTopicRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationSafetyTopic-request)))
  "Returns string type for a service object of type 'OnNotificationSafetyTopic-request"
  "kortex_driver/OnNotificationSafetyTopicRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OnNotificationSafetyTopic-request>)))
  "Returns md5sum for a message object of type '<OnNotificationSafetyTopic-request>"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OnNotificationSafetyTopic-request)))
  "Returns md5sum for a message object of type 'OnNotificationSafetyTopic-request"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OnNotificationSafetyTopic-request>)))
  "Returns full string definition for message of type '<OnNotificationSafetyTopic-request>"
  (cl:format cl:nil "NotificationOptions input~%~%================================================================================~%MSG: kortex_driver/NotificationOptions~%~%uint32 type~%uint32 rate_m_sec~%float32 threshold_value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OnNotificationSafetyTopic-request)))
  "Returns full string definition for message of type 'OnNotificationSafetyTopic-request"
  (cl:format cl:nil "NotificationOptions input~%~%================================================================================~%MSG: kortex_driver/NotificationOptions~%~%uint32 type~%uint32 rate_m_sec~%float32 threshold_value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OnNotificationSafetyTopic-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OnNotificationSafetyTopic-request>))
  "Converts a ROS message object to a list"
  (cl:list 'OnNotificationSafetyTopic-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude OnNotificationSafetyTopic-response.msg.html

(cl:defclass <OnNotificationSafetyTopic-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:NotificationHandle
    :initform (cl:make-instance 'kortex_driver-msg:NotificationHandle)))
)

(cl:defclass OnNotificationSafetyTopic-response (<OnNotificationSafetyTopic-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OnNotificationSafetyTopic-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OnNotificationSafetyTopic-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<OnNotificationSafetyTopic-response> is deprecated: use kortex_driver-srv:OnNotificationSafetyTopic-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <OnNotificationSafetyTopic-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OnNotificationSafetyTopic-response>) ostream)
  "Serializes a message object of type '<OnNotificationSafetyTopic-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OnNotificationSafetyTopic-response>) istream)
  "Deserializes a message object of type '<OnNotificationSafetyTopic-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OnNotificationSafetyTopic-response>)))
  "Returns string type for a service object of type '<OnNotificationSafetyTopic-response>"
  "kortex_driver/OnNotificationSafetyTopicResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationSafetyTopic-response)))
  "Returns string type for a service object of type 'OnNotificationSafetyTopic-response"
  "kortex_driver/OnNotificationSafetyTopicResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OnNotificationSafetyTopic-response>)))
  "Returns md5sum for a message object of type '<OnNotificationSafetyTopic-response>"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OnNotificationSafetyTopic-response)))
  "Returns md5sum for a message object of type 'OnNotificationSafetyTopic-response"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OnNotificationSafetyTopic-response>)))
  "Returns full string definition for message of type '<OnNotificationSafetyTopic-response>"
  (cl:format cl:nil "NotificationHandle output~%~%================================================================================~%MSG: kortex_driver/NotificationHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OnNotificationSafetyTopic-response)))
  "Returns full string definition for message of type 'OnNotificationSafetyTopic-response"
  (cl:format cl:nil "NotificationHandle output~%~%================================================================================~%MSG: kortex_driver/NotificationHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OnNotificationSafetyTopic-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OnNotificationSafetyTopic-response>))
  "Converts a ROS message object to a list"
  (cl:list 'OnNotificationSafetyTopic-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'OnNotificationSafetyTopic)))
  'OnNotificationSafetyTopic-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'OnNotificationSafetyTopic)))
  'OnNotificationSafetyTopic-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationSafetyTopic)))
  "Returns string type for a service object of type '<OnNotificationSafetyTopic>"
  "kortex_driver/OnNotificationSafetyTopic")