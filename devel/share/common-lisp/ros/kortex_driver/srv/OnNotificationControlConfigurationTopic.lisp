; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude OnNotificationControlConfigurationTopic-request.msg.html

(cl:defclass <OnNotificationControlConfigurationTopic-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:NotificationOptions
    :initform (cl:make-instance 'kortex_driver-msg:NotificationOptions)))
)

(cl:defclass OnNotificationControlConfigurationTopic-request (<OnNotificationControlConfigurationTopic-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OnNotificationControlConfigurationTopic-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OnNotificationControlConfigurationTopic-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<OnNotificationControlConfigurationTopic-request> is deprecated: use kortex_driver-srv:OnNotificationControlConfigurationTopic-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <OnNotificationControlConfigurationTopic-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OnNotificationControlConfigurationTopic-request>) ostream)
  "Serializes a message object of type '<OnNotificationControlConfigurationTopic-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OnNotificationControlConfigurationTopic-request>) istream)
  "Deserializes a message object of type '<OnNotificationControlConfigurationTopic-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OnNotificationControlConfigurationTopic-request>)))
  "Returns string type for a service object of type '<OnNotificationControlConfigurationTopic-request>"
  "kortex_driver/OnNotificationControlConfigurationTopicRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationControlConfigurationTopic-request)))
  "Returns string type for a service object of type 'OnNotificationControlConfigurationTopic-request"
  "kortex_driver/OnNotificationControlConfigurationTopicRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OnNotificationControlConfigurationTopic-request>)))
  "Returns md5sum for a message object of type '<OnNotificationControlConfigurationTopic-request>"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OnNotificationControlConfigurationTopic-request)))
  "Returns md5sum for a message object of type 'OnNotificationControlConfigurationTopic-request"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OnNotificationControlConfigurationTopic-request>)))
  "Returns full string definition for message of type '<OnNotificationControlConfigurationTopic-request>"
  (cl:format cl:nil "NotificationOptions input~%~%================================================================================~%MSG: kortex_driver/NotificationOptions~%~%uint32 type~%uint32 rate_m_sec~%float32 threshold_value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OnNotificationControlConfigurationTopic-request)))
  "Returns full string definition for message of type 'OnNotificationControlConfigurationTopic-request"
  (cl:format cl:nil "NotificationOptions input~%~%================================================================================~%MSG: kortex_driver/NotificationOptions~%~%uint32 type~%uint32 rate_m_sec~%float32 threshold_value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OnNotificationControlConfigurationTopic-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OnNotificationControlConfigurationTopic-request>))
  "Converts a ROS message object to a list"
  (cl:list 'OnNotificationControlConfigurationTopic-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude OnNotificationControlConfigurationTopic-response.msg.html

(cl:defclass <OnNotificationControlConfigurationTopic-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:NotificationHandle
    :initform (cl:make-instance 'kortex_driver-msg:NotificationHandle)))
)

(cl:defclass OnNotificationControlConfigurationTopic-response (<OnNotificationControlConfigurationTopic-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OnNotificationControlConfigurationTopic-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OnNotificationControlConfigurationTopic-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<OnNotificationControlConfigurationTopic-response> is deprecated: use kortex_driver-srv:OnNotificationControlConfigurationTopic-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <OnNotificationControlConfigurationTopic-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OnNotificationControlConfigurationTopic-response>) ostream)
  "Serializes a message object of type '<OnNotificationControlConfigurationTopic-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OnNotificationControlConfigurationTopic-response>) istream)
  "Deserializes a message object of type '<OnNotificationControlConfigurationTopic-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OnNotificationControlConfigurationTopic-response>)))
  "Returns string type for a service object of type '<OnNotificationControlConfigurationTopic-response>"
  "kortex_driver/OnNotificationControlConfigurationTopicResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationControlConfigurationTopic-response)))
  "Returns string type for a service object of type 'OnNotificationControlConfigurationTopic-response"
  "kortex_driver/OnNotificationControlConfigurationTopicResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OnNotificationControlConfigurationTopic-response>)))
  "Returns md5sum for a message object of type '<OnNotificationControlConfigurationTopic-response>"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OnNotificationControlConfigurationTopic-response)))
  "Returns md5sum for a message object of type 'OnNotificationControlConfigurationTopic-response"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OnNotificationControlConfigurationTopic-response>)))
  "Returns full string definition for message of type '<OnNotificationControlConfigurationTopic-response>"
  (cl:format cl:nil "NotificationHandle output~%~%================================================================================~%MSG: kortex_driver/NotificationHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OnNotificationControlConfigurationTopic-response)))
  "Returns full string definition for message of type 'OnNotificationControlConfigurationTopic-response"
  (cl:format cl:nil "NotificationHandle output~%~%================================================================================~%MSG: kortex_driver/NotificationHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OnNotificationControlConfigurationTopic-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OnNotificationControlConfigurationTopic-response>))
  "Converts a ROS message object to a list"
  (cl:list 'OnNotificationControlConfigurationTopic-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'OnNotificationControlConfigurationTopic)))
  'OnNotificationControlConfigurationTopic-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'OnNotificationControlConfigurationTopic)))
  'OnNotificationControlConfigurationTopic-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationControlConfigurationTopic)))
  "Returns string type for a service object of type '<OnNotificationControlConfigurationTopic>"
  "kortex_driver/OnNotificationControlConfigurationTopic")