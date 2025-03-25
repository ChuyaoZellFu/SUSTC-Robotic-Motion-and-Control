; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude OnNotificationConfigurationChangeTopic-request.msg.html

(cl:defclass <OnNotificationConfigurationChangeTopic-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:NotificationOptions
    :initform (cl:make-instance 'kortex_driver-msg:NotificationOptions)))
)

(cl:defclass OnNotificationConfigurationChangeTopic-request (<OnNotificationConfigurationChangeTopic-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OnNotificationConfigurationChangeTopic-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OnNotificationConfigurationChangeTopic-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<OnNotificationConfigurationChangeTopic-request> is deprecated: use kortex_driver-srv:OnNotificationConfigurationChangeTopic-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <OnNotificationConfigurationChangeTopic-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OnNotificationConfigurationChangeTopic-request>) ostream)
  "Serializes a message object of type '<OnNotificationConfigurationChangeTopic-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OnNotificationConfigurationChangeTopic-request>) istream)
  "Deserializes a message object of type '<OnNotificationConfigurationChangeTopic-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OnNotificationConfigurationChangeTopic-request>)))
  "Returns string type for a service object of type '<OnNotificationConfigurationChangeTopic-request>"
  "kortex_driver/OnNotificationConfigurationChangeTopicRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationConfigurationChangeTopic-request)))
  "Returns string type for a service object of type 'OnNotificationConfigurationChangeTopic-request"
  "kortex_driver/OnNotificationConfigurationChangeTopicRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OnNotificationConfigurationChangeTopic-request>)))
  "Returns md5sum for a message object of type '<OnNotificationConfigurationChangeTopic-request>"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OnNotificationConfigurationChangeTopic-request)))
  "Returns md5sum for a message object of type 'OnNotificationConfigurationChangeTopic-request"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OnNotificationConfigurationChangeTopic-request>)))
  "Returns full string definition for message of type '<OnNotificationConfigurationChangeTopic-request>"
  (cl:format cl:nil "NotificationOptions input~%~%================================================================================~%MSG: kortex_driver/NotificationOptions~%~%uint32 type~%uint32 rate_m_sec~%float32 threshold_value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OnNotificationConfigurationChangeTopic-request)))
  "Returns full string definition for message of type 'OnNotificationConfigurationChangeTopic-request"
  (cl:format cl:nil "NotificationOptions input~%~%================================================================================~%MSG: kortex_driver/NotificationOptions~%~%uint32 type~%uint32 rate_m_sec~%float32 threshold_value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OnNotificationConfigurationChangeTopic-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OnNotificationConfigurationChangeTopic-request>))
  "Converts a ROS message object to a list"
  (cl:list 'OnNotificationConfigurationChangeTopic-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude OnNotificationConfigurationChangeTopic-response.msg.html

(cl:defclass <OnNotificationConfigurationChangeTopic-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:NotificationHandle
    :initform (cl:make-instance 'kortex_driver-msg:NotificationHandle)))
)

(cl:defclass OnNotificationConfigurationChangeTopic-response (<OnNotificationConfigurationChangeTopic-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OnNotificationConfigurationChangeTopic-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OnNotificationConfigurationChangeTopic-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<OnNotificationConfigurationChangeTopic-response> is deprecated: use kortex_driver-srv:OnNotificationConfigurationChangeTopic-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <OnNotificationConfigurationChangeTopic-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OnNotificationConfigurationChangeTopic-response>) ostream)
  "Serializes a message object of type '<OnNotificationConfigurationChangeTopic-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OnNotificationConfigurationChangeTopic-response>) istream)
  "Deserializes a message object of type '<OnNotificationConfigurationChangeTopic-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OnNotificationConfigurationChangeTopic-response>)))
  "Returns string type for a service object of type '<OnNotificationConfigurationChangeTopic-response>"
  "kortex_driver/OnNotificationConfigurationChangeTopicResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationConfigurationChangeTopic-response)))
  "Returns string type for a service object of type 'OnNotificationConfigurationChangeTopic-response"
  "kortex_driver/OnNotificationConfigurationChangeTopicResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OnNotificationConfigurationChangeTopic-response>)))
  "Returns md5sum for a message object of type '<OnNotificationConfigurationChangeTopic-response>"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OnNotificationConfigurationChangeTopic-response)))
  "Returns md5sum for a message object of type 'OnNotificationConfigurationChangeTopic-response"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OnNotificationConfigurationChangeTopic-response>)))
  "Returns full string definition for message of type '<OnNotificationConfigurationChangeTopic-response>"
  (cl:format cl:nil "NotificationHandle output~%~%================================================================================~%MSG: kortex_driver/NotificationHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OnNotificationConfigurationChangeTopic-response)))
  "Returns full string definition for message of type 'OnNotificationConfigurationChangeTopic-response"
  (cl:format cl:nil "NotificationHandle output~%~%================================================================================~%MSG: kortex_driver/NotificationHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OnNotificationConfigurationChangeTopic-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OnNotificationConfigurationChangeTopic-response>))
  "Converts a ROS message object to a list"
  (cl:list 'OnNotificationConfigurationChangeTopic-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'OnNotificationConfigurationChangeTopic)))
  'OnNotificationConfigurationChangeTopic-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'OnNotificationConfigurationChangeTopic)))
  'OnNotificationConfigurationChangeTopic-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationConfigurationChangeTopic)))
  "Returns string type for a service object of type '<OnNotificationConfigurationChangeTopic>"
  "kortex_driver/OnNotificationConfigurationChangeTopic")