; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude OnNotificationRobotEventTopic-request.msg.html

(cl:defclass <OnNotificationRobotEventTopic-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:NotificationOptions
    :initform (cl:make-instance 'kortex_driver-msg:NotificationOptions)))
)

(cl:defclass OnNotificationRobotEventTopic-request (<OnNotificationRobotEventTopic-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OnNotificationRobotEventTopic-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OnNotificationRobotEventTopic-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<OnNotificationRobotEventTopic-request> is deprecated: use kortex_driver-srv:OnNotificationRobotEventTopic-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <OnNotificationRobotEventTopic-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OnNotificationRobotEventTopic-request>) ostream)
  "Serializes a message object of type '<OnNotificationRobotEventTopic-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OnNotificationRobotEventTopic-request>) istream)
  "Deserializes a message object of type '<OnNotificationRobotEventTopic-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OnNotificationRobotEventTopic-request>)))
  "Returns string type for a service object of type '<OnNotificationRobotEventTopic-request>"
  "kortex_driver/OnNotificationRobotEventTopicRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationRobotEventTopic-request)))
  "Returns string type for a service object of type 'OnNotificationRobotEventTopic-request"
  "kortex_driver/OnNotificationRobotEventTopicRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OnNotificationRobotEventTopic-request>)))
  "Returns md5sum for a message object of type '<OnNotificationRobotEventTopic-request>"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OnNotificationRobotEventTopic-request)))
  "Returns md5sum for a message object of type 'OnNotificationRobotEventTopic-request"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OnNotificationRobotEventTopic-request>)))
  "Returns full string definition for message of type '<OnNotificationRobotEventTopic-request>"
  (cl:format cl:nil "NotificationOptions input~%~%================================================================================~%MSG: kortex_driver/NotificationOptions~%~%uint32 type~%uint32 rate_m_sec~%float32 threshold_value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OnNotificationRobotEventTopic-request)))
  "Returns full string definition for message of type 'OnNotificationRobotEventTopic-request"
  (cl:format cl:nil "NotificationOptions input~%~%================================================================================~%MSG: kortex_driver/NotificationOptions~%~%uint32 type~%uint32 rate_m_sec~%float32 threshold_value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OnNotificationRobotEventTopic-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OnNotificationRobotEventTopic-request>))
  "Converts a ROS message object to a list"
  (cl:list 'OnNotificationRobotEventTopic-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude OnNotificationRobotEventTopic-response.msg.html

(cl:defclass <OnNotificationRobotEventTopic-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:NotificationHandle
    :initform (cl:make-instance 'kortex_driver-msg:NotificationHandle)))
)

(cl:defclass OnNotificationRobotEventTopic-response (<OnNotificationRobotEventTopic-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OnNotificationRobotEventTopic-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OnNotificationRobotEventTopic-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<OnNotificationRobotEventTopic-response> is deprecated: use kortex_driver-srv:OnNotificationRobotEventTopic-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <OnNotificationRobotEventTopic-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OnNotificationRobotEventTopic-response>) ostream)
  "Serializes a message object of type '<OnNotificationRobotEventTopic-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OnNotificationRobotEventTopic-response>) istream)
  "Deserializes a message object of type '<OnNotificationRobotEventTopic-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OnNotificationRobotEventTopic-response>)))
  "Returns string type for a service object of type '<OnNotificationRobotEventTopic-response>"
  "kortex_driver/OnNotificationRobotEventTopicResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationRobotEventTopic-response)))
  "Returns string type for a service object of type 'OnNotificationRobotEventTopic-response"
  "kortex_driver/OnNotificationRobotEventTopicResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OnNotificationRobotEventTopic-response>)))
  "Returns md5sum for a message object of type '<OnNotificationRobotEventTopic-response>"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OnNotificationRobotEventTopic-response)))
  "Returns md5sum for a message object of type 'OnNotificationRobotEventTopic-response"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OnNotificationRobotEventTopic-response>)))
  "Returns full string definition for message of type '<OnNotificationRobotEventTopic-response>"
  (cl:format cl:nil "NotificationHandle output~%~%================================================================================~%MSG: kortex_driver/NotificationHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OnNotificationRobotEventTopic-response)))
  "Returns full string definition for message of type 'OnNotificationRobotEventTopic-response"
  (cl:format cl:nil "NotificationHandle output~%~%================================================================================~%MSG: kortex_driver/NotificationHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OnNotificationRobotEventTopic-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OnNotificationRobotEventTopic-response>))
  "Converts a ROS message object to a list"
  (cl:list 'OnNotificationRobotEventTopic-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'OnNotificationRobotEventTopic)))
  'OnNotificationRobotEventTopic-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'OnNotificationRobotEventTopic)))
  'OnNotificationRobotEventTopic-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationRobotEventTopic)))
  "Returns string type for a service object of type '<OnNotificationRobotEventTopic>"
  "kortex_driver/OnNotificationRobotEventTopic")