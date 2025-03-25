; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude OnNotificationServoingModeTopic-request.msg.html

(cl:defclass <OnNotificationServoingModeTopic-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:NotificationOptions
    :initform (cl:make-instance 'kortex_driver-msg:NotificationOptions)))
)

(cl:defclass OnNotificationServoingModeTopic-request (<OnNotificationServoingModeTopic-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OnNotificationServoingModeTopic-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OnNotificationServoingModeTopic-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<OnNotificationServoingModeTopic-request> is deprecated: use kortex_driver-srv:OnNotificationServoingModeTopic-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <OnNotificationServoingModeTopic-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OnNotificationServoingModeTopic-request>) ostream)
  "Serializes a message object of type '<OnNotificationServoingModeTopic-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OnNotificationServoingModeTopic-request>) istream)
  "Deserializes a message object of type '<OnNotificationServoingModeTopic-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OnNotificationServoingModeTopic-request>)))
  "Returns string type for a service object of type '<OnNotificationServoingModeTopic-request>"
  "kortex_driver/OnNotificationServoingModeTopicRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationServoingModeTopic-request)))
  "Returns string type for a service object of type 'OnNotificationServoingModeTopic-request"
  "kortex_driver/OnNotificationServoingModeTopicRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OnNotificationServoingModeTopic-request>)))
  "Returns md5sum for a message object of type '<OnNotificationServoingModeTopic-request>"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OnNotificationServoingModeTopic-request)))
  "Returns md5sum for a message object of type 'OnNotificationServoingModeTopic-request"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OnNotificationServoingModeTopic-request>)))
  "Returns full string definition for message of type '<OnNotificationServoingModeTopic-request>"
  (cl:format cl:nil "NotificationOptions input~%~%================================================================================~%MSG: kortex_driver/NotificationOptions~%~%uint32 type~%uint32 rate_m_sec~%float32 threshold_value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OnNotificationServoingModeTopic-request)))
  "Returns full string definition for message of type 'OnNotificationServoingModeTopic-request"
  (cl:format cl:nil "NotificationOptions input~%~%================================================================================~%MSG: kortex_driver/NotificationOptions~%~%uint32 type~%uint32 rate_m_sec~%float32 threshold_value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OnNotificationServoingModeTopic-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OnNotificationServoingModeTopic-request>))
  "Converts a ROS message object to a list"
  (cl:list 'OnNotificationServoingModeTopic-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude OnNotificationServoingModeTopic-response.msg.html

(cl:defclass <OnNotificationServoingModeTopic-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:NotificationHandle
    :initform (cl:make-instance 'kortex_driver-msg:NotificationHandle)))
)

(cl:defclass OnNotificationServoingModeTopic-response (<OnNotificationServoingModeTopic-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OnNotificationServoingModeTopic-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OnNotificationServoingModeTopic-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<OnNotificationServoingModeTopic-response> is deprecated: use kortex_driver-srv:OnNotificationServoingModeTopic-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <OnNotificationServoingModeTopic-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OnNotificationServoingModeTopic-response>) ostream)
  "Serializes a message object of type '<OnNotificationServoingModeTopic-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OnNotificationServoingModeTopic-response>) istream)
  "Deserializes a message object of type '<OnNotificationServoingModeTopic-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OnNotificationServoingModeTopic-response>)))
  "Returns string type for a service object of type '<OnNotificationServoingModeTopic-response>"
  "kortex_driver/OnNotificationServoingModeTopicResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationServoingModeTopic-response)))
  "Returns string type for a service object of type 'OnNotificationServoingModeTopic-response"
  "kortex_driver/OnNotificationServoingModeTopicResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OnNotificationServoingModeTopic-response>)))
  "Returns md5sum for a message object of type '<OnNotificationServoingModeTopic-response>"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OnNotificationServoingModeTopic-response)))
  "Returns md5sum for a message object of type 'OnNotificationServoingModeTopic-response"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OnNotificationServoingModeTopic-response>)))
  "Returns full string definition for message of type '<OnNotificationServoingModeTopic-response>"
  (cl:format cl:nil "NotificationHandle output~%~%================================================================================~%MSG: kortex_driver/NotificationHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OnNotificationServoingModeTopic-response)))
  "Returns full string definition for message of type 'OnNotificationServoingModeTopic-response"
  (cl:format cl:nil "NotificationHandle output~%~%================================================================================~%MSG: kortex_driver/NotificationHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OnNotificationServoingModeTopic-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OnNotificationServoingModeTopic-response>))
  "Converts a ROS message object to a list"
  (cl:list 'OnNotificationServoingModeTopic-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'OnNotificationServoingModeTopic)))
  'OnNotificationServoingModeTopic-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'OnNotificationServoingModeTopic)))
  'OnNotificationServoingModeTopic-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationServoingModeTopic)))
  "Returns string type for a service object of type '<OnNotificationServoingModeTopic>"
  "kortex_driver/OnNotificationServoingModeTopic")