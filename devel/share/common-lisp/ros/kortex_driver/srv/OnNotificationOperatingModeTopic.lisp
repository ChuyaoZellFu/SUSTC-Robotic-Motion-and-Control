; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude OnNotificationOperatingModeTopic-request.msg.html

(cl:defclass <OnNotificationOperatingModeTopic-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:NotificationOptions
    :initform (cl:make-instance 'kortex_driver-msg:NotificationOptions)))
)

(cl:defclass OnNotificationOperatingModeTopic-request (<OnNotificationOperatingModeTopic-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OnNotificationOperatingModeTopic-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OnNotificationOperatingModeTopic-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<OnNotificationOperatingModeTopic-request> is deprecated: use kortex_driver-srv:OnNotificationOperatingModeTopic-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <OnNotificationOperatingModeTopic-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OnNotificationOperatingModeTopic-request>) ostream)
  "Serializes a message object of type '<OnNotificationOperatingModeTopic-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OnNotificationOperatingModeTopic-request>) istream)
  "Deserializes a message object of type '<OnNotificationOperatingModeTopic-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OnNotificationOperatingModeTopic-request>)))
  "Returns string type for a service object of type '<OnNotificationOperatingModeTopic-request>"
  "kortex_driver/OnNotificationOperatingModeTopicRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationOperatingModeTopic-request)))
  "Returns string type for a service object of type 'OnNotificationOperatingModeTopic-request"
  "kortex_driver/OnNotificationOperatingModeTopicRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OnNotificationOperatingModeTopic-request>)))
  "Returns md5sum for a message object of type '<OnNotificationOperatingModeTopic-request>"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OnNotificationOperatingModeTopic-request)))
  "Returns md5sum for a message object of type 'OnNotificationOperatingModeTopic-request"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OnNotificationOperatingModeTopic-request>)))
  "Returns full string definition for message of type '<OnNotificationOperatingModeTopic-request>"
  (cl:format cl:nil "NotificationOptions input~%~%================================================================================~%MSG: kortex_driver/NotificationOptions~%~%uint32 type~%uint32 rate_m_sec~%float32 threshold_value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OnNotificationOperatingModeTopic-request)))
  "Returns full string definition for message of type 'OnNotificationOperatingModeTopic-request"
  (cl:format cl:nil "NotificationOptions input~%~%================================================================================~%MSG: kortex_driver/NotificationOptions~%~%uint32 type~%uint32 rate_m_sec~%float32 threshold_value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OnNotificationOperatingModeTopic-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OnNotificationOperatingModeTopic-request>))
  "Converts a ROS message object to a list"
  (cl:list 'OnNotificationOperatingModeTopic-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude OnNotificationOperatingModeTopic-response.msg.html

(cl:defclass <OnNotificationOperatingModeTopic-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:NotificationHandle
    :initform (cl:make-instance 'kortex_driver-msg:NotificationHandle)))
)

(cl:defclass OnNotificationOperatingModeTopic-response (<OnNotificationOperatingModeTopic-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OnNotificationOperatingModeTopic-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OnNotificationOperatingModeTopic-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<OnNotificationOperatingModeTopic-response> is deprecated: use kortex_driver-srv:OnNotificationOperatingModeTopic-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <OnNotificationOperatingModeTopic-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OnNotificationOperatingModeTopic-response>) ostream)
  "Serializes a message object of type '<OnNotificationOperatingModeTopic-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OnNotificationOperatingModeTopic-response>) istream)
  "Deserializes a message object of type '<OnNotificationOperatingModeTopic-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OnNotificationOperatingModeTopic-response>)))
  "Returns string type for a service object of type '<OnNotificationOperatingModeTopic-response>"
  "kortex_driver/OnNotificationOperatingModeTopicResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationOperatingModeTopic-response)))
  "Returns string type for a service object of type 'OnNotificationOperatingModeTopic-response"
  "kortex_driver/OnNotificationOperatingModeTopicResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OnNotificationOperatingModeTopic-response>)))
  "Returns md5sum for a message object of type '<OnNotificationOperatingModeTopic-response>"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OnNotificationOperatingModeTopic-response)))
  "Returns md5sum for a message object of type 'OnNotificationOperatingModeTopic-response"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OnNotificationOperatingModeTopic-response>)))
  "Returns full string definition for message of type '<OnNotificationOperatingModeTopic-response>"
  (cl:format cl:nil "NotificationHandle output~%~%================================================================================~%MSG: kortex_driver/NotificationHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OnNotificationOperatingModeTopic-response)))
  "Returns full string definition for message of type 'OnNotificationOperatingModeTopic-response"
  (cl:format cl:nil "NotificationHandle output~%~%================================================================================~%MSG: kortex_driver/NotificationHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OnNotificationOperatingModeTopic-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OnNotificationOperatingModeTopic-response>))
  "Converts a ROS message object to a list"
  (cl:list 'OnNotificationOperatingModeTopic-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'OnNotificationOperatingModeTopic)))
  'OnNotificationOperatingModeTopic-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'OnNotificationOperatingModeTopic)))
  'OnNotificationOperatingModeTopic-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationOperatingModeTopic)))
  "Returns string type for a service object of type '<OnNotificationOperatingModeTopic>"
  "kortex_driver/OnNotificationOperatingModeTopic")