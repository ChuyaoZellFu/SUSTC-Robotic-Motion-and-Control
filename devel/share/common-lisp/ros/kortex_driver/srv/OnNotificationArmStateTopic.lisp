; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude OnNotificationArmStateTopic-request.msg.html

(cl:defclass <OnNotificationArmStateTopic-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:NotificationOptions
    :initform (cl:make-instance 'kortex_driver-msg:NotificationOptions)))
)

(cl:defclass OnNotificationArmStateTopic-request (<OnNotificationArmStateTopic-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OnNotificationArmStateTopic-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OnNotificationArmStateTopic-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<OnNotificationArmStateTopic-request> is deprecated: use kortex_driver-srv:OnNotificationArmStateTopic-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <OnNotificationArmStateTopic-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OnNotificationArmStateTopic-request>) ostream)
  "Serializes a message object of type '<OnNotificationArmStateTopic-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OnNotificationArmStateTopic-request>) istream)
  "Deserializes a message object of type '<OnNotificationArmStateTopic-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OnNotificationArmStateTopic-request>)))
  "Returns string type for a service object of type '<OnNotificationArmStateTopic-request>"
  "kortex_driver/OnNotificationArmStateTopicRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationArmStateTopic-request)))
  "Returns string type for a service object of type 'OnNotificationArmStateTopic-request"
  "kortex_driver/OnNotificationArmStateTopicRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OnNotificationArmStateTopic-request>)))
  "Returns md5sum for a message object of type '<OnNotificationArmStateTopic-request>"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OnNotificationArmStateTopic-request)))
  "Returns md5sum for a message object of type 'OnNotificationArmStateTopic-request"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OnNotificationArmStateTopic-request>)))
  "Returns full string definition for message of type '<OnNotificationArmStateTopic-request>"
  (cl:format cl:nil "NotificationOptions input~%~%================================================================================~%MSG: kortex_driver/NotificationOptions~%~%uint32 type~%uint32 rate_m_sec~%float32 threshold_value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OnNotificationArmStateTopic-request)))
  "Returns full string definition for message of type 'OnNotificationArmStateTopic-request"
  (cl:format cl:nil "NotificationOptions input~%~%================================================================================~%MSG: kortex_driver/NotificationOptions~%~%uint32 type~%uint32 rate_m_sec~%float32 threshold_value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OnNotificationArmStateTopic-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OnNotificationArmStateTopic-request>))
  "Converts a ROS message object to a list"
  (cl:list 'OnNotificationArmStateTopic-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude OnNotificationArmStateTopic-response.msg.html

(cl:defclass <OnNotificationArmStateTopic-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:NotificationHandle
    :initform (cl:make-instance 'kortex_driver-msg:NotificationHandle)))
)

(cl:defclass OnNotificationArmStateTopic-response (<OnNotificationArmStateTopic-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OnNotificationArmStateTopic-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OnNotificationArmStateTopic-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<OnNotificationArmStateTopic-response> is deprecated: use kortex_driver-srv:OnNotificationArmStateTopic-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <OnNotificationArmStateTopic-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OnNotificationArmStateTopic-response>) ostream)
  "Serializes a message object of type '<OnNotificationArmStateTopic-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OnNotificationArmStateTopic-response>) istream)
  "Deserializes a message object of type '<OnNotificationArmStateTopic-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OnNotificationArmStateTopic-response>)))
  "Returns string type for a service object of type '<OnNotificationArmStateTopic-response>"
  "kortex_driver/OnNotificationArmStateTopicResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationArmStateTopic-response)))
  "Returns string type for a service object of type 'OnNotificationArmStateTopic-response"
  "kortex_driver/OnNotificationArmStateTopicResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OnNotificationArmStateTopic-response>)))
  "Returns md5sum for a message object of type '<OnNotificationArmStateTopic-response>"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OnNotificationArmStateTopic-response)))
  "Returns md5sum for a message object of type 'OnNotificationArmStateTopic-response"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OnNotificationArmStateTopic-response>)))
  "Returns full string definition for message of type '<OnNotificationArmStateTopic-response>"
  (cl:format cl:nil "NotificationHandle output~%~%================================================================================~%MSG: kortex_driver/NotificationHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OnNotificationArmStateTopic-response)))
  "Returns full string definition for message of type 'OnNotificationArmStateTopic-response"
  (cl:format cl:nil "NotificationHandle output~%~%================================================================================~%MSG: kortex_driver/NotificationHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OnNotificationArmStateTopic-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OnNotificationArmStateTopic-response>))
  "Converts a ROS message object to a list"
  (cl:list 'OnNotificationArmStateTopic-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'OnNotificationArmStateTopic)))
  'OnNotificationArmStateTopic-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'OnNotificationArmStateTopic)))
  'OnNotificationArmStateTopic-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationArmStateTopic)))
  "Returns string type for a service object of type '<OnNotificationArmStateTopic>"
  "kortex_driver/OnNotificationArmStateTopic")