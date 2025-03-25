; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude OnNotificationProtectionZoneTopic-request.msg.html

(cl:defclass <OnNotificationProtectionZoneTopic-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:NotificationOptions
    :initform (cl:make-instance 'kortex_driver-msg:NotificationOptions)))
)

(cl:defclass OnNotificationProtectionZoneTopic-request (<OnNotificationProtectionZoneTopic-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OnNotificationProtectionZoneTopic-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OnNotificationProtectionZoneTopic-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<OnNotificationProtectionZoneTopic-request> is deprecated: use kortex_driver-srv:OnNotificationProtectionZoneTopic-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <OnNotificationProtectionZoneTopic-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OnNotificationProtectionZoneTopic-request>) ostream)
  "Serializes a message object of type '<OnNotificationProtectionZoneTopic-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OnNotificationProtectionZoneTopic-request>) istream)
  "Deserializes a message object of type '<OnNotificationProtectionZoneTopic-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OnNotificationProtectionZoneTopic-request>)))
  "Returns string type for a service object of type '<OnNotificationProtectionZoneTopic-request>"
  "kortex_driver/OnNotificationProtectionZoneTopicRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationProtectionZoneTopic-request)))
  "Returns string type for a service object of type 'OnNotificationProtectionZoneTopic-request"
  "kortex_driver/OnNotificationProtectionZoneTopicRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OnNotificationProtectionZoneTopic-request>)))
  "Returns md5sum for a message object of type '<OnNotificationProtectionZoneTopic-request>"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OnNotificationProtectionZoneTopic-request)))
  "Returns md5sum for a message object of type 'OnNotificationProtectionZoneTopic-request"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OnNotificationProtectionZoneTopic-request>)))
  "Returns full string definition for message of type '<OnNotificationProtectionZoneTopic-request>"
  (cl:format cl:nil "NotificationOptions input~%~%================================================================================~%MSG: kortex_driver/NotificationOptions~%~%uint32 type~%uint32 rate_m_sec~%float32 threshold_value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OnNotificationProtectionZoneTopic-request)))
  "Returns full string definition for message of type 'OnNotificationProtectionZoneTopic-request"
  (cl:format cl:nil "NotificationOptions input~%~%================================================================================~%MSG: kortex_driver/NotificationOptions~%~%uint32 type~%uint32 rate_m_sec~%float32 threshold_value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OnNotificationProtectionZoneTopic-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OnNotificationProtectionZoneTopic-request>))
  "Converts a ROS message object to a list"
  (cl:list 'OnNotificationProtectionZoneTopic-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude OnNotificationProtectionZoneTopic-response.msg.html

(cl:defclass <OnNotificationProtectionZoneTopic-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:NotificationHandle
    :initform (cl:make-instance 'kortex_driver-msg:NotificationHandle)))
)

(cl:defclass OnNotificationProtectionZoneTopic-response (<OnNotificationProtectionZoneTopic-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OnNotificationProtectionZoneTopic-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OnNotificationProtectionZoneTopic-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<OnNotificationProtectionZoneTopic-response> is deprecated: use kortex_driver-srv:OnNotificationProtectionZoneTopic-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <OnNotificationProtectionZoneTopic-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OnNotificationProtectionZoneTopic-response>) ostream)
  "Serializes a message object of type '<OnNotificationProtectionZoneTopic-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OnNotificationProtectionZoneTopic-response>) istream)
  "Deserializes a message object of type '<OnNotificationProtectionZoneTopic-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OnNotificationProtectionZoneTopic-response>)))
  "Returns string type for a service object of type '<OnNotificationProtectionZoneTopic-response>"
  "kortex_driver/OnNotificationProtectionZoneTopicResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationProtectionZoneTopic-response)))
  "Returns string type for a service object of type 'OnNotificationProtectionZoneTopic-response"
  "kortex_driver/OnNotificationProtectionZoneTopicResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OnNotificationProtectionZoneTopic-response>)))
  "Returns md5sum for a message object of type '<OnNotificationProtectionZoneTopic-response>"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OnNotificationProtectionZoneTopic-response)))
  "Returns md5sum for a message object of type 'OnNotificationProtectionZoneTopic-response"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OnNotificationProtectionZoneTopic-response>)))
  "Returns full string definition for message of type '<OnNotificationProtectionZoneTopic-response>"
  (cl:format cl:nil "NotificationHandle output~%~%================================================================================~%MSG: kortex_driver/NotificationHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OnNotificationProtectionZoneTopic-response)))
  "Returns full string definition for message of type 'OnNotificationProtectionZoneTopic-response"
  (cl:format cl:nil "NotificationHandle output~%~%================================================================================~%MSG: kortex_driver/NotificationHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OnNotificationProtectionZoneTopic-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OnNotificationProtectionZoneTopic-response>))
  "Converts a ROS message object to a list"
  (cl:list 'OnNotificationProtectionZoneTopic-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'OnNotificationProtectionZoneTopic)))
  'OnNotificationProtectionZoneTopic-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'OnNotificationProtectionZoneTopic)))
  'OnNotificationProtectionZoneTopic-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationProtectionZoneTopic)))
  "Returns string type for a service object of type '<OnNotificationProtectionZoneTopic>"
  "kortex_driver/OnNotificationProtectionZoneTopic")