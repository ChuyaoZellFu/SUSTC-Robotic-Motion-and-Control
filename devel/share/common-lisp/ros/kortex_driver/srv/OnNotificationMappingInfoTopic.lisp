; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude OnNotificationMappingInfoTopic-request.msg.html

(cl:defclass <OnNotificationMappingInfoTopic-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:NotificationOptions
    :initform (cl:make-instance 'kortex_driver-msg:NotificationOptions)))
)

(cl:defclass OnNotificationMappingInfoTopic-request (<OnNotificationMappingInfoTopic-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OnNotificationMappingInfoTopic-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OnNotificationMappingInfoTopic-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<OnNotificationMappingInfoTopic-request> is deprecated: use kortex_driver-srv:OnNotificationMappingInfoTopic-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <OnNotificationMappingInfoTopic-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OnNotificationMappingInfoTopic-request>) ostream)
  "Serializes a message object of type '<OnNotificationMappingInfoTopic-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OnNotificationMappingInfoTopic-request>) istream)
  "Deserializes a message object of type '<OnNotificationMappingInfoTopic-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OnNotificationMappingInfoTopic-request>)))
  "Returns string type for a service object of type '<OnNotificationMappingInfoTopic-request>"
  "kortex_driver/OnNotificationMappingInfoTopicRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationMappingInfoTopic-request)))
  "Returns string type for a service object of type 'OnNotificationMappingInfoTopic-request"
  "kortex_driver/OnNotificationMappingInfoTopicRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OnNotificationMappingInfoTopic-request>)))
  "Returns md5sum for a message object of type '<OnNotificationMappingInfoTopic-request>"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OnNotificationMappingInfoTopic-request)))
  "Returns md5sum for a message object of type 'OnNotificationMappingInfoTopic-request"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OnNotificationMappingInfoTopic-request>)))
  "Returns full string definition for message of type '<OnNotificationMappingInfoTopic-request>"
  (cl:format cl:nil "NotificationOptions input~%~%================================================================================~%MSG: kortex_driver/NotificationOptions~%~%uint32 type~%uint32 rate_m_sec~%float32 threshold_value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OnNotificationMappingInfoTopic-request)))
  "Returns full string definition for message of type 'OnNotificationMappingInfoTopic-request"
  (cl:format cl:nil "NotificationOptions input~%~%================================================================================~%MSG: kortex_driver/NotificationOptions~%~%uint32 type~%uint32 rate_m_sec~%float32 threshold_value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OnNotificationMappingInfoTopic-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OnNotificationMappingInfoTopic-request>))
  "Converts a ROS message object to a list"
  (cl:list 'OnNotificationMappingInfoTopic-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude OnNotificationMappingInfoTopic-response.msg.html

(cl:defclass <OnNotificationMappingInfoTopic-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:NotificationHandle
    :initform (cl:make-instance 'kortex_driver-msg:NotificationHandle)))
)

(cl:defclass OnNotificationMappingInfoTopic-response (<OnNotificationMappingInfoTopic-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OnNotificationMappingInfoTopic-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OnNotificationMappingInfoTopic-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<OnNotificationMappingInfoTopic-response> is deprecated: use kortex_driver-srv:OnNotificationMappingInfoTopic-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <OnNotificationMappingInfoTopic-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OnNotificationMappingInfoTopic-response>) ostream)
  "Serializes a message object of type '<OnNotificationMappingInfoTopic-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OnNotificationMappingInfoTopic-response>) istream)
  "Deserializes a message object of type '<OnNotificationMappingInfoTopic-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OnNotificationMappingInfoTopic-response>)))
  "Returns string type for a service object of type '<OnNotificationMappingInfoTopic-response>"
  "kortex_driver/OnNotificationMappingInfoTopicResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationMappingInfoTopic-response)))
  "Returns string type for a service object of type 'OnNotificationMappingInfoTopic-response"
  "kortex_driver/OnNotificationMappingInfoTopicResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OnNotificationMappingInfoTopic-response>)))
  "Returns md5sum for a message object of type '<OnNotificationMappingInfoTopic-response>"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OnNotificationMappingInfoTopic-response)))
  "Returns md5sum for a message object of type 'OnNotificationMappingInfoTopic-response"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OnNotificationMappingInfoTopic-response>)))
  "Returns full string definition for message of type '<OnNotificationMappingInfoTopic-response>"
  (cl:format cl:nil "NotificationHandle output~%~%================================================================================~%MSG: kortex_driver/NotificationHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OnNotificationMappingInfoTopic-response)))
  "Returns full string definition for message of type 'OnNotificationMappingInfoTopic-response"
  (cl:format cl:nil "NotificationHandle output~%~%================================================================================~%MSG: kortex_driver/NotificationHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OnNotificationMappingInfoTopic-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OnNotificationMappingInfoTopic-response>))
  "Converts a ROS message object to a list"
  (cl:list 'OnNotificationMappingInfoTopic-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'OnNotificationMappingInfoTopic)))
  'OnNotificationMappingInfoTopic-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'OnNotificationMappingInfoTopic)))
  'OnNotificationMappingInfoTopic-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationMappingInfoTopic)))
  "Returns string type for a service object of type '<OnNotificationMappingInfoTopic>"
  "kortex_driver/OnNotificationMappingInfoTopic")