; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude OnNotificationVisionTopic-request.msg.html

(cl:defclass <OnNotificationVisionTopic-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:NotificationOptions
    :initform (cl:make-instance 'kortex_driver-msg:NotificationOptions)))
)

(cl:defclass OnNotificationVisionTopic-request (<OnNotificationVisionTopic-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OnNotificationVisionTopic-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OnNotificationVisionTopic-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<OnNotificationVisionTopic-request> is deprecated: use kortex_driver-srv:OnNotificationVisionTopic-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <OnNotificationVisionTopic-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OnNotificationVisionTopic-request>) ostream)
  "Serializes a message object of type '<OnNotificationVisionTopic-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OnNotificationVisionTopic-request>) istream)
  "Deserializes a message object of type '<OnNotificationVisionTopic-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OnNotificationVisionTopic-request>)))
  "Returns string type for a service object of type '<OnNotificationVisionTopic-request>"
  "kortex_driver/OnNotificationVisionTopicRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationVisionTopic-request)))
  "Returns string type for a service object of type 'OnNotificationVisionTopic-request"
  "kortex_driver/OnNotificationVisionTopicRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OnNotificationVisionTopic-request>)))
  "Returns md5sum for a message object of type '<OnNotificationVisionTopic-request>"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OnNotificationVisionTopic-request)))
  "Returns md5sum for a message object of type 'OnNotificationVisionTopic-request"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OnNotificationVisionTopic-request>)))
  "Returns full string definition for message of type '<OnNotificationVisionTopic-request>"
  (cl:format cl:nil "NotificationOptions input~%~%================================================================================~%MSG: kortex_driver/NotificationOptions~%~%uint32 type~%uint32 rate_m_sec~%float32 threshold_value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OnNotificationVisionTopic-request)))
  "Returns full string definition for message of type 'OnNotificationVisionTopic-request"
  (cl:format cl:nil "NotificationOptions input~%~%================================================================================~%MSG: kortex_driver/NotificationOptions~%~%uint32 type~%uint32 rate_m_sec~%float32 threshold_value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OnNotificationVisionTopic-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OnNotificationVisionTopic-request>))
  "Converts a ROS message object to a list"
  (cl:list 'OnNotificationVisionTopic-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude OnNotificationVisionTopic-response.msg.html

(cl:defclass <OnNotificationVisionTopic-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:NotificationHandle
    :initform (cl:make-instance 'kortex_driver-msg:NotificationHandle)))
)

(cl:defclass OnNotificationVisionTopic-response (<OnNotificationVisionTopic-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OnNotificationVisionTopic-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OnNotificationVisionTopic-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<OnNotificationVisionTopic-response> is deprecated: use kortex_driver-srv:OnNotificationVisionTopic-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <OnNotificationVisionTopic-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OnNotificationVisionTopic-response>) ostream)
  "Serializes a message object of type '<OnNotificationVisionTopic-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OnNotificationVisionTopic-response>) istream)
  "Deserializes a message object of type '<OnNotificationVisionTopic-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OnNotificationVisionTopic-response>)))
  "Returns string type for a service object of type '<OnNotificationVisionTopic-response>"
  "kortex_driver/OnNotificationVisionTopicResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationVisionTopic-response)))
  "Returns string type for a service object of type 'OnNotificationVisionTopic-response"
  "kortex_driver/OnNotificationVisionTopicResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OnNotificationVisionTopic-response>)))
  "Returns md5sum for a message object of type '<OnNotificationVisionTopic-response>"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OnNotificationVisionTopic-response)))
  "Returns md5sum for a message object of type 'OnNotificationVisionTopic-response"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OnNotificationVisionTopic-response>)))
  "Returns full string definition for message of type '<OnNotificationVisionTopic-response>"
  (cl:format cl:nil "NotificationHandle output~%~%================================================================================~%MSG: kortex_driver/NotificationHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OnNotificationVisionTopic-response)))
  "Returns full string definition for message of type 'OnNotificationVisionTopic-response"
  (cl:format cl:nil "NotificationHandle output~%~%================================================================================~%MSG: kortex_driver/NotificationHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OnNotificationVisionTopic-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OnNotificationVisionTopic-response>))
  "Converts a ROS message object to a list"
  (cl:list 'OnNotificationVisionTopic-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'OnNotificationVisionTopic)))
  'OnNotificationVisionTopic-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'OnNotificationVisionTopic)))
  'OnNotificationVisionTopic-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationVisionTopic)))
  "Returns string type for a service object of type '<OnNotificationVisionTopic>"
  "kortex_driver/OnNotificationVisionTopic")