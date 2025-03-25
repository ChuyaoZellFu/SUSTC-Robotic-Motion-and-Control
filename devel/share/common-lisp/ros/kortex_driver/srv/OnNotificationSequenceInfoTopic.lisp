; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude OnNotificationSequenceInfoTopic-request.msg.html

(cl:defclass <OnNotificationSequenceInfoTopic-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:NotificationOptions
    :initform (cl:make-instance 'kortex_driver-msg:NotificationOptions)))
)

(cl:defclass OnNotificationSequenceInfoTopic-request (<OnNotificationSequenceInfoTopic-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OnNotificationSequenceInfoTopic-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OnNotificationSequenceInfoTopic-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<OnNotificationSequenceInfoTopic-request> is deprecated: use kortex_driver-srv:OnNotificationSequenceInfoTopic-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <OnNotificationSequenceInfoTopic-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OnNotificationSequenceInfoTopic-request>) ostream)
  "Serializes a message object of type '<OnNotificationSequenceInfoTopic-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OnNotificationSequenceInfoTopic-request>) istream)
  "Deserializes a message object of type '<OnNotificationSequenceInfoTopic-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OnNotificationSequenceInfoTopic-request>)))
  "Returns string type for a service object of type '<OnNotificationSequenceInfoTopic-request>"
  "kortex_driver/OnNotificationSequenceInfoTopicRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationSequenceInfoTopic-request)))
  "Returns string type for a service object of type 'OnNotificationSequenceInfoTopic-request"
  "kortex_driver/OnNotificationSequenceInfoTopicRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OnNotificationSequenceInfoTopic-request>)))
  "Returns md5sum for a message object of type '<OnNotificationSequenceInfoTopic-request>"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OnNotificationSequenceInfoTopic-request)))
  "Returns md5sum for a message object of type 'OnNotificationSequenceInfoTopic-request"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OnNotificationSequenceInfoTopic-request>)))
  "Returns full string definition for message of type '<OnNotificationSequenceInfoTopic-request>"
  (cl:format cl:nil "NotificationOptions input~%~%================================================================================~%MSG: kortex_driver/NotificationOptions~%~%uint32 type~%uint32 rate_m_sec~%float32 threshold_value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OnNotificationSequenceInfoTopic-request)))
  "Returns full string definition for message of type 'OnNotificationSequenceInfoTopic-request"
  (cl:format cl:nil "NotificationOptions input~%~%================================================================================~%MSG: kortex_driver/NotificationOptions~%~%uint32 type~%uint32 rate_m_sec~%float32 threshold_value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OnNotificationSequenceInfoTopic-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OnNotificationSequenceInfoTopic-request>))
  "Converts a ROS message object to a list"
  (cl:list 'OnNotificationSequenceInfoTopic-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude OnNotificationSequenceInfoTopic-response.msg.html

(cl:defclass <OnNotificationSequenceInfoTopic-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:NotificationHandle
    :initform (cl:make-instance 'kortex_driver-msg:NotificationHandle)))
)

(cl:defclass OnNotificationSequenceInfoTopic-response (<OnNotificationSequenceInfoTopic-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OnNotificationSequenceInfoTopic-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OnNotificationSequenceInfoTopic-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<OnNotificationSequenceInfoTopic-response> is deprecated: use kortex_driver-srv:OnNotificationSequenceInfoTopic-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <OnNotificationSequenceInfoTopic-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OnNotificationSequenceInfoTopic-response>) ostream)
  "Serializes a message object of type '<OnNotificationSequenceInfoTopic-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OnNotificationSequenceInfoTopic-response>) istream)
  "Deserializes a message object of type '<OnNotificationSequenceInfoTopic-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OnNotificationSequenceInfoTopic-response>)))
  "Returns string type for a service object of type '<OnNotificationSequenceInfoTopic-response>"
  "kortex_driver/OnNotificationSequenceInfoTopicResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationSequenceInfoTopic-response)))
  "Returns string type for a service object of type 'OnNotificationSequenceInfoTopic-response"
  "kortex_driver/OnNotificationSequenceInfoTopicResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OnNotificationSequenceInfoTopic-response>)))
  "Returns md5sum for a message object of type '<OnNotificationSequenceInfoTopic-response>"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OnNotificationSequenceInfoTopic-response)))
  "Returns md5sum for a message object of type 'OnNotificationSequenceInfoTopic-response"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OnNotificationSequenceInfoTopic-response>)))
  "Returns full string definition for message of type '<OnNotificationSequenceInfoTopic-response>"
  (cl:format cl:nil "NotificationHandle output~%~%================================================================================~%MSG: kortex_driver/NotificationHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OnNotificationSequenceInfoTopic-response)))
  "Returns full string definition for message of type 'OnNotificationSequenceInfoTopic-response"
  (cl:format cl:nil "NotificationHandle output~%~%================================================================================~%MSG: kortex_driver/NotificationHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OnNotificationSequenceInfoTopic-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OnNotificationSequenceInfoTopic-response>))
  "Converts a ROS message object to a list"
  (cl:list 'OnNotificationSequenceInfoTopic-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'OnNotificationSequenceInfoTopic)))
  'OnNotificationSequenceInfoTopic-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'OnNotificationSequenceInfoTopic)))
  'OnNotificationSequenceInfoTopic-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationSequenceInfoTopic)))
  "Returns string type for a service object of type '<OnNotificationSequenceInfoTopic>"
  "kortex_driver/OnNotificationSequenceInfoTopic")