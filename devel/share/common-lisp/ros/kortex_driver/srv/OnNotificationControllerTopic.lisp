; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude OnNotificationControllerTopic-request.msg.html

(cl:defclass <OnNotificationControllerTopic-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:NotificationOptions
    :initform (cl:make-instance 'kortex_driver-msg:NotificationOptions)))
)

(cl:defclass OnNotificationControllerTopic-request (<OnNotificationControllerTopic-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OnNotificationControllerTopic-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OnNotificationControllerTopic-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<OnNotificationControllerTopic-request> is deprecated: use kortex_driver-srv:OnNotificationControllerTopic-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <OnNotificationControllerTopic-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OnNotificationControllerTopic-request>) ostream)
  "Serializes a message object of type '<OnNotificationControllerTopic-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OnNotificationControllerTopic-request>) istream)
  "Deserializes a message object of type '<OnNotificationControllerTopic-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OnNotificationControllerTopic-request>)))
  "Returns string type for a service object of type '<OnNotificationControllerTopic-request>"
  "kortex_driver/OnNotificationControllerTopicRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationControllerTopic-request)))
  "Returns string type for a service object of type 'OnNotificationControllerTopic-request"
  "kortex_driver/OnNotificationControllerTopicRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OnNotificationControllerTopic-request>)))
  "Returns md5sum for a message object of type '<OnNotificationControllerTopic-request>"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OnNotificationControllerTopic-request)))
  "Returns md5sum for a message object of type 'OnNotificationControllerTopic-request"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OnNotificationControllerTopic-request>)))
  "Returns full string definition for message of type '<OnNotificationControllerTopic-request>"
  (cl:format cl:nil "NotificationOptions input~%~%================================================================================~%MSG: kortex_driver/NotificationOptions~%~%uint32 type~%uint32 rate_m_sec~%float32 threshold_value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OnNotificationControllerTopic-request)))
  "Returns full string definition for message of type 'OnNotificationControllerTopic-request"
  (cl:format cl:nil "NotificationOptions input~%~%================================================================================~%MSG: kortex_driver/NotificationOptions~%~%uint32 type~%uint32 rate_m_sec~%float32 threshold_value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OnNotificationControllerTopic-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OnNotificationControllerTopic-request>))
  "Converts a ROS message object to a list"
  (cl:list 'OnNotificationControllerTopic-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude OnNotificationControllerTopic-response.msg.html

(cl:defclass <OnNotificationControllerTopic-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:NotificationHandle
    :initform (cl:make-instance 'kortex_driver-msg:NotificationHandle)))
)

(cl:defclass OnNotificationControllerTopic-response (<OnNotificationControllerTopic-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OnNotificationControllerTopic-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OnNotificationControllerTopic-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<OnNotificationControllerTopic-response> is deprecated: use kortex_driver-srv:OnNotificationControllerTopic-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <OnNotificationControllerTopic-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OnNotificationControllerTopic-response>) ostream)
  "Serializes a message object of type '<OnNotificationControllerTopic-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OnNotificationControllerTopic-response>) istream)
  "Deserializes a message object of type '<OnNotificationControllerTopic-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OnNotificationControllerTopic-response>)))
  "Returns string type for a service object of type '<OnNotificationControllerTopic-response>"
  "kortex_driver/OnNotificationControllerTopicResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationControllerTopic-response)))
  "Returns string type for a service object of type 'OnNotificationControllerTopic-response"
  "kortex_driver/OnNotificationControllerTopicResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OnNotificationControllerTopic-response>)))
  "Returns md5sum for a message object of type '<OnNotificationControllerTopic-response>"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OnNotificationControllerTopic-response)))
  "Returns md5sum for a message object of type 'OnNotificationControllerTopic-response"
  "6fefdd07c6cb63a94f7b48e7e07e815b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OnNotificationControllerTopic-response>)))
  "Returns full string definition for message of type '<OnNotificationControllerTopic-response>"
  (cl:format cl:nil "NotificationHandle output~%~%================================================================================~%MSG: kortex_driver/NotificationHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OnNotificationControllerTopic-response)))
  "Returns full string definition for message of type 'OnNotificationControllerTopic-response"
  (cl:format cl:nil "NotificationHandle output~%~%================================================================================~%MSG: kortex_driver/NotificationHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OnNotificationControllerTopic-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OnNotificationControllerTopic-response>))
  "Converts a ROS message object to a list"
  (cl:list 'OnNotificationControllerTopic-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'OnNotificationControllerTopic)))
  'OnNotificationControllerTopic-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'OnNotificationControllerTopic)))
  'OnNotificationControllerTopic-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OnNotificationControllerTopic)))
  "Returns string type for a service object of type '<OnNotificationControllerTopic>"
  "kortex_driver/OnNotificationControllerTopic")