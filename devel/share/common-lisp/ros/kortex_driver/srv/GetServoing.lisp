; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetServoing-request.msg.html

(cl:defclass <GetServoing-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetServoing-request (<GetServoing-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetServoing-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetServoing-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetServoing-request> is deprecated: use kortex_driver-srv:GetServoing-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetServoing-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetServoing-request>) ostream)
  "Serializes a message object of type '<GetServoing-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetServoing-request>) istream)
  "Deserializes a message object of type '<GetServoing-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetServoing-request>)))
  "Returns string type for a service object of type '<GetServoing-request>"
  "kortex_driver/GetServoingRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetServoing-request)))
  "Returns string type for a service object of type 'GetServoing-request"
  "kortex_driver/GetServoingRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetServoing-request>)))
  "Returns md5sum for a message object of type '<GetServoing-request>"
  "9747040002a13b23ba7503e4b2f380fb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetServoing-request)))
  "Returns md5sum for a message object of type 'GetServoing-request"
  "9747040002a13b23ba7503e4b2f380fb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetServoing-request>)))
  "Returns full string definition for message of type '<GetServoing-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetServoing-request)))
  "Returns full string definition for message of type 'GetServoing-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetServoing-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetServoing-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetServoing-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetServoing-response.msg.html

(cl:defclass <GetServoing-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Servoing
    :initform (cl:make-instance 'kortex_driver-msg:Servoing)))
)

(cl:defclass GetServoing-response (<GetServoing-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetServoing-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetServoing-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetServoing-response> is deprecated: use kortex_driver-srv:GetServoing-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetServoing-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetServoing-response>) ostream)
  "Serializes a message object of type '<GetServoing-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetServoing-response>) istream)
  "Deserializes a message object of type '<GetServoing-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetServoing-response>)))
  "Returns string type for a service object of type '<GetServoing-response>"
  "kortex_driver/GetServoingResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetServoing-response)))
  "Returns string type for a service object of type 'GetServoing-response"
  "kortex_driver/GetServoingResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetServoing-response>)))
  "Returns md5sum for a message object of type '<GetServoing-response>"
  "9747040002a13b23ba7503e4b2f380fb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetServoing-response)))
  "Returns md5sum for a message object of type 'GetServoing-response"
  "9747040002a13b23ba7503e4b2f380fb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetServoing-response>)))
  "Returns full string definition for message of type '<GetServoing-response>"
  (cl:format cl:nil "Servoing output~%~%================================================================================~%MSG: kortex_driver/Servoing~%~%bool enabled~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetServoing-response)))
  "Returns full string definition for message of type 'GetServoing-response"
  (cl:format cl:nil "Servoing output~%~%================================================================================~%MSG: kortex_driver/Servoing~%~%bool enabled~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetServoing-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetServoing-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetServoing-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetServoing)))
  'GetServoing-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetServoing)))
  'GetServoing-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetServoing)))
  "Returns string type for a service object of type '<GetServoing>"
  "kortex_driver/GetServoing")