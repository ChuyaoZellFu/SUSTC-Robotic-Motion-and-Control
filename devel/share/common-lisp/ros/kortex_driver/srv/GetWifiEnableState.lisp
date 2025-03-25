; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetWifiEnableState-request.msg.html

(cl:defclass <GetWifiEnableState-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetWifiEnableState-request (<GetWifiEnableState-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetWifiEnableState-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetWifiEnableState-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetWifiEnableState-request> is deprecated: use kortex_driver-srv:GetWifiEnableState-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetWifiEnableState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetWifiEnableState-request>) ostream)
  "Serializes a message object of type '<GetWifiEnableState-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetWifiEnableState-request>) istream)
  "Deserializes a message object of type '<GetWifiEnableState-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetWifiEnableState-request>)))
  "Returns string type for a service object of type '<GetWifiEnableState-request>"
  "kortex_driver/GetWifiEnableStateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetWifiEnableState-request)))
  "Returns string type for a service object of type 'GetWifiEnableState-request"
  "kortex_driver/GetWifiEnableStateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetWifiEnableState-request>)))
  "Returns md5sum for a message object of type '<GetWifiEnableState-request>"
  "9747040002a13b23ba7503e4b2f380fb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetWifiEnableState-request)))
  "Returns md5sum for a message object of type 'GetWifiEnableState-request"
  "9747040002a13b23ba7503e4b2f380fb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetWifiEnableState-request>)))
  "Returns full string definition for message of type '<GetWifiEnableState-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetWifiEnableState-request)))
  "Returns full string definition for message of type 'GetWifiEnableState-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetWifiEnableState-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetWifiEnableState-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetWifiEnableState-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetWifiEnableState-response.msg.html

(cl:defclass <GetWifiEnableState-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:WifiEnableState
    :initform (cl:make-instance 'kortex_driver-msg:WifiEnableState)))
)

(cl:defclass GetWifiEnableState-response (<GetWifiEnableState-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetWifiEnableState-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetWifiEnableState-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetWifiEnableState-response> is deprecated: use kortex_driver-srv:GetWifiEnableState-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetWifiEnableState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetWifiEnableState-response>) ostream)
  "Serializes a message object of type '<GetWifiEnableState-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetWifiEnableState-response>) istream)
  "Deserializes a message object of type '<GetWifiEnableState-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetWifiEnableState-response>)))
  "Returns string type for a service object of type '<GetWifiEnableState-response>"
  "kortex_driver/GetWifiEnableStateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetWifiEnableState-response)))
  "Returns string type for a service object of type 'GetWifiEnableState-response"
  "kortex_driver/GetWifiEnableStateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetWifiEnableState-response>)))
  "Returns md5sum for a message object of type '<GetWifiEnableState-response>"
  "9747040002a13b23ba7503e4b2f380fb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetWifiEnableState-response)))
  "Returns md5sum for a message object of type 'GetWifiEnableState-response"
  "9747040002a13b23ba7503e4b2f380fb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetWifiEnableState-response>)))
  "Returns full string definition for message of type '<GetWifiEnableState-response>"
  (cl:format cl:nil "WifiEnableState output~%~%================================================================================~%MSG: kortex_driver/WifiEnableState~%~%bool enabled~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetWifiEnableState-response)))
  "Returns full string definition for message of type 'GetWifiEnableState-response"
  (cl:format cl:nil "WifiEnableState output~%~%================================================================================~%MSG: kortex_driver/WifiEnableState~%~%bool enabled~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetWifiEnableState-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetWifiEnableState-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetWifiEnableState-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetWifiEnableState)))
  'GetWifiEnableState-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetWifiEnableState)))
  'GetWifiEnableState-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetWifiEnableState)))
  "Returns string type for a service object of type '<GetWifiEnableState>"
  "kortex_driver/GetWifiEnableState")