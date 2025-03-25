; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude DisconnectWifi-request.msg.html

(cl:defclass <DisconnectWifi-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass DisconnectWifi-request (<DisconnectWifi-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DisconnectWifi-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DisconnectWifi-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<DisconnectWifi-request> is deprecated: use kortex_driver-srv:DisconnectWifi-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <DisconnectWifi-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DisconnectWifi-request>) ostream)
  "Serializes a message object of type '<DisconnectWifi-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DisconnectWifi-request>) istream)
  "Deserializes a message object of type '<DisconnectWifi-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DisconnectWifi-request>)))
  "Returns string type for a service object of type '<DisconnectWifi-request>"
  "kortex_driver/DisconnectWifiRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DisconnectWifi-request)))
  "Returns string type for a service object of type 'DisconnectWifi-request"
  "kortex_driver/DisconnectWifiRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DisconnectWifi-request>)))
  "Returns md5sum for a message object of type '<DisconnectWifi-request>"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DisconnectWifi-request)))
  "Returns md5sum for a message object of type 'DisconnectWifi-request"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DisconnectWifi-request>)))
  "Returns full string definition for message of type '<DisconnectWifi-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DisconnectWifi-request)))
  "Returns full string definition for message of type 'DisconnectWifi-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DisconnectWifi-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DisconnectWifi-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DisconnectWifi-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude DisconnectWifi-response.msg.html

(cl:defclass <DisconnectWifi-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass DisconnectWifi-response (<DisconnectWifi-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DisconnectWifi-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DisconnectWifi-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<DisconnectWifi-response> is deprecated: use kortex_driver-srv:DisconnectWifi-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <DisconnectWifi-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DisconnectWifi-response>) ostream)
  "Serializes a message object of type '<DisconnectWifi-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DisconnectWifi-response>) istream)
  "Deserializes a message object of type '<DisconnectWifi-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DisconnectWifi-response>)))
  "Returns string type for a service object of type '<DisconnectWifi-response>"
  "kortex_driver/DisconnectWifiResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DisconnectWifi-response)))
  "Returns string type for a service object of type 'DisconnectWifi-response"
  "kortex_driver/DisconnectWifiResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DisconnectWifi-response>)))
  "Returns md5sum for a message object of type '<DisconnectWifi-response>"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DisconnectWifi-response)))
  "Returns md5sum for a message object of type 'DisconnectWifi-response"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DisconnectWifi-response>)))
  "Returns full string definition for message of type '<DisconnectWifi-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DisconnectWifi-response)))
  "Returns full string definition for message of type 'DisconnectWifi-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DisconnectWifi-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DisconnectWifi-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DisconnectWifi-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DisconnectWifi)))
  'DisconnectWifi-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DisconnectWifi)))
  'DisconnectWifi-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DisconnectWifi)))
  "Returns string type for a service object of type '<DisconnectWifi>"
  "kortex_driver/DisconnectWifi")