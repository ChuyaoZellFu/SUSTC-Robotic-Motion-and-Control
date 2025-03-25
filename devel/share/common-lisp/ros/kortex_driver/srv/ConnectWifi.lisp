; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude ConnectWifi-request.msg.html

(cl:defclass <ConnectWifi-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Ssid
    :initform (cl:make-instance 'kortex_driver-msg:Ssid)))
)

(cl:defclass ConnectWifi-request (<ConnectWifi-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConnectWifi-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConnectWifi-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ConnectWifi-request> is deprecated: use kortex_driver-srv:ConnectWifi-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <ConnectWifi-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConnectWifi-request>) ostream)
  "Serializes a message object of type '<ConnectWifi-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConnectWifi-request>) istream)
  "Deserializes a message object of type '<ConnectWifi-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConnectWifi-request>)))
  "Returns string type for a service object of type '<ConnectWifi-request>"
  "kortex_driver/ConnectWifiRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConnectWifi-request)))
  "Returns string type for a service object of type 'ConnectWifi-request"
  "kortex_driver/ConnectWifiRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConnectWifi-request>)))
  "Returns md5sum for a message object of type '<ConnectWifi-request>"
  "fc936133533cfced4ae662bec0d72a39")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConnectWifi-request)))
  "Returns md5sum for a message object of type 'ConnectWifi-request"
  "fc936133533cfced4ae662bec0d72a39")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConnectWifi-request>)))
  "Returns full string definition for message of type '<ConnectWifi-request>"
  (cl:format cl:nil "Ssid input~%~%================================================================================~%MSG: kortex_driver/Ssid~%~%string identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConnectWifi-request)))
  "Returns full string definition for message of type 'ConnectWifi-request"
  (cl:format cl:nil "Ssid input~%~%================================================================================~%MSG: kortex_driver/Ssid~%~%string identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConnectWifi-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConnectWifi-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ConnectWifi-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude ConnectWifi-response.msg.html

(cl:defclass <ConnectWifi-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass ConnectWifi-response (<ConnectWifi-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConnectWifi-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConnectWifi-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ConnectWifi-response> is deprecated: use kortex_driver-srv:ConnectWifi-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <ConnectWifi-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConnectWifi-response>) ostream)
  "Serializes a message object of type '<ConnectWifi-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConnectWifi-response>) istream)
  "Deserializes a message object of type '<ConnectWifi-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConnectWifi-response>)))
  "Returns string type for a service object of type '<ConnectWifi-response>"
  "kortex_driver/ConnectWifiResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConnectWifi-response)))
  "Returns string type for a service object of type 'ConnectWifi-response"
  "kortex_driver/ConnectWifiResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConnectWifi-response>)))
  "Returns md5sum for a message object of type '<ConnectWifi-response>"
  "fc936133533cfced4ae662bec0d72a39")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConnectWifi-response)))
  "Returns md5sum for a message object of type 'ConnectWifi-response"
  "fc936133533cfced4ae662bec0d72a39")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConnectWifi-response>)))
  "Returns full string definition for message of type '<ConnectWifi-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConnectWifi-response)))
  "Returns full string definition for message of type 'ConnectWifi-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConnectWifi-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConnectWifi-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ConnectWifi-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ConnectWifi)))
  'ConnectWifi-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ConnectWifi)))
  'ConnectWifi-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConnectWifi)))
  "Returns string type for a service object of type '<ConnectWifi>"
  "kortex_driver/ConnectWifi")