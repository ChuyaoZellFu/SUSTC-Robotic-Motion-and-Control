; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetWifiEnableState-request.msg.html

(cl:defclass <SetWifiEnableState-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:WifiEnableState
    :initform (cl:make-instance 'kortex_driver-msg:WifiEnableState)))
)

(cl:defclass SetWifiEnableState-request (<SetWifiEnableState-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetWifiEnableState-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetWifiEnableState-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetWifiEnableState-request> is deprecated: use kortex_driver-srv:SetWifiEnableState-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetWifiEnableState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetWifiEnableState-request>) ostream)
  "Serializes a message object of type '<SetWifiEnableState-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetWifiEnableState-request>) istream)
  "Deserializes a message object of type '<SetWifiEnableState-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetWifiEnableState-request>)))
  "Returns string type for a service object of type '<SetWifiEnableState-request>"
  "kortex_driver/SetWifiEnableStateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetWifiEnableState-request)))
  "Returns string type for a service object of type 'SetWifiEnableState-request"
  "kortex_driver/SetWifiEnableStateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetWifiEnableState-request>)))
  "Returns md5sum for a message object of type '<SetWifiEnableState-request>"
  "7ccd6a6c56b1642433541444d0e2ab2f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetWifiEnableState-request)))
  "Returns md5sum for a message object of type 'SetWifiEnableState-request"
  "7ccd6a6c56b1642433541444d0e2ab2f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetWifiEnableState-request>)))
  "Returns full string definition for message of type '<SetWifiEnableState-request>"
  (cl:format cl:nil "WifiEnableState input~%~%================================================================================~%MSG: kortex_driver/WifiEnableState~%~%bool enabled~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetWifiEnableState-request)))
  "Returns full string definition for message of type 'SetWifiEnableState-request"
  (cl:format cl:nil "WifiEnableState input~%~%================================================================================~%MSG: kortex_driver/WifiEnableState~%~%bool enabled~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetWifiEnableState-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetWifiEnableState-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetWifiEnableState-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetWifiEnableState-response.msg.html

(cl:defclass <SetWifiEnableState-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetWifiEnableState-response (<SetWifiEnableState-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetWifiEnableState-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetWifiEnableState-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetWifiEnableState-response> is deprecated: use kortex_driver-srv:SetWifiEnableState-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetWifiEnableState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetWifiEnableState-response>) ostream)
  "Serializes a message object of type '<SetWifiEnableState-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetWifiEnableState-response>) istream)
  "Deserializes a message object of type '<SetWifiEnableState-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetWifiEnableState-response>)))
  "Returns string type for a service object of type '<SetWifiEnableState-response>"
  "kortex_driver/SetWifiEnableStateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetWifiEnableState-response)))
  "Returns string type for a service object of type 'SetWifiEnableState-response"
  "kortex_driver/SetWifiEnableStateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetWifiEnableState-response>)))
  "Returns md5sum for a message object of type '<SetWifiEnableState-response>"
  "7ccd6a6c56b1642433541444d0e2ab2f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetWifiEnableState-response)))
  "Returns md5sum for a message object of type 'SetWifiEnableState-response"
  "7ccd6a6c56b1642433541444d0e2ab2f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetWifiEnableState-response>)))
  "Returns full string definition for message of type '<SetWifiEnableState-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetWifiEnableState-response)))
  "Returns full string definition for message of type 'SetWifiEnableState-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetWifiEnableState-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetWifiEnableState-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetWifiEnableState-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetWifiEnableState)))
  'SetWifiEnableState-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetWifiEnableState)))
  'SetWifiEnableState-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetWifiEnableState)))
  "Returns string type for a service object of type '<SetWifiEnableState>"
  "kortex_driver/SetWifiEnableState")