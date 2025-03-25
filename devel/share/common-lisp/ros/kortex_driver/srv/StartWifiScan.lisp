; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude StartWifiScan-request.msg.html

(cl:defclass <StartWifiScan-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass StartWifiScan-request (<StartWifiScan-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartWifiScan-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartWifiScan-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<StartWifiScan-request> is deprecated: use kortex_driver-srv:StartWifiScan-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <StartWifiScan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartWifiScan-request>) ostream)
  "Serializes a message object of type '<StartWifiScan-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartWifiScan-request>) istream)
  "Deserializes a message object of type '<StartWifiScan-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartWifiScan-request>)))
  "Returns string type for a service object of type '<StartWifiScan-request>"
  "kortex_driver/StartWifiScanRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartWifiScan-request)))
  "Returns string type for a service object of type 'StartWifiScan-request"
  "kortex_driver/StartWifiScanRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartWifiScan-request>)))
  "Returns md5sum for a message object of type '<StartWifiScan-request>"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartWifiScan-request)))
  "Returns md5sum for a message object of type 'StartWifiScan-request"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartWifiScan-request>)))
  "Returns full string definition for message of type '<StartWifiScan-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartWifiScan-request)))
  "Returns full string definition for message of type 'StartWifiScan-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartWifiScan-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartWifiScan-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StartWifiScan-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude StartWifiScan-response.msg.html

(cl:defclass <StartWifiScan-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass StartWifiScan-response (<StartWifiScan-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartWifiScan-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartWifiScan-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<StartWifiScan-response> is deprecated: use kortex_driver-srv:StartWifiScan-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <StartWifiScan-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartWifiScan-response>) ostream)
  "Serializes a message object of type '<StartWifiScan-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartWifiScan-response>) istream)
  "Deserializes a message object of type '<StartWifiScan-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartWifiScan-response>)))
  "Returns string type for a service object of type '<StartWifiScan-response>"
  "kortex_driver/StartWifiScanResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartWifiScan-response)))
  "Returns string type for a service object of type 'StartWifiScan-response"
  "kortex_driver/StartWifiScanResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartWifiScan-response>)))
  "Returns md5sum for a message object of type '<StartWifiScan-response>"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartWifiScan-response)))
  "Returns md5sum for a message object of type 'StartWifiScan-response"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartWifiScan-response>)))
  "Returns full string definition for message of type '<StartWifiScan-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartWifiScan-response)))
  "Returns full string definition for message of type 'StartWifiScan-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartWifiScan-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartWifiScan-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StartWifiScan-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StartWifiScan)))
  'StartWifiScan-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StartWifiScan)))
  'StartWifiScan-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartWifiScan)))
  "Returns string type for a service object of type '<StartWifiScan>"
  "kortex_driver/StartWifiScan")