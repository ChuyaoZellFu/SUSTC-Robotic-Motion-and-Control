; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetBluetoothEnableState-request.msg.html

(cl:defclass <GetBluetoothEnableState-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetBluetoothEnableState-request (<GetBluetoothEnableState-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetBluetoothEnableState-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetBluetoothEnableState-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetBluetoothEnableState-request> is deprecated: use kortex_driver-srv:GetBluetoothEnableState-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetBluetoothEnableState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetBluetoothEnableState-request>) ostream)
  "Serializes a message object of type '<GetBluetoothEnableState-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetBluetoothEnableState-request>) istream)
  "Deserializes a message object of type '<GetBluetoothEnableState-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetBluetoothEnableState-request>)))
  "Returns string type for a service object of type '<GetBluetoothEnableState-request>"
  "kortex_driver/GetBluetoothEnableStateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetBluetoothEnableState-request)))
  "Returns string type for a service object of type 'GetBluetoothEnableState-request"
  "kortex_driver/GetBluetoothEnableStateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetBluetoothEnableState-request>)))
  "Returns md5sum for a message object of type '<GetBluetoothEnableState-request>"
  "9747040002a13b23ba7503e4b2f380fb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetBluetoothEnableState-request)))
  "Returns md5sum for a message object of type 'GetBluetoothEnableState-request"
  "9747040002a13b23ba7503e4b2f380fb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetBluetoothEnableState-request>)))
  "Returns full string definition for message of type '<GetBluetoothEnableState-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetBluetoothEnableState-request)))
  "Returns full string definition for message of type 'GetBluetoothEnableState-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetBluetoothEnableState-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetBluetoothEnableState-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetBluetoothEnableState-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetBluetoothEnableState-response.msg.html

(cl:defclass <GetBluetoothEnableState-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:BluetoothEnableState
    :initform (cl:make-instance 'kortex_driver-msg:BluetoothEnableState)))
)

(cl:defclass GetBluetoothEnableState-response (<GetBluetoothEnableState-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetBluetoothEnableState-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetBluetoothEnableState-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetBluetoothEnableState-response> is deprecated: use kortex_driver-srv:GetBluetoothEnableState-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetBluetoothEnableState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetBluetoothEnableState-response>) ostream)
  "Serializes a message object of type '<GetBluetoothEnableState-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetBluetoothEnableState-response>) istream)
  "Deserializes a message object of type '<GetBluetoothEnableState-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetBluetoothEnableState-response>)))
  "Returns string type for a service object of type '<GetBluetoothEnableState-response>"
  "kortex_driver/GetBluetoothEnableStateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetBluetoothEnableState-response)))
  "Returns string type for a service object of type 'GetBluetoothEnableState-response"
  "kortex_driver/GetBluetoothEnableStateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetBluetoothEnableState-response>)))
  "Returns md5sum for a message object of type '<GetBluetoothEnableState-response>"
  "9747040002a13b23ba7503e4b2f380fb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetBluetoothEnableState-response)))
  "Returns md5sum for a message object of type 'GetBluetoothEnableState-response"
  "9747040002a13b23ba7503e4b2f380fb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetBluetoothEnableState-response>)))
  "Returns full string definition for message of type '<GetBluetoothEnableState-response>"
  (cl:format cl:nil "BluetoothEnableState output~%~%================================================================================~%MSG: kortex_driver/BluetoothEnableState~%~%bool enabled~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetBluetoothEnableState-response)))
  "Returns full string definition for message of type 'GetBluetoothEnableState-response"
  (cl:format cl:nil "BluetoothEnableState output~%~%================================================================================~%MSG: kortex_driver/BluetoothEnableState~%~%bool enabled~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetBluetoothEnableState-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetBluetoothEnableState-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetBluetoothEnableState-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetBluetoothEnableState)))
  'GetBluetoothEnableState-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetBluetoothEnableState)))
  'GetBluetoothEnableState-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetBluetoothEnableState)))
  "Returns string type for a service object of type '<GetBluetoothEnableState>"
  "kortex_driver/GetBluetoothEnableState")