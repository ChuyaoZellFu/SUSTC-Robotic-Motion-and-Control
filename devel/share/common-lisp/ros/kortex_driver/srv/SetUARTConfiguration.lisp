; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetUARTConfiguration-request.msg.html

(cl:defclass <SetUARTConfiguration-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:UARTConfiguration
    :initform (cl:make-instance 'kortex_driver-msg:UARTConfiguration)))
)

(cl:defclass SetUARTConfiguration-request (<SetUARTConfiguration-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetUARTConfiguration-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetUARTConfiguration-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetUARTConfiguration-request> is deprecated: use kortex_driver-srv:SetUARTConfiguration-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetUARTConfiguration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetUARTConfiguration-request>) ostream)
  "Serializes a message object of type '<SetUARTConfiguration-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetUARTConfiguration-request>) istream)
  "Deserializes a message object of type '<SetUARTConfiguration-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetUARTConfiguration-request>)))
  "Returns string type for a service object of type '<SetUARTConfiguration-request>"
  "kortex_driver/SetUARTConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetUARTConfiguration-request)))
  "Returns string type for a service object of type 'SetUARTConfiguration-request"
  "kortex_driver/SetUARTConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetUARTConfiguration-request>)))
  "Returns md5sum for a message object of type '<SetUARTConfiguration-request>"
  "44945f03b0c880ed6bb57eedba493047")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetUARTConfiguration-request)))
  "Returns md5sum for a message object of type 'SetUARTConfiguration-request"
  "44945f03b0c880ed6bb57eedba493047")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetUARTConfiguration-request>)))
  "Returns full string definition for message of type '<SetUARTConfiguration-request>"
  (cl:format cl:nil "UARTConfiguration input~%~%================================================================================~%MSG: kortex_driver/UARTConfiguration~%~%uint32 port_id~%bool enabled~%uint32 speed~%uint32 word_length~%uint32 stop_bits~%uint32 parity~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetUARTConfiguration-request)))
  "Returns full string definition for message of type 'SetUARTConfiguration-request"
  (cl:format cl:nil "UARTConfiguration input~%~%================================================================================~%MSG: kortex_driver/UARTConfiguration~%~%uint32 port_id~%bool enabled~%uint32 speed~%uint32 word_length~%uint32 stop_bits~%uint32 parity~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetUARTConfiguration-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetUARTConfiguration-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetUARTConfiguration-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetUARTConfiguration-response.msg.html

(cl:defclass <SetUARTConfiguration-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetUARTConfiguration-response (<SetUARTConfiguration-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetUARTConfiguration-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetUARTConfiguration-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetUARTConfiguration-response> is deprecated: use kortex_driver-srv:SetUARTConfiguration-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetUARTConfiguration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetUARTConfiguration-response>) ostream)
  "Serializes a message object of type '<SetUARTConfiguration-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetUARTConfiguration-response>) istream)
  "Deserializes a message object of type '<SetUARTConfiguration-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetUARTConfiguration-response>)))
  "Returns string type for a service object of type '<SetUARTConfiguration-response>"
  "kortex_driver/SetUARTConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetUARTConfiguration-response)))
  "Returns string type for a service object of type 'SetUARTConfiguration-response"
  "kortex_driver/SetUARTConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetUARTConfiguration-response>)))
  "Returns md5sum for a message object of type '<SetUARTConfiguration-response>"
  "44945f03b0c880ed6bb57eedba493047")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetUARTConfiguration-response)))
  "Returns md5sum for a message object of type 'SetUARTConfiguration-response"
  "44945f03b0c880ed6bb57eedba493047")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetUARTConfiguration-response>)))
  "Returns full string definition for message of type '<SetUARTConfiguration-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetUARTConfiguration-response)))
  "Returns full string definition for message of type 'SetUARTConfiguration-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetUARTConfiguration-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetUARTConfiguration-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetUARTConfiguration-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetUARTConfiguration)))
  'SetUARTConfiguration-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetUARTConfiguration)))
  'SetUARTConfiguration-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetUARTConfiguration)))
  "Returns string type for a service object of type '<SetUARTConfiguration>"
  "kortex_driver/SetUARTConfiguration")