; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetUARTConfiguration-request.msg.html

(cl:defclass <GetUARTConfiguration-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:UARTDeviceIdentification
    :initform (cl:make-instance 'kortex_driver-msg:UARTDeviceIdentification)))
)

(cl:defclass GetUARTConfiguration-request (<GetUARTConfiguration-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetUARTConfiguration-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetUARTConfiguration-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetUARTConfiguration-request> is deprecated: use kortex_driver-srv:GetUARTConfiguration-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetUARTConfiguration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetUARTConfiguration-request>) ostream)
  "Serializes a message object of type '<GetUARTConfiguration-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetUARTConfiguration-request>) istream)
  "Deserializes a message object of type '<GetUARTConfiguration-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetUARTConfiguration-request>)))
  "Returns string type for a service object of type '<GetUARTConfiguration-request>"
  "kortex_driver/GetUARTConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetUARTConfiguration-request)))
  "Returns string type for a service object of type 'GetUARTConfiguration-request"
  "kortex_driver/GetUARTConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetUARTConfiguration-request>)))
  "Returns md5sum for a message object of type '<GetUARTConfiguration-request>"
  "d041631488869c35b5eb142c6744c8a7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetUARTConfiguration-request)))
  "Returns md5sum for a message object of type 'GetUARTConfiguration-request"
  "d041631488869c35b5eb142c6744c8a7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetUARTConfiguration-request>)))
  "Returns full string definition for message of type '<GetUARTConfiguration-request>"
  (cl:format cl:nil "UARTDeviceIdentification input~%~%================================================================================~%MSG: kortex_driver/UARTDeviceIdentification~%~%uint32 port_id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetUARTConfiguration-request)))
  "Returns full string definition for message of type 'GetUARTConfiguration-request"
  (cl:format cl:nil "UARTDeviceIdentification input~%~%================================================================================~%MSG: kortex_driver/UARTDeviceIdentification~%~%uint32 port_id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetUARTConfiguration-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetUARTConfiguration-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetUARTConfiguration-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetUARTConfiguration-response.msg.html

(cl:defclass <GetUARTConfiguration-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:UARTConfiguration
    :initform (cl:make-instance 'kortex_driver-msg:UARTConfiguration)))
)

(cl:defclass GetUARTConfiguration-response (<GetUARTConfiguration-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetUARTConfiguration-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetUARTConfiguration-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetUARTConfiguration-response> is deprecated: use kortex_driver-srv:GetUARTConfiguration-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetUARTConfiguration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetUARTConfiguration-response>) ostream)
  "Serializes a message object of type '<GetUARTConfiguration-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetUARTConfiguration-response>) istream)
  "Deserializes a message object of type '<GetUARTConfiguration-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetUARTConfiguration-response>)))
  "Returns string type for a service object of type '<GetUARTConfiguration-response>"
  "kortex_driver/GetUARTConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetUARTConfiguration-response)))
  "Returns string type for a service object of type 'GetUARTConfiguration-response"
  "kortex_driver/GetUARTConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetUARTConfiguration-response>)))
  "Returns md5sum for a message object of type '<GetUARTConfiguration-response>"
  "d041631488869c35b5eb142c6744c8a7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetUARTConfiguration-response)))
  "Returns md5sum for a message object of type 'GetUARTConfiguration-response"
  "d041631488869c35b5eb142c6744c8a7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetUARTConfiguration-response>)))
  "Returns full string definition for message of type '<GetUARTConfiguration-response>"
  (cl:format cl:nil "UARTConfiguration output~%~%================================================================================~%MSG: kortex_driver/UARTConfiguration~%~%uint32 port_id~%bool enabled~%uint32 speed~%uint32 word_length~%uint32 stop_bits~%uint32 parity~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetUARTConfiguration-response)))
  "Returns full string definition for message of type 'GetUARTConfiguration-response"
  (cl:format cl:nil "UARTConfiguration output~%~%================================================================================~%MSG: kortex_driver/UARTConfiguration~%~%uint32 port_id~%bool enabled~%uint32 speed~%uint32 word_length~%uint32 stop_bits~%uint32 parity~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetUARTConfiguration-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetUARTConfiguration-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetUARTConfiguration-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetUARTConfiguration)))
  'GetUARTConfiguration-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetUARTConfiguration)))
  'GetUARTConfiguration-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetUARTConfiguration)))
  "Returns string type for a service object of type '<GetUARTConfiguration>"
  "kortex_driver/GetUARTConfiguration")