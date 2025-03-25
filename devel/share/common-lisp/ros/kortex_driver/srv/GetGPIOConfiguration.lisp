; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetGPIOConfiguration-request.msg.html

(cl:defclass <GetGPIOConfiguration-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:GPIOIdentification
    :initform (cl:make-instance 'kortex_driver-msg:GPIOIdentification)))
)

(cl:defclass GetGPIOConfiguration-request (<GetGPIOConfiguration-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetGPIOConfiguration-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetGPIOConfiguration-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetGPIOConfiguration-request> is deprecated: use kortex_driver-srv:GetGPIOConfiguration-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetGPIOConfiguration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetGPIOConfiguration-request>) ostream)
  "Serializes a message object of type '<GetGPIOConfiguration-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetGPIOConfiguration-request>) istream)
  "Deserializes a message object of type '<GetGPIOConfiguration-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetGPIOConfiguration-request>)))
  "Returns string type for a service object of type '<GetGPIOConfiguration-request>"
  "kortex_driver/GetGPIOConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetGPIOConfiguration-request)))
  "Returns string type for a service object of type 'GetGPIOConfiguration-request"
  "kortex_driver/GetGPIOConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetGPIOConfiguration-request>)))
  "Returns md5sum for a message object of type '<GetGPIOConfiguration-request>"
  "54322e7ee9a9cb34b1b8f34c41c5ae4d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetGPIOConfiguration-request)))
  "Returns md5sum for a message object of type 'GetGPIOConfiguration-request"
  "54322e7ee9a9cb34b1b8f34c41c5ae4d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetGPIOConfiguration-request>)))
  "Returns full string definition for message of type '<GetGPIOConfiguration-request>"
  (cl:format cl:nil "GPIOIdentification input~%~%================================================================================~%MSG: kortex_driver/GPIOIdentification~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetGPIOConfiguration-request)))
  "Returns full string definition for message of type 'GetGPIOConfiguration-request"
  (cl:format cl:nil "GPIOIdentification input~%~%================================================================================~%MSG: kortex_driver/GPIOIdentification~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetGPIOConfiguration-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetGPIOConfiguration-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetGPIOConfiguration-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetGPIOConfiguration-response.msg.html

(cl:defclass <GetGPIOConfiguration-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:InterconnectConfig_GPIOConfiguration
    :initform (cl:make-instance 'kortex_driver-msg:InterconnectConfig_GPIOConfiguration)))
)

(cl:defclass GetGPIOConfiguration-response (<GetGPIOConfiguration-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetGPIOConfiguration-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetGPIOConfiguration-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetGPIOConfiguration-response> is deprecated: use kortex_driver-srv:GetGPIOConfiguration-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetGPIOConfiguration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetGPIOConfiguration-response>) ostream)
  "Serializes a message object of type '<GetGPIOConfiguration-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetGPIOConfiguration-response>) istream)
  "Deserializes a message object of type '<GetGPIOConfiguration-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetGPIOConfiguration-response>)))
  "Returns string type for a service object of type '<GetGPIOConfiguration-response>"
  "kortex_driver/GetGPIOConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetGPIOConfiguration-response)))
  "Returns string type for a service object of type 'GetGPIOConfiguration-response"
  "kortex_driver/GetGPIOConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetGPIOConfiguration-response>)))
  "Returns md5sum for a message object of type '<GetGPIOConfiguration-response>"
  "54322e7ee9a9cb34b1b8f34c41c5ae4d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetGPIOConfiguration-response)))
  "Returns md5sum for a message object of type 'GetGPIOConfiguration-response"
  "54322e7ee9a9cb34b1b8f34c41c5ae4d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetGPIOConfiguration-response>)))
  "Returns full string definition for message of type '<GetGPIOConfiguration-response>"
  (cl:format cl:nil "InterconnectConfig_GPIOConfiguration output~%~%================================================================================~%MSG: kortex_driver/InterconnectConfig_GPIOConfiguration~%~%uint32 identifier~%uint32 mode~%uint32 pull~%uint32 default_value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetGPIOConfiguration-response)))
  "Returns full string definition for message of type 'GetGPIOConfiguration-response"
  (cl:format cl:nil "InterconnectConfig_GPIOConfiguration output~%~%================================================================================~%MSG: kortex_driver/InterconnectConfig_GPIOConfiguration~%~%uint32 identifier~%uint32 mode~%uint32 pull~%uint32 default_value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetGPIOConfiguration-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetGPIOConfiguration-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetGPIOConfiguration-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetGPIOConfiguration)))
  'GetGPIOConfiguration-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetGPIOConfiguration)))
  'GetGPIOConfiguration-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetGPIOConfiguration)))
  "Returns string type for a service object of type '<GetGPIOConfiguration>"
  "kortex_driver/GetGPIOConfiguration")