; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetEthernetConfiguration-request.msg.html

(cl:defclass <GetEthernetConfiguration-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:EthernetDeviceIdentification
    :initform (cl:make-instance 'kortex_driver-msg:EthernetDeviceIdentification)))
)

(cl:defclass GetEthernetConfiguration-request (<GetEthernetConfiguration-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetEthernetConfiguration-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetEthernetConfiguration-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetEthernetConfiguration-request> is deprecated: use kortex_driver-srv:GetEthernetConfiguration-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetEthernetConfiguration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetEthernetConfiguration-request>) ostream)
  "Serializes a message object of type '<GetEthernetConfiguration-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetEthernetConfiguration-request>) istream)
  "Deserializes a message object of type '<GetEthernetConfiguration-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetEthernetConfiguration-request>)))
  "Returns string type for a service object of type '<GetEthernetConfiguration-request>"
  "kortex_driver/GetEthernetConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetEthernetConfiguration-request)))
  "Returns string type for a service object of type 'GetEthernetConfiguration-request"
  "kortex_driver/GetEthernetConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetEthernetConfiguration-request>)))
  "Returns md5sum for a message object of type '<GetEthernetConfiguration-request>"
  "d50fb352c6ad553c74cb37db16e3d609")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetEthernetConfiguration-request)))
  "Returns md5sum for a message object of type 'GetEthernetConfiguration-request"
  "d50fb352c6ad553c74cb37db16e3d609")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetEthernetConfiguration-request>)))
  "Returns full string definition for message of type '<GetEthernetConfiguration-request>"
  (cl:format cl:nil "EthernetDeviceIdentification input~%~%================================================================================~%MSG: kortex_driver/EthernetDeviceIdentification~%~%uint32 device~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetEthernetConfiguration-request)))
  "Returns full string definition for message of type 'GetEthernetConfiguration-request"
  (cl:format cl:nil "EthernetDeviceIdentification input~%~%================================================================================~%MSG: kortex_driver/EthernetDeviceIdentification~%~%uint32 device~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetEthernetConfiguration-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetEthernetConfiguration-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetEthernetConfiguration-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetEthernetConfiguration-response.msg.html

(cl:defclass <GetEthernetConfiguration-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:EthernetConfiguration
    :initform (cl:make-instance 'kortex_driver-msg:EthernetConfiguration)))
)

(cl:defclass GetEthernetConfiguration-response (<GetEthernetConfiguration-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetEthernetConfiguration-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetEthernetConfiguration-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetEthernetConfiguration-response> is deprecated: use kortex_driver-srv:GetEthernetConfiguration-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetEthernetConfiguration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetEthernetConfiguration-response>) ostream)
  "Serializes a message object of type '<GetEthernetConfiguration-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetEthernetConfiguration-response>) istream)
  "Deserializes a message object of type '<GetEthernetConfiguration-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetEthernetConfiguration-response>)))
  "Returns string type for a service object of type '<GetEthernetConfiguration-response>"
  "kortex_driver/GetEthernetConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetEthernetConfiguration-response)))
  "Returns string type for a service object of type 'GetEthernetConfiguration-response"
  "kortex_driver/GetEthernetConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetEthernetConfiguration-response>)))
  "Returns md5sum for a message object of type '<GetEthernetConfiguration-response>"
  "d50fb352c6ad553c74cb37db16e3d609")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetEthernetConfiguration-response)))
  "Returns md5sum for a message object of type 'GetEthernetConfiguration-response"
  "d50fb352c6ad553c74cb37db16e3d609")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetEthernetConfiguration-response>)))
  "Returns full string definition for message of type '<GetEthernetConfiguration-response>"
  (cl:format cl:nil "EthernetConfiguration output~%~%================================================================================~%MSG: kortex_driver/EthernetConfiguration~%~%uint32 device~%bool enabled~%uint32 speed~%uint32 duplex~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetEthernetConfiguration-response)))
  "Returns full string definition for message of type 'GetEthernetConfiguration-response"
  (cl:format cl:nil "EthernetConfiguration output~%~%================================================================================~%MSG: kortex_driver/EthernetConfiguration~%~%uint32 device~%bool enabled~%uint32 speed~%uint32 duplex~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetEthernetConfiguration-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetEthernetConfiguration-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetEthernetConfiguration-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetEthernetConfiguration)))
  'GetEthernetConfiguration-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetEthernetConfiguration)))
  'GetEthernetConfiguration-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetEthernetConfiguration)))
  "Returns string type for a service object of type '<GetEthernetConfiguration>"
  "kortex_driver/GetEthernetConfiguration")