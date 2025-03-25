; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetEthernetConfiguration-request.msg.html

(cl:defclass <SetEthernetConfiguration-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:EthernetConfiguration
    :initform (cl:make-instance 'kortex_driver-msg:EthernetConfiguration)))
)

(cl:defclass SetEthernetConfiguration-request (<SetEthernetConfiguration-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetEthernetConfiguration-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetEthernetConfiguration-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetEthernetConfiguration-request> is deprecated: use kortex_driver-srv:SetEthernetConfiguration-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetEthernetConfiguration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetEthernetConfiguration-request>) ostream)
  "Serializes a message object of type '<SetEthernetConfiguration-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetEthernetConfiguration-request>) istream)
  "Deserializes a message object of type '<SetEthernetConfiguration-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetEthernetConfiguration-request>)))
  "Returns string type for a service object of type '<SetEthernetConfiguration-request>"
  "kortex_driver/SetEthernetConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetEthernetConfiguration-request)))
  "Returns string type for a service object of type 'SetEthernetConfiguration-request"
  "kortex_driver/SetEthernetConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetEthernetConfiguration-request>)))
  "Returns md5sum for a message object of type '<SetEthernetConfiguration-request>"
  "fdfcb3d5cd015a2a65d4aee878ff80be")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetEthernetConfiguration-request)))
  "Returns md5sum for a message object of type 'SetEthernetConfiguration-request"
  "fdfcb3d5cd015a2a65d4aee878ff80be")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetEthernetConfiguration-request>)))
  "Returns full string definition for message of type '<SetEthernetConfiguration-request>"
  (cl:format cl:nil "EthernetConfiguration input~%~%================================================================================~%MSG: kortex_driver/EthernetConfiguration~%~%uint32 device~%bool enabled~%uint32 speed~%uint32 duplex~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetEthernetConfiguration-request)))
  "Returns full string definition for message of type 'SetEthernetConfiguration-request"
  (cl:format cl:nil "EthernetConfiguration input~%~%================================================================================~%MSG: kortex_driver/EthernetConfiguration~%~%uint32 device~%bool enabled~%uint32 speed~%uint32 duplex~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetEthernetConfiguration-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetEthernetConfiguration-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetEthernetConfiguration-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetEthernetConfiguration-response.msg.html

(cl:defclass <SetEthernetConfiguration-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetEthernetConfiguration-response (<SetEthernetConfiguration-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetEthernetConfiguration-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetEthernetConfiguration-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetEthernetConfiguration-response> is deprecated: use kortex_driver-srv:SetEthernetConfiguration-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetEthernetConfiguration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetEthernetConfiguration-response>) ostream)
  "Serializes a message object of type '<SetEthernetConfiguration-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetEthernetConfiguration-response>) istream)
  "Deserializes a message object of type '<SetEthernetConfiguration-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetEthernetConfiguration-response>)))
  "Returns string type for a service object of type '<SetEthernetConfiguration-response>"
  "kortex_driver/SetEthernetConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetEthernetConfiguration-response)))
  "Returns string type for a service object of type 'SetEthernetConfiguration-response"
  "kortex_driver/SetEthernetConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetEthernetConfiguration-response>)))
  "Returns md5sum for a message object of type '<SetEthernetConfiguration-response>"
  "fdfcb3d5cd015a2a65d4aee878ff80be")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetEthernetConfiguration-response)))
  "Returns md5sum for a message object of type 'SetEthernetConfiguration-response"
  "fdfcb3d5cd015a2a65d4aee878ff80be")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetEthernetConfiguration-response>)))
  "Returns full string definition for message of type '<SetEthernetConfiguration-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetEthernetConfiguration-response)))
  "Returns full string definition for message of type 'SetEthernetConfiguration-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetEthernetConfiguration-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetEthernetConfiguration-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetEthernetConfiguration-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetEthernetConfiguration)))
  'SetEthernetConfiguration-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetEthernetConfiguration)))
  'SetEthernetConfiguration-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetEthernetConfiguration)))
  "Returns string type for a service object of type '<SetEthernetConfiguration>"
  "kortex_driver/SetEthernetConfiguration")