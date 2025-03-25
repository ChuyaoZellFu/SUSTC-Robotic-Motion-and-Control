; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetCommunicationInterfaceEnable-request.msg.html

(cl:defclass <SetCommunicationInterfaceEnable-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:CommunicationInterfaceConfiguration
    :initform (cl:make-instance 'kortex_driver-msg:CommunicationInterfaceConfiguration)))
)

(cl:defclass SetCommunicationInterfaceEnable-request (<SetCommunicationInterfaceEnable-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetCommunicationInterfaceEnable-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetCommunicationInterfaceEnable-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetCommunicationInterfaceEnable-request> is deprecated: use kortex_driver-srv:SetCommunicationInterfaceEnable-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetCommunicationInterfaceEnable-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetCommunicationInterfaceEnable-request>) ostream)
  "Serializes a message object of type '<SetCommunicationInterfaceEnable-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetCommunicationInterfaceEnable-request>) istream)
  "Deserializes a message object of type '<SetCommunicationInterfaceEnable-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetCommunicationInterfaceEnable-request>)))
  "Returns string type for a service object of type '<SetCommunicationInterfaceEnable-request>"
  "kortex_driver/SetCommunicationInterfaceEnableRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCommunicationInterfaceEnable-request)))
  "Returns string type for a service object of type 'SetCommunicationInterfaceEnable-request"
  "kortex_driver/SetCommunicationInterfaceEnableRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetCommunicationInterfaceEnable-request>)))
  "Returns md5sum for a message object of type '<SetCommunicationInterfaceEnable-request>"
  "ea61eebb1ea7afcd99f7fa2e0cb8d9db")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetCommunicationInterfaceEnable-request)))
  "Returns md5sum for a message object of type 'SetCommunicationInterfaceEnable-request"
  "ea61eebb1ea7afcd99f7fa2e0cb8d9db")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetCommunicationInterfaceEnable-request>)))
  "Returns full string definition for message of type '<SetCommunicationInterfaceEnable-request>"
  (cl:format cl:nil "CommunicationInterfaceConfiguration input~%~%================================================================================~%MSG: kortex_driver/CommunicationInterfaceConfiguration~%~%uint32 type~%bool enable~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetCommunicationInterfaceEnable-request)))
  "Returns full string definition for message of type 'SetCommunicationInterfaceEnable-request"
  (cl:format cl:nil "CommunicationInterfaceConfiguration input~%~%================================================================================~%MSG: kortex_driver/CommunicationInterfaceConfiguration~%~%uint32 type~%bool enable~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetCommunicationInterfaceEnable-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetCommunicationInterfaceEnable-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetCommunicationInterfaceEnable-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetCommunicationInterfaceEnable-response.msg.html

(cl:defclass <SetCommunicationInterfaceEnable-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetCommunicationInterfaceEnable-response (<SetCommunicationInterfaceEnable-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetCommunicationInterfaceEnable-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetCommunicationInterfaceEnable-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetCommunicationInterfaceEnable-response> is deprecated: use kortex_driver-srv:SetCommunicationInterfaceEnable-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetCommunicationInterfaceEnable-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetCommunicationInterfaceEnable-response>) ostream)
  "Serializes a message object of type '<SetCommunicationInterfaceEnable-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetCommunicationInterfaceEnable-response>) istream)
  "Deserializes a message object of type '<SetCommunicationInterfaceEnable-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetCommunicationInterfaceEnable-response>)))
  "Returns string type for a service object of type '<SetCommunicationInterfaceEnable-response>"
  "kortex_driver/SetCommunicationInterfaceEnableResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCommunicationInterfaceEnable-response)))
  "Returns string type for a service object of type 'SetCommunicationInterfaceEnable-response"
  "kortex_driver/SetCommunicationInterfaceEnableResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetCommunicationInterfaceEnable-response>)))
  "Returns md5sum for a message object of type '<SetCommunicationInterfaceEnable-response>"
  "ea61eebb1ea7afcd99f7fa2e0cb8d9db")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetCommunicationInterfaceEnable-response)))
  "Returns md5sum for a message object of type 'SetCommunicationInterfaceEnable-response"
  "ea61eebb1ea7afcd99f7fa2e0cb8d9db")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetCommunicationInterfaceEnable-response>)))
  "Returns full string definition for message of type '<SetCommunicationInterfaceEnable-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetCommunicationInterfaceEnable-response)))
  "Returns full string definition for message of type 'SetCommunicationInterfaceEnable-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetCommunicationInterfaceEnable-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetCommunicationInterfaceEnable-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetCommunicationInterfaceEnable-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetCommunicationInterfaceEnable)))
  'SetCommunicationInterfaceEnable-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetCommunicationInterfaceEnable)))
  'SetCommunicationInterfaceEnable-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCommunicationInterfaceEnable)))
  "Returns string type for a service object of type '<SetCommunicationInterfaceEnable>"
  "kortex_driver/SetCommunicationInterfaceEnable")