; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude IsCommunicationInterfaceEnable-request.msg.html

(cl:defclass <IsCommunicationInterfaceEnable-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:NetworkHandle
    :initform (cl:make-instance 'kortex_driver-msg:NetworkHandle)))
)

(cl:defclass IsCommunicationInterfaceEnable-request (<IsCommunicationInterfaceEnable-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IsCommunicationInterfaceEnable-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IsCommunicationInterfaceEnable-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<IsCommunicationInterfaceEnable-request> is deprecated: use kortex_driver-srv:IsCommunicationInterfaceEnable-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <IsCommunicationInterfaceEnable-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IsCommunicationInterfaceEnable-request>) ostream)
  "Serializes a message object of type '<IsCommunicationInterfaceEnable-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IsCommunicationInterfaceEnable-request>) istream)
  "Deserializes a message object of type '<IsCommunicationInterfaceEnable-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IsCommunicationInterfaceEnable-request>)))
  "Returns string type for a service object of type '<IsCommunicationInterfaceEnable-request>"
  "kortex_driver/IsCommunicationInterfaceEnableRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IsCommunicationInterfaceEnable-request)))
  "Returns string type for a service object of type 'IsCommunicationInterfaceEnable-request"
  "kortex_driver/IsCommunicationInterfaceEnableRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IsCommunicationInterfaceEnable-request>)))
  "Returns md5sum for a message object of type '<IsCommunicationInterfaceEnable-request>"
  "7b7c3f92182fedb31e77cfcc39090ac1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IsCommunicationInterfaceEnable-request)))
  "Returns md5sum for a message object of type 'IsCommunicationInterfaceEnable-request"
  "7b7c3f92182fedb31e77cfcc39090ac1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IsCommunicationInterfaceEnable-request>)))
  "Returns full string definition for message of type '<IsCommunicationInterfaceEnable-request>"
  (cl:format cl:nil "NetworkHandle input~%~%================================================================================~%MSG: kortex_driver/NetworkHandle~%~%uint32 type~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IsCommunicationInterfaceEnable-request)))
  "Returns full string definition for message of type 'IsCommunicationInterfaceEnable-request"
  (cl:format cl:nil "NetworkHandle input~%~%================================================================================~%MSG: kortex_driver/NetworkHandle~%~%uint32 type~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IsCommunicationInterfaceEnable-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IsCommunicationInterfaceEnable-request>))
  "Converts a ROS message object to a list"
  (cl:list 'IsCommunicationInterfaceEnable-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude IsCommunicationInterfaceEnable-response.msg.html

(cl:defclass <IsCommunicationInterfaceEnable-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:CommunicationInterfaceConfiguration
    :initform (cl:make-instance 'kortex_driver-msg:CommunicationInterfaceConfiguration)))
)

(cl:defclass IsCommunicationInterfaceEnable-response (<IsCommunicationInterfaceEnable-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IsCommunicationInterfaceEnable-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IsCommunicationInterfaceEnable-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<IsCommunicationInterfaceEnable-response> is deprecated: use kortex_driver-srv:IsCommunicationInterfaceEnable-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <IsCommunicationInterfaceEnable-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IsCommunicationInterfaceEnable-response>) ostream)
  "Serializes a message object of type '<IsCommunicationInterfaceEnable-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IsCommunicationInterfaceEnable-response>) istream)
  "Deserializes a message object of type '<IsCommunicationInterfaceEnable-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IsCommunicationInterfaceEnable-response>)))
  "Returns string type for a service object of type '<IsCommunicationInterfaceEnable-response>"
  "kortex_driver/IsCommunicationInterfaceEnableResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IsCommunicationInterfaceEnable-response)))
  "Returns string type for a service object of type 'IsCommunicationInterfaceEnable-response"
  "kortex_driver/IsCommunicationInterfaceEnableResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IsCommunicationInterfaceEnable-response>)))
  "Returns md5sum for a message object of type '<IsCommunicationInterfaceEnable-response>"
  "7b7c3f92182fedb31e77cfcc39090ac1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IsCommunicationInterfaceEnable-response)))
  "Returns md5sum for a message object of type 'IsCommunicationInterfaceEnable-response"
  "7b7c3f92182fedb31e77cfcc39090ac1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IsCommunicationInterfaceEnable-response>)))
  "Returns full string definition for message of type '<IsCommunicationInterfaceEnable-response>"
  (cl:format cl:nil "CommunicationInterfaceConfiguration output~%~%================================================================================~%MSG: kortex_driver/CommunicationInterfaceConfiguration~%~%uint32 type~%bool enable~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IsCommunicationInterfaceEnable-response)))
  "Returns full string definition for message of type 'IsCommunicationInterfaceEnable-response"
  (cl:format cl:nil "CommunicationInterfaceConfiguration output~%~%================================================================================~%MSG: kortex_driver/CommunicationInterfaceConfiguration~%~%uint32 type~%bool enable~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IsCommunicationInterfaceEnable-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IsCommunicationInterfaceEnable-response>))
  "Converts a ROS message object to a list"
  (cl:list 'IsCommunicationInterfaceEnable-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'IsCommunicationInterfaceEnable)))
  'IsCommunicationInterfaceEnable-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'IsCommunicationInterfaceEnable)))
  'IsCommunicationInterfaceEnable-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IsCommunicationInterfaceEnable)))
  "Returns string type for a service object of type '<IsCommunicationInterfaceEnable>"
  "kortex_driver/IsCommunicationInterfaceEnable")