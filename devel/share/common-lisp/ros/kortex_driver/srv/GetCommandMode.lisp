; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetCommandMode-request.msg.html

(cl:defclass <GetCommandMode-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetCommandMode-request (<GetCommandMode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCommandMode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCommandMode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetCommandMode-request> is deprecated: use kortex_driver-srv:GetCommandMode-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetCommandMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCommandMode-request>) ostream)
  "Serializes a message object of type '<GetCommandMode-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCommandMode-request>) istream)
  "Deserializes a message object of type '<GetCommandMode-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCommandMode-request>)))
  "Returns string type for a service object of type '<GetCommandMode-request>"
  "kortex_driver/GetCommandModeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCommandMode-request)))
  "Returns string type for a service object of type 'GetCommandMode-request"
  "kortex_driver/GetCommandModeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCommandMode-request>)))
  "Returns md5sum for a message object of type '<GetCommandMode-request>"
  "6149f98eefd701a9f785c37c49326979")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCommandMode-request)))
  "Returns md5sum for a message object of type 'GetCommandMode-request"
  "6149f98eefd701a9f785c37c49326979")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCommandMode-request>)))
  "Returns full string definition for message of type '<GetCommandMode-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCommandMode-request)))
  "Returns full string definition for message of type 'GetCommandMode-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCommandMode-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCommandMode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCommandMode-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetCommandMode-response.msg.html

(cl:defclass <GetCommandMode-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:CommandModeInformation
    :initform (cl:make-instance 'kortex_driver-msg:CommandModeInformation)))
)

(cl:defclass GetCommandMode-response (<GetCommandMode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCommandMode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCommandMode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetCommandMode-response> is deprecated: use kortex_driver-srv:GetCommandMode-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetCommandMode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCommandMode-response>) ostream)
  "Serializes a message object of type '<GetCommandMode-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCommandMode-response>) istream)
  "Deserializes a message object of type '<GetCommandMode-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCommandMode-response>)))
  "Returns string type for a service object of type '<GetCommandMode-response>"
  "kortex_driver/GetCommandModeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCommandMode-response)))
  "Returns string type for a service object of type 'GetCommandMode-response"
  "kortex_driver/GetCommandModeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCommandMode-response>)))
  "Returns md5sum for a message object of type '<GetCommandMode-response>"
  "6149f98eefd701a9f785c37c49326979")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCommandMode-response)))
  "Returns md5sum for a message object of type 'GetCommandMode-response"
  "6149f98eefd701a9f785c37c49326979")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCommandMode-response>)))
  "Returns full string definition for message of type '<GetCommandMode-response>"
  (cl:format cl:nil "CommandModeInformation output~%~%================================================================================~%MSG: kortex_driver/CommandModeInformation~%~%uint32 command_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCommandMode-response)))
  "Returns full string definition for message of type 'GetCommandMode-response"
  (cl:format cl:nil "CommandModeInformation output~%~%================================================================================~%MSG: kortex_driver/CommandModeInformation~%~%uint32 command_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCommandMode-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCommandMode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCommandMode-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetCommandMode)))
  'GetCommandMode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetCommandMode)))
  'GetCommandMode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCommandMode)))
  "Returns string type for a service object of type '<GetCommandMode>"
  "kortex_driver/GetCommandMode")