; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetCommandMode-request.msg.html

(cl:defclass <SetCommandMode-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:CommandModeInformation
    :initform (cl:make-instance 'kortex_driver-msg:CommandModeInformation)))
)

(cl:defclass SetCommandMode-request (<SetCommandMode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetCommandMode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetCommandMode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetCommandMode-request> is deprecated: use kortex_driver-srv:SetCommandMode-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetCommandMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetCommandMode-request>) ostream)
  "Serializes a message object of type '<SetCommandMode-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetCommandMode-request>) istream)
  "Deserializes a message object of type '<SetCommandMode-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetCommandMode-request>)))
  "Returns string type for a service object of type '<SetCommandMode-request>"
  "kortex_driver/SetCommandModeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCommandMode-request)))
  "Returns string type for a service object of type 'SetCommandMode-request"
  "kortex_driver/SetCommandModeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetCommandMode-request>)))
  "Returns md5sum for a message object of type '<SetCommandMode-request>"
  "ca50afb242dba49984abe55c3e0b6692")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetCommandMode-request)))
  "Returns md5sum for a message object of type 'SetCommandMode-request"
  "ca50afb242dba49984abe55c3e0b6692")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetCommandMode-request>)))
  "Returns full string definition for message of type '<SetCommandMode-request>"
  (cl:format cl:nil "CommandModeInformation input~%~%================================================================================~%MSG: kortex_driver/CommandModeInformation~%~%uint32 command_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetCommandMode-request)))
  "Returns full string definition for message of type 'SetCommandMode-request"
  (cl:format cl:nil "CommandModeInformation input~%~%================================================================================~%MSG: kortex_driver/CommandModeInformation~%~%uint32 command_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetCommandMode-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetCommandMode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetCommandMode-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetCommandMode-response.msg.html

(cl:defclass <SetCommandMode-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetCommandMode-response (<SetCommandMode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetCommandMode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetCommandMode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetCommandMode-response> is deprecated: use kortex_driver-srv:SetCommandMode-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetCommandMode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetCommandMode-response>) ostream)
  "Serializes a message object of type '<SetCommandMode-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetCommandMode-response>) istream)
  "Deserializes a message object of type '<SetCommandMode-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetCommandMode-response>)))
  "Returns string type for a service object of type '<SetCommandMode-response>"
  "kortex_driver/SetCommandModeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCommandMode-response)))
  "Returns string type for a service object of type 'SetCommandMode-response"
  "kortex_driver/SetCommandModeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetCommandMode-response>)))
  "Returns md5sum for a message object of type '<SetCommandMode-response>"
  "ca50afb242dba49984abe55c3e0b6692")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetCommandMode-response)))
  "Returns md5sum for a message object of type 'SetCommandMode-response"
  "ca50afb242dba49984abe55c3e0b6692")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetCommandMode-response>)))
  "Returns full string definition for message of type '<SetCommandMode-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetCommandMode-response)))
  "Returns full string definition for message of type 'SetCommandMode-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetCommandMode-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetCommandMode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetCommandMode-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetCommandMode)))
  'SetCommandMode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetCommandMode)))
  'SetCommandMode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCommandMode)))
  "Returns string type for a service object of type '<SetCommandMode>"
  "kortex_driver/SetCommandMode")