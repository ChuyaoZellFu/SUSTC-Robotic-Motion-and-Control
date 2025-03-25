; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetGPIOConfiguration-request.msg.html

(cl:defclass <SetGPIOConfiguration-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:InterconnectConfig_GPIOConfiguration
    :initform (cl:make-instance 'kortex_driver-msg:InterconnectConfig_GPIOConfiguration)))
)

(cl:defclass SetGPIOConfiguration-request (<SetGPIOConfiguration-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetGPIOConfiguration-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetGPIOConfiguration-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetGPIOConfiguration-request> is deprecated: use kortex_driver-srv:SetGPIOConfiguration-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetGPIOConfiguration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetGPIOConfiguration-request>) ostream)
  "Serializes a message object of type '<SetGPIOConfiguration-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetGPIOConfiguration-request>) istream)
  "Deserializes a message object of type '<SetGPIOConfiguration-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetGPIOConfiguration-request>)))
  "Returns string type for a service object of type '<SetGPIOConfiguration-request>"
  "kortex_driver/SetGPIOConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetGPIOConfiguration-request)))
  "Returns string type for a service object of type 'SetGPIOConfiguration-request"
  "kortex_driver/SetGPIOConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetGPIOConfiguration-request>)))
  "Returns md5sum for a message object of type '<SetGPIOConfiguration-request>"
  "c67d61eec82191ca0955210766f4f427")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetGPIOConfiguration-request)))
  "Returns md5sum for a message object of type 'SetGPIOConfiguration-request"
  "c67d61eec82191ca0955210766f4f427")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetGPIOConfiguration-request>)))
  "Returns full string definition for message of type '<SetGPIOConfiguration-request>"
  (cl:format cl:nil "InterconnectConfig_GPIOConfiguration input~%~%================================================================================~%MSG: kortex_driver/InterconnectConfig_GPIOConfiguration~%~%uint32 identifier~%uint32 mode~%uint32 pull~%uint32 default_value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetGPIOConfiguration-request)))
  "Returns full string definition for message of type 'SetGPIOConfiguration-request"
  (cl:format cl:nil "InterconnectConfig_GPIOConfiguration input~%~%================================================================================~%MSG: kortex_driver/InterconnectConfig_GPIOConfiguration~%~%uint32 identifier~%uint32 mode~%uint32 pull~%uint32 default_value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetGPIOConfiguration-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetGPIOConfiguration-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetGPIOConfiguration-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetGPIOConfiguration-response.msg.html

(cl:defclass <SetGPIOConfiguration-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetGPIOConfiguration-response (<SetGPIOConfiguration-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetGPIOConfiguration-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetGPIOConfiguration-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetGPIOConfiguration-response> is deprecated: use kortex_driver-srv:SetGPIOConfiguration-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetGPIOConfiguration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetGPIOConfiguration-response>) ostream)
  "Serializes a message object of type '<SetGPIOConfiguration-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetGPIOConfiguration-response>) istream)
  "Deserializes a message object of type '<SetGPIOConfiguration-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetGPIOConfiguration-response>)))
  "Returns string type for a service object of type '<SetGPIOConfiguration-response>"
  "kortex_driver/SetGPIOConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetGPIOConfiguration-response)))
  "Returns string type for a service object of type 'SetGPIOConfiguration-response"
  "kortex_driver/SetGPIOConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetGPIOConfiguration-response>)))
  "Returns md5sum for a message object of type '<SetGPIOConfiguration-response>"
  "c67d61eec82191ca0955210766f4f427")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetGPIOConfiguration-response)))
  "Returns md5sum for a message object of type 'SetGPIOConfiguration-response"
  "c67d61eec82191ca0955210766f4f427")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetGPIOConfiguration-response>)))
  "Returns full string definition for message of type '<SetGPIOConfiguration-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetGPIOConfiguration-response)))
  "Returns full string definition for message of type 'SetGPIOConfiguration-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetGPIOConfiguration-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetGPIOConfiguration-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetGPIOConfiguration-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetGPIOConfiguration)))
  'SetGPIOConfiguration-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetGPIOConfiguration)))
  'SetGPIOConfiguration-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetGPIOConfiguration)))
  "Returns string type for a service object of type '<SetGPIOConfiguration>"
  "kortex_driver/SetGPIOConfiguration")