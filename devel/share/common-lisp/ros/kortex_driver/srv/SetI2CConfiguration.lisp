; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetI2CConfiguration-request.msg.html

(cl:defclass <SetI2CConfiguration-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:I2CConfiguration
    :initform (cl:make-instance 'kortex_driver-msg:I2CConfiguration)))
)

(cl:defclass SetI2CConfiguration-request (<SetI2CConfiguration-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetI2CConfiguration-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetI2CConfiguration-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetI2CConfiguration-request> is deprecated: use kortex_driver-srv:SetI2CConfiguration-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetI2CConfiguration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetI2CConfiguration-request>) ostream)
  "Serializes a message object of type '<SetI2CConfiguration-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetI2CConfiguration-request>) istream)
  "Deserializes a message object of type '<SetI2CConfiguration-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetI2CConfiguration-request>)))
  "Returns string type for a service object of type '<SetI2CConfiguration-request>"
  "kortex_driver/SetI2CConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetI2CConfiguration-request)))
  "Returns string type for a service object of type 'SetI2CConfiguration-request"
  "kortex_driver/SetI2CConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetI2CConfiguration-request>)))
  "Returns md5sum for a message object of type '<SetI2CConfiguration-request>"
  "fcaaa1a0d5ba74e939b1b6f1a0988c2a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetI2CConfiguration-request)))
  "Returns md5sum for a message object of type 'SetI2CConfiguration-request"
  "fcaaa1a0d5ba74e939b1b6f1a0988c2a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetI2CConfiguration-request>)))
  "Returns full string definition for message of type '<SetI2CConfiguration-request>"
  (cl:format cl:nil "I2CConfiguration input~%~%================================================================================~%MSG: kortex_driver/I2CConfiguration~%~%uint32 device~%bool enabled~%uint32 mode~%uint32 addressing~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetI2CConfiguration-request)))
  "Returns full string definition for message of type 'SetI2CConfiguration-request"
  (cl:format cl:nil "I2CConfiguration input~%~%================================================================================~%MSG: kortex_driver/I2CConfiguration~%~%uint32 device~%bool enabled~%uint32 mode~%uint32 addressing~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetI2CConfiguration-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetI2CConfiguration-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetI2CConfiguration-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetI2CConfiguration-response.msg.html

(cl:defclass <SetI2CConfiguration-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetI2CConfiguration-response (<SetI2CConfiguration-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetI2CConfiguration-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetI2CConfiguration-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetI2CConfiguration-response> is deprecated: use kortex_driver-srv:SetI2CConfiguration-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetI2CConfiguration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetI2CConfiguration-response>) ostream)
  "Serializes a message object of type '<SetI2CConfiguration-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetI2CConfiguration-response>) istream)
  "Deserializes a message object of type '<SetI2CConfiguration-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetI2CConfiguration-response>)))
  "Returns string type for a service object of type '<SetI2CConfiguration-response>"
  "kortex_driver/SetI2CConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetI2CConfiguration-response)))
  "Returns string type for a service object of type 'SetI2CConfiguration-response"
  "kortex_driver/SetI2CConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetI2CConfiguration-response>)))
  "Returns md5sum for a message object of type '<SetI2CConfiguration-response>"
  "fcaaa1a0d5ba74e939b1b6f1a0988c2a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetI2CConfiguration-response)))
  "Returns md5sum for a message object of type 'SetI2CConfiguration-response"
  "fcaaa1a0d5ba74e939b1b6f1a0988c2a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetI2CConfiguration-response>)))
  "Returns full string definition for message of type '<SetI2CConfiguration-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetI2CConfiguration-response)))
  "Returns full string definition for message of type 'SetI2CConfiguration-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetI2CConfiguration-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetI2CConfiguration-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetI2CConfiguration-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetI2CConfiguration)))
  'SetI2CConfiguration-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetI2CConfiguration)))
  'SetI2CConfiguration-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetI2CConfiguration)))
  "Returns string type for a service object of type '<SetI2CConfiguration>"
  "kortex_driver/SetI2CConfiguration")