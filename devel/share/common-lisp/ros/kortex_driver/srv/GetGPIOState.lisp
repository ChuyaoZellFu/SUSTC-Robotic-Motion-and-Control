; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetGPIOState-request.msg.html

(cl:defclass <GetGPIOState-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:GPIOIdentification
    :initform (cl:make-instance 'kortex_driver-msg:GPIOIdentification)))
)

(cl:defclass GetGPIOState-request (<GetGPIOState-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetGPIOState-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetGPIOState-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetGPIOState-request> is deprecated: use kortex_driver-srv:GetGPIOState-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetGPIOState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetGPIOState-request>) ostream)
  "Serializes a message object of type '<GetGPIOState-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetGPIOState-request>) istream)
  "Deserializes a message object of type '<GetGPIOState-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetGPIOState-request>)))
  "Returns string type for a service object of type '<GetGPIOState-request>"
  "kortex_driver/GetGPIOStateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetGPIOState-request)))
  "Returns string type for a service object of type 'GetGPIOState-request"
  "kortex_driver/GetGPIOStateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetGPIOState-request>)))
  "Returns md5sum for a message object of type '<GetGPIOState-request>"
  "ee1aa8c7cecf549eed7751c07a3eb88c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetGPIOState-request)))
  "Returns md5sum for a message object of type 'GetGPIOState-request"
  "ee1aa8c7cecf549eed7751c07a3eb88c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetGPIOState-request>)))
  "Returns full string definition for message of type '<GetGPIOState-request>"
  (cl:format cl:nil "GPIOIdentification input~%~%================================================================================~%MSG: kortex_driver/GPIOIdentification~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetGPIOState-request)))
  "Returns full string definition for message of type 'GetGPIOState-request"
  (cl:format cl:nil "GPIOIdentification input~%~%================================================================================~%MSG: kortex_driver/GPIOIdentification~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetGPIOState-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetGPIOState-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetGPIOState-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetGPIOState-response.msg.html

(cl:defclass <GetGPIOState-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:GPIOState
    :initform (cl:make-instance 'kortex_driver-msg:GPIOState)))
)

(cl:defclass GetGPIOState-response (<GetGPIOState-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetGPIOState-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetGPIOState-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetGPIOState-response> is deprecated: use kortex_driver-srv:GetGPIOState-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetGPIOState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetGPIOState-response>) ostream)
  "Serializes a message object of type '<GetGPIOState-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetGPIOState-response>) istream)
  "Deserializes a message object of type '<GetGPIOState-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetGPIOState-response>)))
  "Returns string type for a service object of type '<GetGPIOState-response>"
  "kortex_driver/GetGPIOStateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetGPIOState-response)))
  "Returns string type for a service object of type 'GetGPIOState-response"
  "kortex_driver/GetGPIOStateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetGPIOState-response>)))
  "Returns md5sum for a message object of type '<GetGPIOState-response>"
  "ee1aa8c7cecf549eed7751c07a3eb88c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetGPIOState-response)))
  "Returns md5sum for a message object of type 'GetGPIOState-response"
  "ee1aa8c7cecf549eed7751c07a3eb88c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetGPIOState-response>)))
  "Returns full string definition for message of type '<GetGPIOState-response>"
  (cl:format cl:nil "GPIOState output~%~%================================================================================~%MSG: kortex_driver/GPIOState~%~%uint32 identifier~%uint32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetGPIOState-response)))
  "Returns full string definition for message of type 'GetGPIOState-response"
  (cl:format cl:nil "GPIOState output~%~%================================================================================~%MSG: kortex_driver/GPIOState~%~%uint32 identifier~%uint32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetGPIOState-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetGPIOState-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetGPIOState-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetGPIOState)))
  'GetGPIOState-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetGPIOState)))
  'GetGPIOState-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetGPIOState)))
  "Returns string type for a service object of type '<GetGPIOState>"
  "kortex_driver/GetGPIOState")