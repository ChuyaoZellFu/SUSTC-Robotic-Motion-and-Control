; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetActuatorCount-request.msg.html

(cl:defclass <GetActuatorCount-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetActuatorCount-request (<GetActuatorCount-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetActuatorCount-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetActuatorCount-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetActuatorCount-request> is deprecated: use kortex_driver-srv:GetActuatorCount-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetActuatorCount-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetActuatorCount-request>) ostream)
  "Serializes a message object of type '<GetActuatorCount-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetActuatorCount-request>) istream)
  "Deserializes a message object of type '<GetActuatorCount-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetActuatorCount-request>)))
  "Returns string type for a service object of type '<GetActuatorCount-request>"
  "kortex_driver/GetActuatorCountRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetActuatorCount-request)))
  "Returns string type for a service object of type 'GetActuatorCount-request"
  "kortex_driver/GetActuatorCountRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetActuatorCount-request>)))
  "Returns md5sum for a message object of type '<GetActuatorCount-request>"
  "e8b096678f65c6ea2fbcbfdbbf77d42f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetActuatorCount-request)))
  "Returns md5sum for a message object of type 'GetActuatorCount-request"
  "e8b096678f65c6ea2fbcbfdbbf77d42f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetActuatorCount-request>)))
  "Returns full string definition for message of type '<GetActuatorCount-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetActuatorCount-request)))
  "Returns full string definition for message of type 'GetActuatorCount-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetActuatorCount-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetActuatorCount-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetActuatorCount-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetActuatorCount-response.msg.html

(cl:defclass <GetActuatorCount-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:ActuatorInformation
    :initform (cl:make-instance 'kortex_driver-msg:ActuatorInformation)))
)

(cl:defclass GetActuatorCount-response (<GetActuatorCount-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetActuatorCount-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetActuatorCount-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetActuatorCount-response> is deprecated: use kortex_driver-srv:GetActuatorCount-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetActuatorCount-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetActuatorCount-response>) ostream)
  "Serializes a message object of type '<GetActuatorCount-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetActuatorCount-response>) istream)
  "Deserializes a message object of type '<GetActuatorCount-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetActuatorCount-response>)))
  "Returns string type for a service object of type '<GetActuatorCount-response>"
  "kortex_driver/GetActuatorCountResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetActuatorCount-response)))
  "Returns string type for a service object of type 'GetActuatorCount-response"
  "kortex_driver/GetActuatorCountResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetActuatorCount-response>)))
  "Returns md5sum for a message object of type '<GetActuatorCount-response>"
  "e8b096678f65c6ea2fbcbfdbbf77d42f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetActuatorCount-response)))
  "Returns md5sum for a message object of type 'GetActuatorCount-response"
  "e8b096678f65c6ea2fbcbfdbbf77d42f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetActuatorCount-response>)))
  "Returns full string definition for message of type '<GetActuatorCount-response>"
  (cl:format cl:nil "ActuatorInformation output~%~%================================================================================~%MSG: kortex_driver/ActuatorInformation~%~%uint32 count~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetActuatorCount-response)))
  "Returns full string definition for message of type 'GetActuatorCount-response"
  (cl:format cl:nil "ActuatorInformation output~%~%================================================================================~%MSG: kortex_driver/ActuatorInformation~%~%uint32 count~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetActuatorCount-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetActuatorCount-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetActuatorCount-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetActuatorCount)))
  'GetActuatorCount-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetActuatorCount)))
  'GetActuatorCount-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetActuatorCount)))
  "Returns string type for a service object of type '<GetActuatorCount>"
  "kortex_driver/GetActuatorCount")