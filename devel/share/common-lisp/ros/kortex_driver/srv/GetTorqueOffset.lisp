; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetTorqueOffset-request.msg.html

(cl:defclass <GetTorqueOffset-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetTorqueOffset-request (<GetTorqueOffset-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetTorqueOffset-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetTorqueOffset-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetTorqueOffset-request> is deprecated: use kortex_driver-srv:GetTorqueOffset-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetTorqueOffset-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetTorqueOffset-request>) ostream)
  "Serializes a message object of type '<GetTorqueOffset-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetTorqueOffset-request>) istream)
  "Deserializes a message object of type '<GetTorqueOffset-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetTorqueOffset-request>)))
  "Returns string type for a service object of type '<GetTorqueOffset-request>"
  "kortex_driver/GetTorqueOffsetRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTorqueOffset-request)))
  "Returns string type for a service object of type 'GetTorqueOffset-request"
  "kortex_driver/GetTorqueOffsetRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetTorqueOffset-request>)))
  "Returns md5sum for a message object of type '<GetTorqueOffset-request>"
  "6ed1621e934f6480be9371fb25fd7ea1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetTorqueOffset-request)))
  "Returns md5sum for a message object of type 'GetTorqueOffset-request"
  "6ed1621e934f6480be9371fb25fd7ea1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetTorqueOffset-request>)))
  "Returns full string definition for message of type '<GetTorqueOffset-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetTorqueOffset-request)))
  "Returns full string definition for message of type 'GetTorqueOffset-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetTorqueOffset-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetTorqueOffset-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetTorqueOffset-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetTorqueOffset-response.msg.html

(cl:defclass <GetTorqueOffset-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:TorqueOffset
    :initform (cl:make-instance 'kortex_driver-msg:TorqueOffset)))
)

(cl:defclass GetTorqueOffset-response (<GetTorqueOffset-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetTorqueOffset-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetTorqueOffset-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetTorqueOffset-response> is deprecated: use kortex_driver-srv:GetTorqueOffset-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetTorqueOffset-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetTorqueOffset-response>) ostream)
  "Serializes a message object of type '<GetTorqueOffset-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetTorqueOffset-response>) istream)
  "Deserializes a message object of type '<GetTorqueOffset-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetTorqueOffset-response>)))
  "Returns string type for a service object of type '<GetTorqueOffset-response>"
  "kortex_driver/GetTorqueOffsetResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTorqueOffset-response)))
  "Returns string type for a service object of type 'GetTorqueOffset-response"
  "kortex_driver/GetTorqueOffsetResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetTorqueOffset-response>)))
  "Returns md5sum for a message object of type '<GetTorqueOffset-response>"
  "6ed1621e934f6480be9371fb25fd7ea1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetTorqueOffset-response)))
  "Returns md5sum for a message object of type 'GetTorqueOffset-response"
  "6ed1621e934f6480be9371fb25fd7ea1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetTorqueOffset-response>)))
  "Returns full string definition for message of type '<GetTorqueOffset-response>"
  (cl:format cl:nil "TorqueOffset output~%~%================================================================================~%MSG: kortex_driver/TorqueOffset~%~%float32 torque_offset~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetTorqueOffset-response)))
  "Returns full string definition for message of type 'GetTorqueOffset-response"
  (cl:format cl:nil "TorqueOffset output~%~%================================================================================~%MSG: kortex_driver/TorqueOffset~%~%float32 torque_offset~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetTorqueOffset-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetTorqueOffset-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetTorqueOffset-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetTorqueOffset)))
  'GetTorqueOffset-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetTorqueOffset)))
  'GetTorqueOffset-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTorqueOffset)))
  "Returns string type for a service object of type '<GetTorqueOffset>"
  "kortex_driver/GetTorqueOffset")