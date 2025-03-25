; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude PauseAction-request.msg.html

(cl:defclass <PauseAction-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass PauseAction-request (<PauseAction-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PauseAction-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PauseAction-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<PauseAction-request> is deprecated: use kortex_driver-srv:PauseAction-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <PauseAction-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PauseAction-request>) ostream)
  "Serializes a message object of type '<PauseAction-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PauseAction-request>) istream)
  "Deserializes a message object of type '<PauseAction-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PauseAction-request>)))
  "Returns string type for a service object of type '<PauseAction-request>"
  "kortex_driver/PauseActionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PauseAction-request)))
  "Returns string type for a service object of type 'PauseAction-request"
  "kortex_driver/PauseActionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PauseAction-request>)))
  "Returns md5sum for a message object of type '<PauseAction-request>"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PauseAction-request)))
  "Returns md5sum for a message object of type 'PauseAction-request"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PauseAction-request>)))
  "Returns full string definition for message of type '<PauseAction-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PauseAction-request)))
  "Returns full string definition for message of type 'PauseAction-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PauseAction-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PauseAction-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PauseAction-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude PauseAction-response.msg.html

(cl:defclass <PauseAction-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass PauseAction-response (<PauseAction-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PauseAction-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PauseAction-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<PauseAction-response> is deprecated: use kortex_driver-srv:PauseAction-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <PauseAction-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PauseAction-response>) ostream)
  "Serializes a message object of type '<PauseAction-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PauseAction-response>) istream)
  "Deserializes a message object of type '<PauseAction-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PauseAction-response>)))
  "Returns string type for a service object of type '<PauseAction-response>"
  "kortex_driver/PauseActionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PauseAction-response)))
  "Returns string type for a service object of type 'PauseAction-response"
  "kortex_driver/PauseActionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PauseAction-response>)))
  "Returns md5sum for a message object of type '<PauseAction-response>"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PauseAction-response)))
  "Returns md5sum for a message object of type 'PauseAction-response"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PauseAction-response>)))
  "Returns full string definition for message of type '<PauseAction-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PauseAction-response)))
  "Returns full string definition for message of type 'PauseAction-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PauseAction-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PauseAction-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PauseAction-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PauseAction)))
  'PauseAction-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PauseAction)))
  'PauseAction-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PauseAction)))
  "Returns string type for a service object of type '<PauseAction>"
  "kortex_driver/PauseAction")