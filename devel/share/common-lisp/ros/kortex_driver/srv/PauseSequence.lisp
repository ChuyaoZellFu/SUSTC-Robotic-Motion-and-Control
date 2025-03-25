; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude PauseSequence-request.msg.html

(cl:defclass <PauseSequence-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass PauseSequence-request (<PauseSequence-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PauseSequence-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PauseSequence-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<PauseSequence-request> is deprecated: use kortex_driver-srv:PauseSequence-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <PauseSequence-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PauseSequence-request>) ostream)
  "Serializes a message object of type '<PauseSequence-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PauseSequence-request>) istream)
  "Deserializes a message object of type '<PauseSequence-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PauseSequence-request>)))
  "Returns string type for a service object of type '<PauseSequence-request>"
  "kortex_driver/PauseSequenceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PauseSequence-request)))
  "Returns string type for a service object of type 'PauseSequence-request"
  "kortex_driver/PauseSequenceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PauseSequence-request>)))
  "Returns md5sum for a message object of type '<PauseSequence-request>"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PauseSequence-request)))
  "Returns md5sum for a message object of type 'PauseSequence-request"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PauseSequence-request>)))
  "Returns full string definition for message of type '<PauseSequence-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PauseSequence-request)))
  "Returns full string definition for message of type 'PauseSequence-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PauseSequence-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PauseSequence-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PauseSequence-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude PauseSequence-response.msg.html

(cl:defclass <PauseSequence-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass PauseSequence-response (<PauseSequence-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PauseSequence-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PauseSequence-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<PauseSequence-response> is deprecated: use kortex_driver-srv:PauseSequence-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <PauseSequence-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PauseSequence-response>) ostream)
  "Serializes a message object of type '<PauseSequence-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PauseSequence-response>) istream)
  "Deserializes a message object of type '<PauseSequence-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PauseSequence-response>)))
  "Returns string type for a service object of type '<PauseSequence-response>"
  "kortex_driver/PauseSequenceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PauseSequence-response)))
  "Returns string type for a service object of type 'PauseSequence-response"
  "kortex_driver/PauseSequenceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PauseSequence-response>)))
  "Returns md5sum for a message object of type '<PauseSequence-response>"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PauseSequence-response)))
  "Returns md5sum for a message object of type 'PauseSequence-response"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PauseSequence-response>)))
  "Returns full string definition for message of type '<PauseSequence-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PauseSequence-response)))
  "Returns full string definition for message of type 'PauseSequence-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PauseSequence-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PauseSequence-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PauseSequence-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PauseSequence)))
  'PauseSequence-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PauseSequence)))
  'PauseSequence-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PauseSequence)))
  "Returns string type for a service object of type '<PauseSequence>"
  "kortex_driver/PauseSequence")