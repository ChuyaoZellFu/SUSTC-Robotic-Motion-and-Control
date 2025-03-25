; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude StopSequence-request.msg.html

(cl:defclass <StopSequence-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass StopSequence-request (<StopSequence-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StopSequence-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StopSequence-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<StopSequence-request> is deprecated: use kortex_driver-srv:StopSequence-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <StopSequence-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StopSequence-request>) ostream)
  "Serializes a message object of type '<StopSequence-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StopSequence-request>) istream)
  "Deserializes a message object of type '<StopSequence-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StopSequence-request>)))
  "Returns string type for a service object of type '<StopSequence-request>"
  "kortex_driver/StopSequenceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopSequence-request)))
  "Returns string type for a service object of type 'StopSequence-request"
  "kortex_driver/StopSequenceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StopSequence-request>)))
  "Returns md5sum for a message object of type '<StopSequence-request>"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StopSequence-request)))
  "Returns md5sum for a message object of type 'StopSequence-request"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StopSequence-request>)))
  "Returns full string definition for message of type '<StopSequence-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StopSequence-request)))
  "Returns full string definition for message of type 'StopSequence-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StopSequence-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StopSequence-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StopSequence-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude StopSequence-response.msg.html

(cl:defclass <StopSequence-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass StopSequence-response (<StopSequence-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StopSequence-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StopSequence-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<StopSequence-response> is deprecated: use kortex_driver-srv:StopSequence-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <StopSequence-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StopSequence-response>) ostream)
  "Serializes a message object of type '<StopSequence-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StopSequence-response>) istream)
  "Deserializes a message object of type '<StopSequence-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StopSequence-response>)))
  "Returns string type for a service object of type '<StopSequence-response>"
  "kortex_driver/StopSequenceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopSequence-response)))
  "Returns string type for a service object of type 'StopSequence-response"
  "kortex_driver/StopSequenceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StopSequence-response>)))
  "Returns md5sum for a message object of type '<StopSequence-response>"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StopSequence-response)))
  "Returns md5sum for a message object of type 'StopSequence-response"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StopSequence-response>)))
  "Returns full string definition for message of type '<StopSequence-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StopSequence-response)))
  "Returns full string definition for message of type 'StopSequence-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StopSequence-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StopSequence-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StopSequence-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StopSequence)))
  'StopSequence-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StopSequence)))
  'StopSequence-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopSequence)))
  "Returns string type for a service object of type '<StopSequence>"
  "kortex_driver/StopSequence")