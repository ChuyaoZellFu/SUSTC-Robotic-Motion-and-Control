; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude ResumeSequence-request.msg.html

(cl:defclass <ResumeSequence-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass ResumeSequence-request (<ResumeSequence-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResumeSequence-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResumeSequence-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ResumeSequence-request> is deprecated: use kortex_driver-srv:ResumeSequence-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <ResumeSequence-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResumeSequence-request>) ostream)
  "Serializes a message object of type '<ResumeSequence-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResumeSequence-request>) istream)
  "Deserializes a message object of type '<ResumeSequence-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResumeSequence-request>)))
  "Returns string type for a service object of type '<ResumeSequence-request>"
  "kortex_driver/ResumeSequenceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResumeSequence-request)))
  "Returns string type for a service object of type 'ResumeSequence-request"
  "kortex_driver/ResumeSequenceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResumeSequence-request>)))
  "Returns md5sum for a message object of type '<ResumeSequence-request>"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResumeSequence-request)))
  "Returns md5sum for a message object of type 'ResumeSequence-request"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResumeSequence-request>)))
  "Returns full string definition for message of type '<ResumeSequence-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResumeSequence-request)))
  "Returns full string definition for message of type 'ResumeSequence-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResumeSequence-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResumeSequence-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ResumeSequence-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude ResumeSequence-response.msg.html

(cl:defclass <ResumeSequence-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass ResumeSequence-response (<ResumeSequence-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResumeSequence-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResumeSequence-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ResumeSequence-response> is deprecated: use kortex_driver-srv:ResumeSequence-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <ResumeSequence-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResumeSequence-response>) ostream)
  "Serializes a message object of type '<ResumeSequence-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResumeSequence-response>) istream)
  "Deserializes a message object of type '<ResumeSequence-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResumeSequence-response>)))
  "Returns string type for a service object of type '<ResumeSequence-response>"
  "kortex_driver/ResumeSequenceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResumeSequence-response)))
  "Returns string type for a service object of type 'ResumeSequence-response"
  "kortex_driver/ResumeSequenceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResumeSequence-response>)))
  "Returns md5sum for a message object of type '<ResumeSequence-response>"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResumeSequence-response)))
  "Returns md5sum for a message object of type 'ResumeSequence-response"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResumeSequence-response>)))
  "Returns full string definition for message of type '<ResumeSequence-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResumeSequence-response)))
  "Returns full string definition for message of type 'ResumeSequence-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResumeSequence-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResumeSequence-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ResumeSequence-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ResumeSequence)))
  'ResumeSequence-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ResumeSequence)))
  'ResumeSequence-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResumeSequence)))
  "Returns string type for a service object of type '<ResumeSequence>"
  "kortex_driver/ResumeSequence")