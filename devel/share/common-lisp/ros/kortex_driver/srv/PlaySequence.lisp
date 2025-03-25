; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude PlaySequence-request.msg.html

(cl:defclass <PlaySequence-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:SequenceHandle
    :initform (cl:make-instance 'kortex_driver-msg:SequenceHandle)))
)

(cl:defclass PlaySequence-request (<PlaySequence-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlaySequence-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlaySequence-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<PlaySequence-request> is deprecated: use kortex_driver-srv:PlaySequence-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <PlaySequence-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlaySequence-request>) ostream)
  "Serializes a message object of type '<PlaySequence-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlaySequence-request>) istream)
  "Deserializes a message object of type '<PlaySequence-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlaySequence-request>)))
  "Returns string type for a service object of type '<PlaySequence-request>"
  "kortex_driver/PlaySequenceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlaySequence-request)))
  "Returns string type for a service object of type 'PlaySequence-request"
  "kortex_driver/PlaySequenceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlaySequence-request>)))
  "Returns md5sum for a message object of type '<PlaySequence-request>"
  "04dfaeca45772f660e0913aa84774e13")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlaySequence-request)))
  "Returns md5sum for a message object of type 'PlaySequence-request"
  "04dfaeca45772f660e0913aa84774e13")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlaySequence-request>)))
  "Returns full string definition for message of type '<PlaySequence-request>"
  (cl:format cl:nil "SequenceHandle input~%~%================================================================================~%MSG: kortex_driver/SequenceHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlaySequence-request)))
  "Returns full string definition for message of type 'PlaySequence-request"
  (cl:format cl:nil "SequenceHandle input~%~%================================================================================~%MSG: kortex_driver/SequenceHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlaySequence-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlaySequence-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PlaySequence-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude PlaySequence-response.msg.html

(cl:defclass <PlaySequence-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass PlaySequence-response (<PlaySequence-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlaySequence-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlaySequence-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<PlaySequence-response> is deprecated: use kortex_driver-srv:PlaySequence-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <PlaySequence-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlaySequence-response>) ostream)
  "Serializes a message object of type '<PlaySequence-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlaySequence-response>) istream)
  "Deserializes a message object of type '<PlaySequence-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlaySequence-response>)))
  "Returns string type for a service object of type '<PlaySequence-response>"
  "kortex_driver/PlaySequenceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlaySequence-response)))
  "Returns string type for a service object of type 'PlaySequence-response"
  "kortex_driver/PlaySequenceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlaySequence-response>)))
  "Returns md5sum for a message object of type '<PlaySequence-response>"
  "04dfaeca45772f660e0913aa84774e13")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlaySequence-response)))
  "Returns md5sum for a message object of type 'PlaySequence-response"
  "04dfaeca45772f660e0913aa84774e13")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlaySequence-response>)))
  "Returns full string definition for message of type '<PlaySequence-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlaySequence-response)))
  "Returns full string definition for message of type 'PlaySequence-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlaySequence-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlaySequence-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PlaySequence-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PlaySequence)))
  'PlaySequence-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PlaySequence)))
  'PlaySequence-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlaySequence)))
  "Returns string type for a service object of type '<PlaySequence>"
  "kortex_driver/PlaySequence")