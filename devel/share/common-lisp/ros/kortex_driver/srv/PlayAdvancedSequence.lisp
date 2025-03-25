; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude PlayAdvancedSequence-request.msg.html

(cl:defclass <PlayAdvancedSequence-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:AdvancedSequenceHandle
    :initform (cl:make-instance 'kortex_driver-msg:AdvancedSequenceHandle)))
)

(cl:defclass PlayAdvancedSequence-request (<PlayAdvancedSequence-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlayAdvancedSequence-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlayAdvancedSequence-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<PlayAdvancedSequence-request> is deprecated: use kortex_driver-srv:PlayAdvancedSequence-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <PlayAdvancedSequence-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlayAdvancedSequence-request>) ostream)
  "Serializes a message object of type '<PlayAdvancedSequence-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlayAdvancedSequence-request>) istream)
  "Deserializes a message object of type '<PlayAdvancedSequence-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlayAdvancedSequence-request>)))
  "Returns string type for a service object of type '<PlayAdvancedSequence-request>"
  "kortex_driver/PlayAdvancedSequenceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlayAdvancedSequence-request)))
  "Returns string type for a service object of type 'PlayAdvancedSequence-request"
  "kortex_driver/PlayAdvancedSequenceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlayAdvancedSequence-request>)))
  "Returns md5sum for a message object of type '<PlayAdvancedSequence-request>"
  "ff3d79c2f78fa637d54a7284e0052ea0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlayAdvancedSequence-request)))
  "Returns md5sum for a message object of type 'PlayAdvancedSequence-request"
  "ff3d79c2f78fa637d54a7284e0052ea0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlayAdvancedSequence-request>)))
  "Returns full string definition for message of type '<PlayAdvancedSequence-request>"
  (cl:format cl:nil "AdvancedSequenceHandle input~%~%================================================================================~%MSG: kortex_driver/AdvancedSequenceHandle~%~%SequenceHandle handle~%bool in_loop~%================================================================================~%MSG: kortex_driver/SequenceHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlayAdvancedSequence-request)))
  "Returns full string definition for message of type 'PlayAdvancedSequence-request"
  (cl:format cl:nil "AdvancedSequenceHandle input~%~%================================================================================~%MSG: kortex_driver/AdvancedSequenceHandle~%~%SequenceHandle handle~%bool in_loop~%================================================================================~%MSG: kortex_driver/SequenceHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlayAdvancedSequence-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlayAdvancedSequence-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PlayAdvancedSequence-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude PlayAdvancedSequence-response.msg.html

(cl:defclass <PlayAdvancedSequence-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass PlayAdvancedSequence-response (<PlayAdvancedSequence-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlayAdvancedSequence-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlayAdvancedSequence-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<PlayAdvancedSequence-response> is deprecated: use kortex_driver-srv:PlayAdvancedSequence-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <PlayAdvancedSequence-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlayAdvancedSequence-response>) ostream)
  "Serializes a message object of type '<PlayAdvancedSequence-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlayAdvancedSequence-response>) istream)
  "Deserializes a message object of type '<PlayAdvancedSequence-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlayAdvancedSequence-response>)))
  "Returns string type for a service object of type '<PlayAdvancedSequence-response>"
  "kortex_driver/PlayAdvancedSequenceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlayAdvancedSequence-response)))
  "Returns string type for a service object of type 'PlayAdvancedSequence-response"
  "kortex_driver/PlayAdvancedSequenceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlayAdvancedSequence-response>)))
  "Returns md5sum for a message object of type '<PlayAdvancedSequence-response>"
  "ff3d79c2f78fa637d54a7284e0052ea0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlayAdvancedSequence-response)))
  "Returns md5sum for a message object of type 'PlayAdvancedSequence-response"
  "ff3d79c2f78fa637d54a7284e0052ea0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlayAdvancedSequence-response>)))
  "Returns full string definition for message of type '<PlayAdvancedSequence-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlayAdvancedSequence-response)))
  "Returns full string definition for message of type 'PlayAdvancedSequence-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlayAdvancedSequence-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlayAdvancedSequence-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PlayAdvancedSequence-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PlayAdvancedSequence)))
  'PlayAdvancedSequence-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PlayAdvancedSequence)))
  'PlayAdvancedSequence-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlayAdvancedSequence)))
  "Returns string type for a service object of type '<PlayAdvancedSequence>"
  "kortex_driver/PlayAdvancedSequence")