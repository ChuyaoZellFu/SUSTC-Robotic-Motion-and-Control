; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude MoveSequenceTask-request.msg.html

(cl:defclass <MoveSequenceTask-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:SequenceTasksPair
    :initform (cl:make-instance 'kortex_driver-msg:SequenceTasksPair)))
)

(cl:defclass MoveSequenceTask-request (<MoveSequenceTask-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveSequenceTask-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveSequenceTask-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<MoveSequenceTask-request> is deprecated: use kortex_driver-srv:MoveSequenceTask-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <MoveSequenceTask-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveSequenceTask-request>) ostream)
  "Serializes a message object of type '<MoveSequenceTask-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveSequenceTask-request>) istream)
  "Deserializes a message object of type '<MoveSequenceTask-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveSequenceTask-request>)))
  "Returns string type for a service object of type '<MoveSequenceTask-request>"
  "kortex_driver/MoveSequenceTaskRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveSequenceTask-request)))
  "Returns string type for a service object of type 'MoveSequenceTask-request"
  "kortex_driver/MoveSequenceTaskRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveSequenceTask-request>)))
  "Returns md5sum for a message object of type '<MoveSequenceTask-request>"
  "5e966d67ad7cb2fd416e98745a3e8271")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveSequenceTask-request)))
  "Returns md5sum for a message object of type 'MoveSequenceTask-request"
  "5e966d67ad7cb2fd416e98745a3e8271")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveSequenceTask-request>)))
  "Returns full string definition for message of type '<MoveSequenceTask-request>"
  (cl:format cl:nil "SequenceTasksPair input~%~%================================================================================~%MSG: kortex_driver/SequenceTasksPair~%~%SequenceHandle sequence_handle~%uint32 first_task_index~%uint32 second_task_index~%================================================================================~%MSG: kortex_driver/SequenceHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveSequenceTask-request)))
  "Returns full string definition for message of type 'MoveSequenceTask-request"
  (cl:format cl:nil "SequenceTasksPair input~%~%================================================================================~%MSG: kortex_driver/SequenceTasksPair~%~%SequenceHandle sequence_handle~%uint32 first_task_index~%uint32 second_task_index~%================================================================================~%MSG: kortex_driver/SequenceHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveSequenceTask-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveSequenceTask-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveSequenceTask-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude MoveSequenceTask-response.msg.html

(cl:defclass <MoveSequenceTask-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass MoveSequenceTask-response (<MoveSequenceTask-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveSequenceTask-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveSequenceTask-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<MoveSequenceTask-response> is deprecated: use kortex_driver-srv:MoveSequenceTask-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <MoveSequenceTask-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveSequenceTask-response>) ostream)
  "Serializes a message object of type '<MoveSequenceTask-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveSequenceTask-response>) istream)
  "Deserializes a message object of type '<MoveSequenceTask-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveSequenceTask-response>)))
  "Returns string type for a service object of type '<MoveSequenceTask-response>"
  "kortex_driver/MoveSequenceTaskResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveSequenceTask-response)))
  "Returns string type for a service object of type 'MoveSequenceTask-response"
  "kortex_driver/MoveSequenceTaskResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveSequenceTask-response>)))
  "Returns md5sum for a message object of type '<MoveSequenceTask-response>"
  "5e966d67ad7cb2fd416e98745a3e8271")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveSequenceTask-response)))
  "Returns md5sum for a message object of type 'MoveSequenceTask-response"
  "5e966d67ad7cb2fd416e98745a3e8271")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveSequenceTask-response>)))
  "Returns full string definition for message of type '<MoveSequenceTask-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveSequenceTask-response)))
  "Returns full string definition for message of type 'MoveSequenceTask-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveSequenceTask-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveSequenceTask-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveSequenceTask-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MoveSequenceTask)))
  'MoveSequenceTask-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MoveSequenceTask)))
  'MoveSequenceTask-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveSequenceTask)))
  "Returns string type for a service object of type '<MoveSequenceTask>"
  "kortex_driver/MoveSequenceTask")