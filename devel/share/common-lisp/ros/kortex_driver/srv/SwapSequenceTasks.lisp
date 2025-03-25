; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SwapSequenceTasks-request.msg.html

(cl:defclass <SwapSequenceTasks-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:SequenceTasksPair
    :initform (cl:make-instance 'kortex_driver-msg:SequenceTasksPair)))
)

(cl:defclass SwapSequenceTasks-request (<SwapSequenceTasks-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SwapSequenceTasks-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SwapSequenceTasks-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SwapSequenceTasks-request> is deprecated: use kortex_driver-srv:SwapSequenceTasks-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SwapSequenceTasks-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SwapSequenceTasks-request>) ostream)
  "Serializes a message object of type '<SwapSequenceTasks-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SwapSequenceTasks-request>) istream)
  "Deserializes a message object of type '<SwapSequenceTasks-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SwapSequenceTasks-request>)))
  "Returns string type for a service object of type '<SwapSequenceTasks-request>"
  "kortex_driver/SwapSequenceTasksRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SwapSequenceTasks-request)))
  "Returns string type for a service object of type 'SwapSequenceTasks-request"
  "kortex_driver/SwapSequenceTasksRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SwapSequenceTasks-request>)))
  "Returns md5sum for a message object of type '<SwapSequenceTasks-request>"
  "5e966d67ad7cb2fd416e98745a3e8271")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SwapSequenceTasks-request)))
  "Returns md5sum for a message object of type 'SwapSequenceTasks-request"
  "5e966d67ad7cb2fd416e98745a3e8271")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SwapSequenceTasks-request>)))
  "Returns full string definition for message of type '<SwapSequenceTasks-request>"
  (cl:format cl:nil "SequenceTasksPair input~%~%================================================================================~%MSG: kortex_driver/SequenceTasksPair~%~%SequenceHandle sequence_handle~%uint32 first_task_index~%uint32 second_task_index~%================================================================================~%MSG: kortex_driver/SequenceHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SwapSequenceTasks-request)))
  "Returns full string definition for message of type 'SwapSequenceTasks-request"
  (cl:format cl:nil "SequenceTasksPair input~%~%================================================================================~%MSG: kortex_driver/SequenceTasksPair~%~%SequenceHandle sequence_handle~%uint32 first_task_index~%uint32 second_task_index~%================================================================================~%MSG: kortex_driver/SequenceHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SwapSequenceTasks-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SwapSequenceTasks-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SwapSequenceTasks-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SwapSequenceTasks-response.msg.html

(cl:defclass <SwapSequenceTasks-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SwapSequenceTasks-response (<SwapSequenceTasks-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SwapSequenceTasks-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SwapSequenceTasks-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SwapSequenceTasks-response> is deprecated: use kortex_driver-srv:SwapSequenceTasks-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SwapSequenceTasks-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SwapSequenceTasks-response>) ostream)
  "Serializes a message object of type '<SwapSequenceTasks-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SwapSequenceTasks-response>) istream)
  "Deserializes a message object of type '<SwapSequenceTasks-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SwapSequenceTasks-response>)))
  "Returns string type for a service object of type '<SwapSequenceTasks-response>"
  "kortex_driver/SwapSequenceTasksResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SwapSequenceTasks-response)))
  "Returns string type for a service object of type 'SwapSequenceTasks-response"
  "kortex_driver/SwapSequenceTasksResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SwapSequenceTasks-response>)))
  "Returns md5sum for a message object of type '<SwapSequenceTasks-response>"
  "5e966d67ad7cb2fd416e98745a3e8271")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SwapSequenceTasks-response)))
  "Returns md5sum for a message object of type 'SwapSequenceTasks-response"
  "5e966d67ad7cb2fd416e98745a3e8271")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SwapSequenceTasks-response>)))
  "Returns full string definition for message of type '<SwapSequenceTasks-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SwapSequenceTasks-response)))
  "Returns full string definition for message of type 'SwapSequenceTasks-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SwapSequenceTasks-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SwapSequenceTasks-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SwapSequenceTasks-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SwapSequenceTasks)))
  'SwapSequenceTasks-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SwapSequenceTasks)))
  'SwapSequenceTasks-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SwapSequenceTasks)))
  "Returns string type for a service object of type '<SwapSequenceTasks>"
  "kortex_driver/SwapSequenceTasks")