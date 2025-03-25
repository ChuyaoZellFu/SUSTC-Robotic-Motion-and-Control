; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude StartTeaching-request.msg.html

(cl:defclass <StartTeaching-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:SequenceTaskHandle
    :initform (cl:make-instance 'kortex_driver-msg:SequenceTaskHandle)))
)

(cl:defclass StartTeaching-request (<StartTeaching-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartTeaching-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartTeaching-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<StartTeaching-request> is deprecated: use kortex_driver-srv:StartTeaching-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <StartTeaching-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartTeaching-request>) ostream)
  "Serializes a message object of type '<StartTeaching-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartTeaching-request>) istream)
  "Deserializes a message object of type '<StartTeaching-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartTeaching-request>)))
  "Returns string type for a service object of type '<StartTeaching-request>"
  "kortex_driver/StartTeachingRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartTeaching-request)))
  "Returns string type for a service object of type 'StartTeaching-request"
  "kortex_driver/StartTeachingRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartTeaching-request>)))
  "Returns md5sum for a message object of type '<StartTeaching-request>"
  "830e67a103da79a18d6fa6ffc8b42b46")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartTeaching-request)))
  "Returns md5sum for a message object of type 'StartTeaching-request"
  "830e67a103da79a18d6fa6ffc8b42b46")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartTeaching-request>)))
  "Returns full string definition for message of type '<StartTeaching-request>"
  (cl:format cl:nil "SequenceTaskHandle input~%~%================================================================================~%MSG: kortex_driver/SequenceTaskHandle~%~%SequenceHandle sequence_handle~%uint32 task_index~%================================================================================~%MSG: kortex_driver/SequenceHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartTeaching-request)))
  "Returns full string definition for message of type 'StartTeaching-request"
  (cl:format cl:nil "SequenceTaskHandle input~%~%================================================================================~%MSG: kortex_driver/SequenceTaskHandle~%~%SequenceHandle sequence_handle~%uint32 task_index~%================================================================================~%MSG: kortex_driver/SequenceHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartTeaching-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartTeaching-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StartTeaching-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude StartTeaching-response.msg.html

(cl:defclass <StartTeaching-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass StartTeaching-response (<StartTeaching-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartTeaching-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartTeaching-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<StartTeaching-response> is deprecated: use kortex_driver-srv:StartTeaching-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <StartTeaching-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartTeaching-response>) ostream)
  "Serializes a message object of type '<StartTeaching-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartTeaching-response>) istream)
  "Deserializes a message object of type '<StartTeaching-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartTeaching-response>)))
  "Returns string type for a service object of type '<StartTeaching-response>"
  "kortex_driver/StartTeachingResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartTeaching-response)))
  "Returns string type for a service object of type 'StartTeaching-response"
  "kortex_driver/StartTeachingResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartTeaching-response>)))
  "Returns md5sum for a message object of type '<StartTeaching-response>"
  "830e67a103da79a18d6fa6ffc8b42b46")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartTeaching-response)))
  "Returns md5sum for a message object of type 'StartTeaching-response"
  "830e67a103da79a18d6fa6ffc8b42b46")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartTeaching-response>)))
  "Returns full string definition for message of type '<StartTeaching-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartTeaching-response)))
  "Returns full string definition for message of type 'StartTeaching-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartTeaching-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartTeaching-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StartTeaching-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StartTeaching)))
  'StartTeaching-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StartTeaching)))
  'StartTeaching-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartTeaching)))
  "Returns string type for a service object of type '<StartTeaching>"
  "kortex_driver/StartTeaching")