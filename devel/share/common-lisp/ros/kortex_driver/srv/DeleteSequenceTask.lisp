; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude DeleteSequenceTask-request.msg.html

(cl:defclass <DeleteSequenceTask-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:SequenceTaskHandle
    :initform (cl:make-instance 'kortex_driver-msg:SequenceTaskHandle)))
)

(cl:defclass DeleteSequenceTask-request (<DeleteSequenceTask-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeleteSequenceTask-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeleteSequenceTask-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<DeleteSequenceTask-request> is deprecated: use kortex_driver-srv:DeleteSequenceTask-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <DeleteSequenceTask-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeleteSequenceTask-request>) ostream)
  "Serializes a message object of type '<DeleteSequenceTask-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeleteSequenceTask-request>) istream)
  "Deserializes a message object of type '<DeleteSequenceTask-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeleteSequenceTask-request>)))
  "Returns string type for a service object of type '<DeleteSequenceTask-request>"
  "kortex_driver/DeleteSequenceTaskRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeleteSequenceTask-request)))
  "Returns string type for a service object of type 'DeleteSequenceTask-request"
  "kortex_driver/DeleteSequenceTaskRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeleteSequenceTask-request>)))
  "Returns md5sum for a message object of type '<DeleteSequenceTask-request>"
  "830e67a103da79a18d6fa6ffc8b42b46")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeleteSequenceTask-request)))
  "Returns md5sum for a message object of type 'DeleteSequenceTask-request"
  "830e67a103da79a18d6fa6ffc8b42b46")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeleteSequenceTask-request>)))
  "Returns full string definition for message of type '<DeleteSequenceTask-request>"
  (cl:format cl:nil "SequenceTaskHandle input~%~%================================================================================~%MSG: kortex_driver/SequenceTaskHandle~%~%SequenceHandle sequence_handle~%uint32 task_index~%================================================================================~%MSG: kortex_driver/SequenceHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeleteSequenceTask-request)))
  "Returns full string definition for message of type 'DeleteSequenceTask-request"
  (cl:format cl:nil "SequenceTaskHandle input~%~%================================================================================~%MSG: kortex_driver/SequenceTaskHandle~%~%SequenceHandle sequence_handle~%uint32 task_index~%================================================================================~%MSG: kortex_driver/SequenceHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeleteSequenceTask-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeleteSequenceTask-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DeleteSequenceTask-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude DeleteSequenceTask-response.msg.html

(cl:defclass <DeleteSequenceTask-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass DeleteSequenceTask-response (<DeleteSequenceTask-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeleteSequenceTask-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeleteSequenceTask-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<DeleteSequenceTask-response> is deprecated: use kortex_driver-srv:DeleteSequenceTask-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <DeleteSequenceTask-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeleteSequenceTask-response>) ostream)
  "Serializes a message object of type '<DeleteSequenceTask-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeleteSequenceTask-response>) istream)
  "Deserializes a message object of type '<DeleteSequenceTask-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeleteSequenceTask-response>)))
  "Returns string type for a service object of type '<DeleteSequenceTask-response>"
  "kortex_driver/DeleteSequenceTaskResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeleteSequenceTask-response)))
  "Returns string type for a service object of type 'DeleteSequenceTask-response"
  "kortex_driver/DeleteSequenceTaskResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeleteSequenceTask-response>)))
  "Returns md5sum for a message object of type '<DeleteSequenceTask-response>"
  "830e67a103da79a18d6fa6ffc8b42b46")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeleteSequenceTask-response)))
  "Returns md5sum for a message object of type 'DeleteSequenceTask-response"
  "830e67a103da79a18d6fa6ffc8b42b46")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeleteSequenceTask-response>)))
  "Returns full string definition for message of type '<DeleteSequenceTask-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeleteSequenceTask-response)))
  "Returns full string definition for message of type 'DeleteSequenceTask-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeleteSequenceTask-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeleteSequenceTask-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DeleteSequenceTask-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DeleteSequenceTask)))
  'DeleteSequenceTask-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DeleteSequenceTask)))
  'DeleteSequenceTask-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeleteSequenceTask)))
  "Returns string type for a service object of type '<DeleteSequenceTask>"
  "kortex_driver/DeleteSequenceTask")