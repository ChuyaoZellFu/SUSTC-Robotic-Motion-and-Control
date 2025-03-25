; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude DeleteAllSequenceTasks-request.msg.html

(cl:defclass <DeleteAllSequenceTasks-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:SequenceHandle
    :initform (cl:make-instance 'kortex_driver-msg:SequenceHandle)))
)

(cl:defclass DeleteAllSequenceTasks-request (<DeleteAllSequenceTasks-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeleteAllSequenceTasks-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeleteAllSequenceTasks-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<DeleteAllSequenceTasks-request> is deprecated: use kortex_driver-srv:DeleteAllSequenceTasks-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <DeleteAllSequenceTasks-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeleteAllSequenceTasks-request>) ostream)
  "Serializes a message object of type '<DeleteAllSequenceTasks-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeleteAllSequenceTasks-request>) istream)
  "Deserializes a message object of type '<DeleteAllSequenceTasks-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeleteAllSequenceTasks-request>)))
  "Returns string type for a service object of type '<DeleteAllSequenceTasks-request>"
  "kortex_driver/DeleteAllSequenceTasksRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeleteAllSequenceTasks-request)))
  "Returns string type for a service object of type 'DeleteAllSequenceTasks-request"
  "kortex_driver/DeleteAllSequenceTasksRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeleteAllSequenceTasks-request>)))
  "Returns md5sum for a message object of type '<DeleteAllSequenceTasks-request>"
  "04dfaeca45772f660e0913aa84774e13")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeleteAllSequenceTasks-request)))
  "Returns md5sum for a message object of type 'DeleteAllSequenceTasks-request"
  "04dfaeca45772f660e0913aa84774e13")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeleteAllSequenceTasks-request>)))
  "Returns full string definition for message of type '<DeleteAllSequenceTasks-request>"
  (cl:format cl:nil "SequenceHandle input~%~%================================================================================~%MSG: kortex_driver/SequenceHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeleteAllSequenceTasks-request)))
  "Returns full string definition for message of type 'DeleteAllSequenceTasks-request"
  (cl:format cl:nil "SequenceHandle input~%~%================================================================================~%MSG: kortex_driver/SequenceHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeleteAllSequenceTasks-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeleteAllSequenceTasks-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DeleteAllSequenceTasks-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude DeleteAllSequenceTasks-response.msg.html

(cl:defclass <DeleteAllSequenceTasks-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass DeleteAllSequenceTasks-response (<DeleteAllSequenceTasks-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeleteAllSequenceTasks-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeleteAllSequenceTasks-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<DeleteAllSequenceTasks-response> is deprecated: use kortex_driver-srv:DeleteAllSequenceTasks-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <DeleteAllSequenceTasks-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeleteAllSequenceTasks-response>) ostream)
  "Serializes a message object of type '<DeleteAllSequenceTasks-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeleteAllSequenceTasks-response>) istream)
  "Deserializes a message object of type '<DeleteAllSequenceTasks-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeleteAllSequenceTasks-response>)))
  "Returns string type for a service object of type '<DeleteAllSequenceTasks-response>"
  "kortex_driver/DeleteAllSequenceTasksResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeleteAllSequenceTasks-response)))
  "Returns string type for a service object of type 'DeleteAllSequenceTasks-response"
  "kortex_driver/DeleteAllSequenceTasksResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeleteAllSequenceTasks-response>)))
  "Returns md5sum for a message object of type '<DeleteAllSequenceTasks-response>"
  "04dfaeca45772f660e0913aa84774e13")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeleteAllSequenceTasks-response)))
  "Returns md5sum for a message object of type 'DeleteAllSequenceTasks-response"
  "04dfaeca45772f660e0913aa84774e13")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeleteAllSequenceTasks-response>)))
  "Returns full string definition for message of type '<DeleteAllSequenceTasks-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeleteAllSequenceTasks-response)))
  "Returns full string definition for message of type 'DeleteAllSequenceTasks-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeleteAllSequenceTasks-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeleteAllSequenceTasks-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DeleteAllSequenceTasks-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DeleteAllSequenceTasks)))
  'DeleteAllSequenceTasks-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DeleteAllSequenceTasks)))
  'DeleteAllSequenceTasks-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeleteAllSequenceTasks)))
  "Returns string type for a service object of type '<DeleteAllSequenceTasks>"
  "kortex_driver/DeleteAllSequenceTasks")