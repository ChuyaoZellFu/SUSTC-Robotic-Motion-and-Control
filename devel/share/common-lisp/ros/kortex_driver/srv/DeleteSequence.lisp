; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude DeleteSequence-request.msg.html

(cl:defclass <DeleteSequence-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:SequenceHandle
    :initform (cl:make-instance 'kortex_driver-msg:SequenceHandle)))
)

(cl:defclass DeleteSequence-request (<DeleteSequence-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeleteSequence-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeleteSequence-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<DeleteSequence-request> is deprecated: use kortex_driver-srv:DeleteSequence-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <DeleteSequence-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeleteSequence-request>) ostream)
  "Serializes a message object of type '<DeleteSequence-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeleteSequence-request>) istream)
  "Deserializes a message object of type '<DeleteSequence-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeleteSequence-request>)))
  "Returns string type for a service object of type '<DeleteSequence-request>"
  "kortex_driver/DeleteSequenceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeleteSequence-request)))
  "Returns string type for a service object of type 'DeleteSequence-request"
  "kortex_driver/DeleteSequenceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeleteSequence-request>)))
  "Returns md5sum for a message object of type '<DeleteSequence-request>"
  "04dfaeca45772f660e0913aa84774e13")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeleteSequence-request)))
  "Returns md5sum for a message object of type 'DeleteSequence-request"
  "04dfaeca45772f660e0913aa84774e13")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeleteSequence-request>)))
  "Returns full string definition for message of type '<DeleteSequence-request>"
  (cl:format cl:nil "SequenceHandle input~%~%================================================================================~%MSG: kortex_driver/SequenceHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeleteSequence-request)))
  "Returns full string definition for message of type 'DeleteSequence-request"
  (cl:format cl:nil "SequenceHandle input~%~%================================================================================~%MSG: kortex_driver/SequenceHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeleteSequence-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeleteSequence-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DeleteSequence-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude DeleteSequence-response.msg.html

(cl:defclass <DeleteSequence-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass DeleteSequence-response (<DeleteSequence-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeleteSequence-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeleteSequence-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<DeleteSequence-response> is deprecated: use kortex_driver-srv:DeleteSequence-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <DeleteSequence-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeleteSequence-response>) ostream)
  "Serializes a message object of type '<DeleteSequence-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeleteSequence-response>) istream)
  "Deserializes a message object of type '<DeleteSequence-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeleteSequence-response>)))
  "Returns string type for a service object of type '<DeleteSequence-response>"
  "kortex_driver/DeleteSequenceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeleteSequence-response)))
  "Returns string type for a service object of type 'DeleteSequence-response"
  "kortex_driver/DeleteSequenceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeleteSequence-response>)))
  "Returns md5sum for a message object of type '<DeleteSequence-response>"
  "04dfaeca45772f660e0913aa84774e13")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeleteSequence-response)))
  "Returns md5sum for a message object of type 'DeleteSequence-response"
  "04dfaeca45772f660e0913aa84774e13")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeleteSequence-response>)))
  "Returns full string definition for message of type '<DeleteSequence-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeleteSequence-response)))
  "Returns full string definition for message of type 'DeleteSequence-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeleteSequence-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeleteSequence-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DeleteSequence-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DeleteSequence)))
  'DeleteSequence-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DeleteSequence)))
  'DeleteSequence-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeleteSequence)))
  "Returns string type for a service object of type '<DeleteSequence>"
  "kortex_driver/DeleteSequence")