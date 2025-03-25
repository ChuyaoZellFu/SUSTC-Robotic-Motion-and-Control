; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude MoveToPosition-request.msg.html

(cl:defclass <MoveToPosition-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:PositionCommand
    :initform (cl:make-instance 'kortex_driver-msg:PositionCommand)))
)

(cl:defclass MoveToPosition-request (<MoveToPosition-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveToPosition-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveToPosition-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<MoveToPosition-request> is deprecated: use kortex_driver-srv:MoveToPosition-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <MoveToPosition-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveToPosition-request>) ostream)
  "Serializes a message object of type '<MoveToPosition-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveToPosition-request>) istream)
  "Deserializes a message object of type '<MoveToPosition-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveToPosition-request>)))
  "Returns string type for a service object of type '<MoveToPosition-request>"
  "kortex_driver/MoveToPositionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveToPosition-request)))
  "Returns string type for a service object of type 'MoveToPosition-request"
  "kortex_driver/MoveToPositionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveToPosition-request>)))
  "Returns md5sum for a message object of type '<MoveToPosition-request>"
  "e232d9e676cf9963716162da0597fb96")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveToPosition-request)))
  "Returns md5sum for a message object of type 'MoveToPosition-request"
  "e232d9e676cf9963716162da0597fb96")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveToPosition-request>)))
  "Returns full string definition for message of type '<MoveToPosition-request>"
  (cl:format cl:nil "PositionCommand input~%~%================================================================================~%MSG: kortex_driver/PositionCommand~%~%float32 position~%float32 velocity~%float32 acceleration~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveToPosition-request)))
  "Returns full string definition for message of type 'MoveToPosition-request"
  (cl:format cl:nil "PositionCommand input~%~%================================================================================~%MSG: kortex_driver/PositionCommand~%~%float32 position~%float32 velocity~%float32 acceleration~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveToPosition-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveToPosition-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveToPosition-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude MoveToPosition-response.msg.html

(cl:defclass <MoveToPosition-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass MoveToPosition-response (<MoveToPosition-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveToPosition-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveToPosition-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<MoveToPosition-response> is deprecated: use kortex_driver-srv:MoveToPosition-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <MoveToPosition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveToPosition-response>) ostream)
  "Serializes a message object of type '<MoveToPosition-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveToPosition-response>) istream)
  "Deserializes a message object of type '<MoveToPosition-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveToPosition-response>)))
  "Returns string type for a service object of type '<MoveToPosition-response>"
  "kortex_driver/MoveToPositionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveToPosition-response)))
  "Returns string type for a service object of type 'MoveToPosition-response"
  "kortex_driver/MoveToPositionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveToPosition-response>)))
  "Returns md5sum for a message object of type '<MoveToPosition-response>"
  "e232d9e676cf9963716162da0597fb96")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveToPosition-response)))
  "Returns md5sum for a message object of type 'MoveToPosition-response"
  "e232d9e676cf9963716162da0597fb96")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveToPosition-response>)))
  "Returns full string definition for message of type '<MoveToPosition-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveToPosition-response)))
  "Returns full string definition for message of type 'MoveToPosition-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveToPosition-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveToPosition-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveToPosition-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MoveToPosition)))
  'MoveToPosition-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MoveToPosition)))
  'MoveToPosition-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveToPosition)))
  "Returns string type for a service object of type '<MoveToPosition>"
  "kortex_driver/MoveToPosition")