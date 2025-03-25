; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude ResetGravityVector-request.msg.html

(cl:defclass <ResetGravityVector-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass ResetGravityVector-request (<ResetGravityVector-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetGravityVector-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetGravityVector-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ResetGravityVector-request> is deprecated: use kortex_driver-srv:ResetGravityVector-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <ResetGravityVector-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetGravityVector-request>) ostream)
  "Serializes a message object of type '<ResetGravityVector-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetGravityVector-request>) istream)
  "Deserializes a message object of type '<ResetGravityVector-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetGravityVector-request>)))
  "Returns string type for a service object of type '<ResetGravityVector-request>"
  "kortex_driver/ResetGravityVectorRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetGravityVector-request)))
  "Returns string type for a service object of type 'ResetGravityVector-request"
  "kortex_driver/ResetGravityVectorRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetGravityVector-request>)))
  "Returns md5sum for a message object of type '<ResetGravityVector-request>"
  "8956f0eb2b4baf668929f3e8781781fa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetGravityVector-request)))
  "Returns md5sum for a message object of type 'ResetGravityVector-request"
  "8956f0eb2b4baf668929f3e8781781fa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetGravityVector-request>)))
  "Returns full string definition for message of type '<ResetGravityVector-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetGravityVector-request)))
  "Returns full string definition for message of type 'ResetGravityVector-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetGravityVector-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetGravityVector-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetGravityVector-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude ResetGravityVector-response.msg.html

(cl:defclass <ResetGravityVector-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:GravityVector
    :initform (cl:make-instance 'kortex_driver-msg:GravityVector)))
)

(cl:defclass ResetGravityVector-response (<ResetGravityVector-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetGravityVector-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetGravityVector-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ResetGravityVector-response> is deprecated: use kortex_driver-srv:ResetGravityVector-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <ResetGravityVector-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetGravityVector-response>) ostream)
  "Serializes a message object of type '<ResetGravityVector-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetGravityVector-response>) istream)
  "Deserializes a message object of type '<ResetGravityVector-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetGravityVector-response>)))
  "Returns string type for a service object of type '<ResetGravityVector-response>"
  "kortex_driver/ResetGravityVectorResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetGravityVector-response)))
  "Returns string type for a service object of type 'ResetGravityVector-response"
  "kortex_driver/ResetGravityVectorResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetGravityVector-response>)))
  "Returns md5sum for a message object of type '<ResetGravityVector-response>"
  "8956f0eb2b4baf668929f3e8781781fa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetGravityVector-response)))
  "Returns md5sum for a message object of type 'ResetGravityVector-response"
  "8956f0eb2b4baf668929f3e8781781fa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetGravityVector-response>)))
  "Returns full string definition for message of type '<ResetGravityVector-response>"
  (cl:format cl:nil "GravityVector output~%~%================================================================================~%MSG: kortex_driver/GravityVector~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetGravityVector-response)))
  "Returns full string definition for message of type 'ResetGravityVector-response"
  (cl:format cl:nil "GravityVector output~%~%================================================================================~%MSG: kortex_driver/GravityVector~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetGravityVector-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetGravityVector-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetGravityVector-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ResetGravityVector)))
  'ResetGravityVector-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ResetGravityVector)))
  'ResetGravityVector-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetGravityVector)))
  "Returns string type for a service object of type '<ResetGravityVector>"
  "kortex_driver/ResetGravityVector")