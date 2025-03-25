; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetGravityVector-request.msg.html

(cl:defclass <GetGravityVector-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetGravityVector-request (<GetGravityVector-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetGravityVector-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetGravityVector-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetGravityVector-request> is deprecated: use kortex_driver-srv:GetGravityVector-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetGravityVector-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetGravityVector-request>) ostream)
  "Serializes a message object of type '<GetGravityVector-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetGravityVector-request>) istream)
  "Deserializes a message object of type '<GetGravityVector-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetGravityVector-request>)))
  "Returns string type for a service object of type '<GetGravityVector-request>"
  "kortex_driver/GetGravityVectorRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetGravityVector-request)))
  "Returns string type for a service object of type 'GetGravityVector-request"
  "kortex_driver/GetGravityVectorRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetGravityVector-request>)))
  "Returns md5sum for a message object of type '<GetGravityVector-request>"
  "8956f0eb2b4baf668929f3e8781781fa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetGravityVector-request)))
  "Returns md5sum for a message object of type 'GetGravityVector-request"
  "8956f0eb2b4baf668929f3e8781781fa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetGravityVector-request>)))
  "Returns full string definition for message of type '<GetGravityVector-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetGravityVector-request)))
  "Returns full string definition for message of type 'GetGravityVector-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetGravityVector-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetGravityVector-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetGravityVector-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetGravityVector-response.msg.html

(cl:defclass <GetGravityVector-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:GravityVector
    :initform (cl:make-instance 'kortex_driver-msg:GravityVector)))
)

(cl:defclass GetGravityVector-response (<GetGravityVector-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetGravityVector-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetGravityVector-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetGravityVector-response> is deprecated: use kortex_driver-srv:GetGravityVector-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetGravityVector-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetGravityVector-response>) ostream)
  "Serializes a message object of type '<GetGravityVector-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetGravityVector-response>) istream)
  "Deserializes a message object of type '<GetGravityVector-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetGravityVector-response>)))
  "Returns string type for a service object of type '<GetGravityVector-response>"
  "kortex_driver/GetGravityVectorResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetGravityVector-response)))
  "Returns string type for a service object of type 'GetGravityVector-response"
  "kortex_driver/GetGravityVectorResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetGravityVector-response>)))
  "Returns md5sum for a message object of type '<GetGravityVector-response>"
  "8956f0eb2b4baf668929f3e8781781fa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetGravityVector-response)))
  "Returns md5sum for a message object of type 'GetGravityVector-response"
  "8956f0eb2b4baf668929f3e8781781fa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetGravityVector-response>)))
  "Returns full string definition for message of type '<GetGravityVector-response>"
  (cl:format cl:nil "GravityVector output~%~%================================================================================~%MSG: kortex_driver/GravityVector~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetGravityVector-response)))
  "Returns full string definition for message of type 'GetGravityVector-response"
  (cl:format cl:nil "GravityVector output~%~%================================================================================~%MSG: kortex_driver/GravityVector~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetGravityVector-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetGravityVector-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetGravityVector-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetGravityVector)))
  'GetGravityVector-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetGravityVector)))
  'GetGravityVector-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetGravityVector)))
  "Returns string type for a service object of type '<GetGravityVector>"
  "kortex_driver/GetGravityVector")