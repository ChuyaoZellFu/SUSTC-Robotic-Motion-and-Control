; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude StopAction-request.msg.html

(cl:defclass <StopAction-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass StopAction-request (<StopAction-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StopAction-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StopAction-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<StopAction-request> is deprecated: use kortex_driver-srv:StopAction-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <StopAction-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StopAction-request>) ostream)
  "Serializes a message object of type '<StopAction-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StopAction-request>) istream)
  "Deserializes a message object of type '<StopAction-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StopAction-request>)))
  "Returns string type for a service object of type '<StopAction-request>"
  "kortex_driver/StopActionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopAction-request)))
  "Returns string type for a service object of type 'StopAction-request"
  "kortex_driver/StopActionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StopAction-request>)))
  "Returns md5sum for a message object of type '<StopAction-request>"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StopAction-request)))
  "Returns md5sum for a message object of type 'StopAction-request"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StopAction-request>)))
  "Returns full string definition for message of type '<StopAction-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StopAction-request)))
  "Returns full string definition for message of type 'StopAction-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StopAction-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StopAction-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StopAction-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude StopAction-response.msg.html

(cl:defclass <StopAction-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass StopAction-response (<StopAction-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StopAction-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StopAction-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<StopAction-response> is deprecated: use kortex_driver-srv:StopAction-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <StopAction-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StopAction-response>) ostream)
  "Serializes a message object of type '<StopAction-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StopAction-response>) istream)
  "Deserializes a message object of type '<StopAction-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StopAction-response>)))
  "Returns string type for a service object of type '<StopAction-response>"
  "kortex_driver/StopActionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopAction-response)))
  "Returns string type for a service object of type 'StopAction-response"
  "kortex_driver/StopActionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StopAction-response>)))
  "Returns md5sum for a message object of type '<StopAction-response>"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StopAction-response)))
  "Returns md5sum for a message object of type 'StopAction-response"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StopAction-response>)))
  "Returns full string definition for message of type '<StopAction-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StopAction-response)))
  "Returns full string definition for message of type 'StopAction-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StopAction-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StopAction-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StopAction-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StopAction)))
  'StopAction-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StopAction)))
  'StopAction-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopAction)))
  "Returns string type for a service object of type '<StopAction>"
  "kortex_driver/StopAction")