; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetWrenchHardLimitation-request.msg.html

(cl:defclass <GetWrenchHardLimitation-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetWrenchHardLimitation-request (<GetWrenchHardLimitation-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetWrenchHardLimitation-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetWrenchHardLimitation-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetWrenchHardLimitation-request> is deprecated: use kortex_driver-srv:GetWrenchHardLimitation-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetWrenchHardLimitation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetWrenchHardLimitation-request>) ostream)
  "Serializes a message object of type '<GetWrenchHardLimitation-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetWrenchHardLimitation-request>) istream)
  "Deserializes a message object of type '<GetWrenchHardLimitation-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetWrenchHardLimitation-request>)))
  "Returns string type for a service object of type '<GetWrenchHardLimitation-request>"
  "kortex_driver/GetWrenchHardLimitationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetWrenchHardLimitation-request)))
  "Returns string type for a service object of type 'GetWrenchHardLimitation-request"
  "kortex_driver/GetWrenchHardLimitationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetWrenchHardLimitation-request>)))
  "Returns md5sum for a message object of type '<GetWrenchHardLimitation-request>"
  "d1a67d52edbd3c6e07b5922a22648cf3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetWrenchHardLimitation-request)))
  "Returns md5sum for a message object of type 'GetWrenchHardLimitation-request"
  "d1a67d52edbd3c6e07b5922a22648cf3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetWrenchHardLimitation-request>)))
  "Returns full string definition for message of type '<GetWrenchHardLimitation-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetWrenchHardLimitation-request)))
  "Returns full string definition for message of type 'GetWrenchHardLimitation-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetWrenchHardLimitation-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetWrenchHardLimitation-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetWrenchHardLimitation-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetWrenchHardLimitation-response.msg.html

(cl:defclass <GetWrenchHardLimitation-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:WrenchLimitation
    :initform (cl:make-instance 'kortex_driver-msg:WrenchLimitation)))
)

(cl:defclass GetWrenchHardLimitation-response (<GetWrenchHardLimitation-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetWrenchHardLimitation-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetWrenchHardLimitation-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetWrenchHardLimitation-response> is deprecated: use kortex_driver-srv:GetWrenchHardLimitation-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetWrenchHardLimitation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetWrenchHardLimitation-response>) ostream)
  "Serializes a message object of type '<GetWrenchHardLimitation-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetWrenchHardLimitation-response>) istream)
  "Deserializes a message object of type '<GetWrenchHardLimitation-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetWrenchHardLimitation-response>)))
  "Returns string type for a service object of type '<GetWrenchHardLimitation-response>"
  "kortex_driver/GetWrenchHardLimitationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetWrenchHardLimitation-response)))
  "Returns string type for a service object of type 'GetWrenchHardLimitation-response"
  "kortex_driver/GetWrenchHardLimitationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetWrenchHardLimitation-response>)))
  "Returns md5sum for a message object of type '<GetWrenchHardLimitation-response>"
  "d1a67d52edbd3c6e07b5922a22648cf3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetWrenchHardLimitation-response)))
  "Returns md5sum for a message object of type 'GetWrenchHardLimitation-response"
  "d1a67d52edbd3c6e07b5922a22648cf3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetWrenchHardLimitation-response>)))
  "Returns full string definition for message of type '<GetWrenchHardLimitation-response>"
  (cl:format cl:nil "WrenchLimitation output~%~%================================================================================~%MSG: kortex_driver/WrenchLimitation~%~%float32 force~%float32 torque~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetWrenchHardLimitation-response)))
  "Returns full string definition for message of type 'GetWrenchHardLimitation-response"
  (cl:format cl:nil "WrenchLimitation output~%~%================================================================================~%MSG: kortex_driver/WrenchLimitation~%~%float32 force~%float32 torque~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetWrenchHardLimitation-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetWrenchHardLimitation-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetWrenchHardLimitation-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetWrenchHardLimitation)))
  'GetWrenchHardLimitation-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetWrenchHardLimitation)))
  'GetWrenchHardLimitation-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetWrenchHardLimitation)))
  "Returns string type for a service object of type '<GetWrenchHardLimitation>"
  "kortex_driver/GetWrenchHardLimitation")