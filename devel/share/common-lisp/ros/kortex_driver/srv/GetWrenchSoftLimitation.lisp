; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetWrenchSoftLimitation-request.msg.html

(cl:defclass <GetWrenchSoftLimitation-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetWrenchSoftLimitation-request (<GetWrenchSoftLimitation-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetWrenchSoftLimitation-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetWrenchSoftLimitation-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetWrenchSoftLimitation-request> is deprecated: use kortex_driver-srv:GetWrenchSoftLimitation-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetWrenchSoftLimitation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetWrenchSoftLimitation-request>) ostream)
  "Serializes a message object of type '<GetWrenchSoftLimitation-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetWrenchSoftLimitation-request>) istream)
  "Deserializes a message object of type '<GetWrenchSoftLimitation-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetWrenchSoftLimitation-request>)))
  "Returns string type for a service object of type '<GetWrenchSoftLimitation-request>"
  "kortex_driver/GetWrenchSoftLimitationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetWrenchSoftLimitation-request)))
  "Returns string type for a service object of type 'GetWrenchSoftLimitation-request"
  "kortex_driver/GetWrenchSoftLimitationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetWrenchSoftLimitation-request>)))
  "Returns md5sum for a message object of type '<GetWrenchSoftLimitation-request>"
  "d1a67d52edbd3c6e07b5922a22648cf3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetWrenchSoftLimitation-request)))
  "Returns md5sum for a message object of type 'GetWrenchSoftLimitation-request"
  "d1a67d52edbd3c6e07b5922a22648cf3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetWrenchSoftLimitation-request>)))
  "Returns full string definition for message of type '<GetWrenchSoftLimitation-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetWrenchSoftLimitation-request)))
  "Returns full string definition for message of type 'GetWrenchSoftLimitation-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetWrenchSoftLimitation-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetWrenchSoftLimitation-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetWrenchSoftLimitation-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetWrenchSoftLimitation-response.msg.html

(cl:defclass <GetWrenchSoftLimitation-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:WrenchLimitation
    :initform (cl:make-instance 'kortex_driver-msg:WrenchLimitation)))
)

(cl:defclass GetWrenchSoftLimitation-response (<GetWrenchSoftLimitation-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetWrenchSoftLimitation-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetWrenchSoftLimitation-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetWrenchSoftLimitation-response> is deprecated: use kortex_driver-srv:GetWrenchSoftLimitation-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetWrenchSoftLimitation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetWrenchSoftLimitation-response>) ostream)
  "Serializes a message object of type '<GetWrenchSoftLimitation-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetWrenchSoftLimitation-response>) istream)
  "Deserializes a message object of type '<GetWrenchSoftLimitation-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetWrenchSoftLimitation-response>)))
  "Returns string type for a service object of type '<GetWrenchSoftLimitation-response>"
  "kortex_driver/GetWrenchSoftLimitationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetWrenchSoftLimitation-response)))
  "Returns string type for a service object of type 'GetWrenchSoftLimitation-response"
  "kortex_driver/GetWrenchSoftLimitationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetWrenchSoftLimitation-response>)))
  "Returns md5sum for a message object of type '<GetWrenchSoftLimitation-response>"
  "d1a67d52edbd3c6e07b5922a22648cf3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetWrenchSoftLimitation-response)))
  "Returns md5sum for a message object of type 'GetWrenchSoftLimitation-response"
  "d1a67d52edbd3c6e07b5922a22648cf3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetWrenchSoftLimitation-response>)))
  "Returns full string definition for message of type '<GetWrenchSoftLimitation-response>"
  (cl:format cl:nil "WrenchLimitation output~%~%================================================================================~%MSG: kortex_driver/WrenchLimitation~%~%float32 force~%float32 torque~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetWrenchSoftLimitation-response)))
  "Returns full string definition for message of type 'GetWrenchSoftLimitation-response"
  (cl:format cl:nil "WrenchLimitation output~%~%================================================================================~%MSG: kortex_driver/WrenchLimitation~%~%float32 force~%float32 torque~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetWrenchSoftLimitation-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetWrenchSoftLimitation-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetWrenchSoftLimitation-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetWrenchSoftLimitation)))
  'GetWrenchSoftLimitation-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetWrenchSoftLimitation)))
  'GetWrenchSoftLimitation-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetWrenchSoftLimitation)))
  "Returns string type for a service object of type '<GetWrenchSoftLimitation>"
  "kortex_driver/GetWrenchSoftLimitation")