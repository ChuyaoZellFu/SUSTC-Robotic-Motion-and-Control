; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetDesiredSpeeds-request.msg.html

(cl:defclass <GetDesiredSpeeds-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetDesiredSpeeds-request (<GetDesiredSpeeds-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetDesiredSpeeds-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetDesiredSpeeds-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetDesiredSpeeds-request> is deprecated: use kortex_driver-srv:GetDesiredSpeeds-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetDesiredSpeeds-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetDesiredSpeeds-request>) ostream)
  "Serializes a message object of type '<GetDesiredSpeeds-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetDesiredSpeeds-request>) istream)
  "Deserializes a message object of type '<GetDesiredSpeeds-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetDesiredSpeeds-request>)))
  "Returns string type for a service object of type '<GetDesiredSpeeds-request>"
  "kortex_driver/GetDesiredSpeedsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDesiredSpeeds-request)))
  "Returns string type for a service object of type 'GetDesiredSpeeds-request"
  "kortex_driver/GetDesiredSpeedsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetDesiredSpeeds-request>)))
  "Returns md5sum for a message object of type '<GetDesiredSpeeds-request>"
  "1dd665a1d839a65c717e02f2317ca84c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetDesiredSpeeds-request)))
  "Returns md5sum for a message object of type 'GetDesiredSpeeds-request"
  "1dd665a1d839a65c717e02f2317ca84c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetDesiredSpeeds-request>)))
  "Returns full string definition for message of type '<GetDesiredSpeeds-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetDesiredSpeeds-request)))
  "Returns full string definition for message of type 'GetDesiredSpeeds-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetDesiredSpeeds-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetDesiredSpeeds-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetDesiredSpeeds-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetDesiredSpeeds-response.msg.html

(cl:defclass <GetDesiredSpeeds-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:DesiredSpeeds
    :initform (cl:make-instance 'kortex_driver-msg:DesiredSpeeds)))
)

(cl:defclass GetDesiredSpeeds-response (<GetDesiredSpeeds-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetDesiredSpeeds-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetDesiredSpeeds-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetDesiredSpeeds-response> is deprecated: use kortex_driver-srv:GetDesiredSpeeds-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetDesiredSpeeds-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetDesiredSpeeds-response>) ostream)
  "Serializes a message object of type '<GetDesiredSpeeds-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetDesiredSpeeds-response>) istream)
  "Deserializes a message object of type '<GetDesiredSpeeds-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetDesiredSpeeds-response>)))
  "Returns string type for a service object of type '<GetDesiredSpeeds-response>"
  "kortex_driver/GetDesiredSpeedsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDesiredSpeeds-response)))
  "Returns string type for a service object of type 'GetDesiredSpeeds-response"
  "kortex_driver/GetDesiredSpeedsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetDesiredSpeeds-response>)))
  "Returns md5sum for a message object of type '<GetDesiredSpeeds-response>"
  "1dd665a1d839a65c717e02f2317ca84c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetDesiredSpeeds-response)))
  "Returns md5sum for a message object of type 'GetDesiredSpeeds-response"
  "1dd665a1d839a65c717e02f2317ca84c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetDesiredSpeeds-response>)))
  "Returns full string definition for message of type '<GetDesiredSpeeds-response>"
  (cl:format cl:nil "DesiredSpeeds output~%~%================================================================================~%MSG: kortex_driver/DesiredSpeeds~%~%float32 linear~%float32 angular~%float32[] joint_speed~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetDesiredSpeeds-response)))
  "Returns full string definition for message of type 'GetDesiredSpeeds-response"
  (cl:format cl:nil "DesiredSpeeds output~%~%================================================================================~%MSG: kortex_driver/DesiredSpeeds~%~%float32 linear~%float32 angular~%float32[] joint_speed~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetDesiredSpeeds-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetDesiredSpeeds-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetDesiredSpeeds-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetDesiredSpeeds)))
  'GetDesiredSpeeds-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetDesiredSpeeds)))
  'GetDesiredSpeeds-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDesiredSpeeds)))
  "Returns string type for a service object of type '<GetDesiredSpeeds>"
  "kortex_driver/GetDesiredSpeeds")