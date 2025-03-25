; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetTwistHardLimitation-request.msg.html

(cl:defclass <GetTwistHardLimitation-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetTwistHardLimitation-request (<GetTwistHardLimitation-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetTwistHardLimitation-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetTwistHardLimitation-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetTwistHardLimitation-request> is deprecated: use kortex_driver-srv:GetTwistHardLimitation-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetTwistHardLimitation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetTwistHardLimitation-request>) ostream)
  "Serializes a message object of type '<GetTwistHardLimitation-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetTwistHardLimitation-request>) istream)
  "Deserializes a message object of type '<GetTwistHardLimitation-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetTwistHardLimitation-request>)))
  "Returns string type for a service object of type '<GetTwistHardLimitation-request>"
  "kortex_driver/GetTwistHardLimitationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTwistHardLimitation-request)))
  "Returns string type for a service object of type 'GetTwistHardLimitation-request"
  "kortex_driver/GetTwistHardLimitationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetTwistHardLimitation-request>)))
  "Returns md5sum for a message object of type '<GetTwistHardLimitation-request>"
  "69b1f43d97a46e21f8b8586022fd23ec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetTwistHardLimitation-request)))
  "Returns md5sum for a message object of type 'GetTwistHardLimitation-request"
  "69b1f43d97a46e21f8b8586022fd23ec")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetTwistHardLimitation-request>)))
  "Returns full string definition for message of type '<GetTwistHardLimitation-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetTwistHardLimitation-request)))
  "Returns full string definition for message of type 'GetTwistHardLimitation-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetTwistHardLimitation-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetTwistHardLimitation-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetTwistHardLimitation-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetTwistHardLimitation-response.msg.html

(cl:defclass <GetTwistHardLimitation-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:TwistLimitation
    :initform (cl:make-instance 'kortex_driver-msg:TwistLimitation)))
)

(cl:defclass GetTwistHardLimitation-response (<GetTwistHardLimitation-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetTwistHardLimitation-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetTwistHardLimitation-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetTwistHardLimitation-response> is deprecated: use kortex_driver-srv:GetTwistHardLimitation-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetTwistHardLimitation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetTwistHardLimitation-response>) ostream)
  "Serializes a message object of type '<GetTwistHardLimitation-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetTwistHardLimitation-response>) istream)
  "Deserializes a message object of type '<GetTwistHardLimitation-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetTwistHardLimitation-response>)))
  "Returns string type for a service object of type '<GetTwistHardLimitation-response>"
  "kortex_driver/GetTwistHardLimitationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTwistHardLimitation-response)))
  "Returns string type for a service object of type 'GetTwistHardLimitation-response"
  "kortex_driver/GetTwistHardLimitationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetTwistHardLimitation-response>)))
  "Returns md5sum for a message object of type '<GetTwistHardLimitation-response>"
  "69b1f43d97a46e21f8b8586022fd23ec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetTwistHardLimitation-response)))
  "Returns md5sum for a message object of type 'GetTwistHardLimitation-response"
  "69b1f43d97a46e21f8b8586022fd23ec")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetTwistHardLimitation-response>)))
  "Returns full string definition for message of type '<GetTwistHardLimitation-response>"
  (cl:format cl:nil "TwistLimitation output~%~%================================================================================~%MSG: kortex_driver/TwistLimitation~%~%float32 linear~%float32 angular~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetTwistHardLimitation-response)))
  "Returns full string definition for message of type 'GetTwistHardLimitation-response"
  (cl:format cl:nil "TwistLimitation output~%~%================================================================================~%MSG: kortex_driver/TwistLimitation~%~%float32 linear~%float32 angular~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetTwistHardLimitation-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetTwistHardLimitation-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetTwistHardLimitation-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetTwistHardLimitation)))
  'GetTwistHardLimitation-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetTwistHardLimitation)))
  'GetTwistHardLimitation-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTwistHardLimitation)))
  "Returns string type for a service object of type '<GetTwistHardLimitation>"
  "kortex_driver/GetTwistHardLimitation")