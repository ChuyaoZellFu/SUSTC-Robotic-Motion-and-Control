; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetTwistSoftLimitation-request.msg.html

(cl:defclass <GetTwistSoftLimitation-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetTwistSoftLimitation-request (<GetTwistSoftLimitation-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetTwistSoftLimitation-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetTwistSoftLimitation-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetTwistSoftLimitation-request> is deprecated: use kortex_driver-srv:GetTwistSoftLimitation-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetTwistSoftLimitation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetTwistSoftLimitation-request>) ostream)
  "Serializes a message object of type '<GetTwistSoftLimitation-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetTwistSoftLimitation-request>) istream)
  "Deserializes a message object of type '<GetTwistSoftLimitation-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetTwistSoftLimitation-request>)))
  "Returns string type for a service object of type '<GetTwistSoftLimitation-request>"
  "kortex_driver/GetTwistSoftLimitationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTwistSoftLimitation-request)))
  "Returns string type for a service object of type 'GetTwistSoftLimitation-request"
  "kortex_driver/GetTwistSoftLimitationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetTwistSoftLimitation-request>)))
  "Returns md5sum for a message object of type '<GetTwistSoftLimitation-request>"
  "69b1f43d97a46e21f8b8586022fd23ec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetTwistSoftLimitation-request)))
  "Returns md5sum for a message object of type 'GetTwistSoftLimitation-request"
  "69b1f43d97a46e21f8b8586022fd23ec")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetTwistSoftLimitation-request>)))
  "Returns full string definition for message of type '<GetTwistSoftLimitation-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetTwistSoftLimitation-request)))
  "Returns full string definition for message of type 'GetTwistSoftLimitation-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetTwistSoftLimitation-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetTwistSoftLimitation-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetTwistSoftLimitation-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetTwistSoftLimitation-response.msg.html

(cl:defclass <GetTwistSoftLimitation-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:TwistLimitation
    :initform (cl:make-instance 'kortex_driver-msg:TwistLimitation)))
)

(cl:defclass GetTwistSoftLimitation-response (<GetTwistSoftLimitation-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetTwistSoftLimitation-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetTwistSoftLimitation-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetTwistSoftLimitation-response> is deprecated: use kortex_driver-srv:GetTwistSoftLimitation-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetTwistSoftLimitation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetTwistSoftLimitation-response>) ostream)
  "Serializes a message object of type '<GetTwistSoftLimitation-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetTwistSoftLimitation-response>) istream)
  "Deserializes a message object of type '<GetTwistSoftLimitation-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetTwistSoftLimitation-response>)))
  "Returns string type for a service object of type '<GetTwistSoftLimitation-response>"
  "kortex_driver/GetTwistSoftLimitationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTwistSoftLimitation-response)))
  "Returns string type for a service object of type 'GetTwistSoftLimitation-response"
  "kortex_driver/GetTwistSoftLimitationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetTwistSoftLimitation-response>)))
  "Returns md5sum for a message object of type '<GetTwistSoftLimitation-response>"
  "69b1f43d97a46e21f8b8586022fd23ec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetTwistSoftLimitation-response)))
  "Returns md5sum for a message object of type 'GetTwistSoftLimitation-response"
  "69b1f43d97a46e21f8b8586022fd23ec")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetTwistSoftLimitation-response>)))
  "Returns full string definition for message of type '<GetTwistSoftLimitation-response>"
  (cl:format cl:nil "TwistLimitation output~%~%================================================================================~%MSG: kortex_driver/TwistLimitation~%~%float32 linear~%float32 angular~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetTwistSoftLimitation-response)))
  "Returns full string definition for message of type 'GetTwistSoftLimitation-response"
  (cl:format cl:nil "TwistLimitation output~%~%================================================================================~%MSG: kortex_driver/TwistLimitation~%~%float32 linear~%float32 angular~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetTwistSoftLimitation-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetTwistSoftLimitation-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetTwistSoftLimitation-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetTwistSoftLimitation)))
  'GetTwistSoftLimitation-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetTwistSoftLimitation)))
  'GetTwistSoftLimitation-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTwistSoftLimitation)))
  "Returns string type for a service object of type '<GetTwistSoftLimitation>"
  "kortex_driver/GetTwistSoftLimitation")