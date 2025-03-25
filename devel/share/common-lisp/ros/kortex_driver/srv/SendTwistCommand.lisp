; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SendTwistCommand-request.msg.html

(cl:defclass <SendTwistCommand-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:TwistCommand
    :initform (cl:make-instance 'kortex_driver-msg:TwistCommand)))
)

(cl:defclass SendTwistCommand-request (<SendTwistCommand-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SendTwistCommand-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SendTwistCommand-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SendTwistCommand-request> is deprecated: use kortex_driver-srv:SendTwistCommand-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SendTwistCommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SendTwistCommand-request>) ostream)
  "Serializes a message object of type '<SendTwistCommand-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SendTwistCommand-request>) istream)
  "Deserializes a message object of type '<SendTwistCommand-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SendTwistCommand-request>)))
  "Returns string type for a service object of type '<SendTwistCommand-request>"
  "kortex_driver/SendTwistCommandRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendTwistCommand-request)))
  "Returns string type for a service object of type 'SendTwistCommand-request"
  "kortex_driver/SendTwistCommandRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SendTwistCommand-request>)))
  "Returns md5sum for a message object of type '<SendTwistCommand-request>"
  "1d982d6ab027b7ae362cacd16bfb339f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SendTwistCommand-request)))
  "Returns md5sum for a message object of type 'SendTwistCommand-request"
  "1d982d6ab027b7ae362cacd16bfb339f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SendTwistCommand-request>)))
  "Returns full string definition for message of type '<SendTwistCommand-request>"
  (cl:format cl:nil "TwistCommand input~%~%================================================================================~%MSG: kortex_driver/TwistCommand~%~%uint32 reference_frame~%Twist twist~%uint32 duration~%================================================================================~%MSG: kortex_driver/Twist~%~%float32 linear_x~%float32 linear_y~%float32 linear_z~%float32 angular_x~%float32 angular_y~%float32 angular_z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SendTwistCommand-request)))
  "Returns full string definition for message of type 'SendTwistCommand-request"
  (cl:format cl:nil "TwistCommand input~%~%================================================================================~%MSG: kortex_driver/TwistCommand~%~%uint32 reference_frame~%Twist twist~%uint32 duration~%================================================================================~%MSG: kortex_driver/Twist~%~%float32 linear_x~%float32 linear_y~%float32 linear_z~%float32 angular_x~%float32 angular_y~%float32 angular_z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SendTwistCommand-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SendTwistCommand-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SendTwistCommand-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SendTwistCommand-response.msg.html

(cl:defclass <SendTwistCommand-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SendTwistCommand-response (<SendTwistCommand-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SendTwistCommand-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SendTwistCommand-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SendTwistCommand-response> is deprecated: use kortex_driver-srv:SendTwistCommand-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SendTwistCommand-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SendTwistCommand-response>) ostream)
  "Serializes a message object of type '<SendTwistCommand-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SendTwistCommand-response>) istream)
  "Deserializes a message object of type '<SendTwistCommand-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SendTwistCommand-response>)))
  "Returns string type for a service object of type '<SendTwistCommand-response>"
  "kortex_driver/SendTwistCommandResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendTwistCommand-response)))
  "Returns string type for a service object of type 'SendTwistCommand-response"
  "kortex_driver/SendTwistCommandResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SendTwistCommand-response>)))
  "Returns md5sum for a message object of type '<SendTwistCommand-response>"
  "1d982d6ab027b7ae362cacd16bfb339f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SendTwistCommand-response)))
  "Returns md5sum for a message object of type 'SendTwistCommand-response"
  "1d982d6ab027b7ae362cacd16bfb339f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SendTwistCommand-response>)))
  "Returns full string definition for message of type '<SendTwistCommand-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SendTwistCommand-response)))
  "Returns full string definition for message of type 'SendTwistCommand-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SendTwistCommand-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SendTwistCommand-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SendTwistCommand-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SendTwistCommand)))
  'SendTwistCommand-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SendTwistCommand)))
  'SendTwistCommand-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendTwistCommand)))
  "Returns string type for a service object of type '<SendTwistCommand>"
  "kortex_driver/SendTwistCommand")