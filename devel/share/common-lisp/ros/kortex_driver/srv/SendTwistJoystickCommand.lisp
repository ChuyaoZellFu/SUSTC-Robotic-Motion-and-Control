; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SendTwistJoystickCommand-request.msg.html

(cl:defclass <SendTwistJoystickCommand-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:TwistCommand
    :initform (cl:make-instance 'kortex_driver-msg:TwistCommand)))
)

(cl:defclass SendTwistJoystickCommand-request (<SendTwistJoystickCommand-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SendTwistJoystickCommand-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SendTwistJoystickCommand-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SendTwistJoystickCommand-request> is deprecated: use kortex_driver-srv:SendTwistJoystickCommand-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SendTwistJoystickCommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SendTwistJoystickCommand-request>) ostream)
  "Serializes a message object of type '<SendTwistJoystickCommand-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SendTwistJoystickCommand-request>) istream)
  "Deserializes a message object of type '<SendTwistJoystickCommand-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SendTwistJoystickCommand-request>)))
  "Returns string type for a service object of type '<SendTwistJoystickCommand-request>"
  "kortex_driver/SendTwistJoystickCommandRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendTwistJoystickCommand-request)))
  "Returns string type for a service object of type 'SendTwistJoystickCommand-request"
  "kortex_driver/SendTwistJoystickCommandRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SendTwistJoystickCommand-request>)))
  "Returns md5sum for a message object of type '<SendTwistJoystickCommand-request>"
  "1d982d6ab027b7ae362cacd16bfb339f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SendTwistJoystickCommand-request)))
  "Returns md5sum for a message object of type 'SendTwistJoystickCommand-request"
  "1d982d6ab027b7ae362cacd16bfb339f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SendTwistJoystickCommand-request>)))
  "Returns full string definition for message of type '<SendTwistJoystickCommand-request>"
  (cl:format cl:nil "TwistCommand input~%~%================================================================================~%MSG: kortex_driver/TwistCommand~%~%uint32 reference_frame~%Twist twist~%uint32 duration~%================================================================================~%MSG: kortex_driver/Twist~%~%float32 linear_x~%float32 linear_y~%float32 linear_z~%float32 angular_x~%float32 angular_y~%float32 angular_z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SendTwistJoystickCommand-request)))
  "Returns full string definition for message of type 'SendTwistJoystickCommand-request"
  (cl:format cl:nil "TwistCommand input~%~%================================================================================~%MSG: kortex_driver/TwistCommand~%~%uint32 reference_frame~%Twist twist~%uint32 duration~%================================================================================~%MSG: kortex_driver/Twist~%~%float32 linear_x~%float32 linear_y~%float32 linear_z~%float32 angular_x~%float32 angular_y~%float32 angular_z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SendTwistJoystickCommand-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SendTwistJoystickCommand-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SendTwistJoystickCommand-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SendTwistJoystickCommand-response.msg.html

(cl:defclass <SendTwistJoystickCommand-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SendTwistJoystickCommand-response (<SendTwistJoystickCommand-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SendTwistJoystickCommand-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SendTwistJoystickCommand-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SendTwistJoystickCommand-response> is deprecated: use kortex_driver-srv:SendTwistJoystickCommand-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SendTwistJoystickCommand-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SendTwistJoystickCommand-response>) ostream)
  "Serializes a message object of type '<SendTwistJoystickCommand-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SendTwistJoystickCommand-response>) istream)
  "Deserializes a message object of type '<SendTwistJoystickCommand-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SendTwistJoystickCommand-response>)))
  "Returns string type for a service object of type '<SendTwistJoystickCommand-response>"
  "kortex_driver/SendTwistJoystickCommandResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendTwistJoystickCommand-response)))
  "Returns string type for a service object of type 'SendTwistJoystickCommand-response"
  "kortex_driver/SendTwistJoystickCommandResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SendTwistJoystickCommand-response>)))
  "Returns md5sum for a message object of type '<SendTwistJoystickCommand-response>"
  "1d982d6ab027b7ae362cacd16bfb339f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SendTwistJoystickCommand-response)))
  "Returns md5sum for a message object of type 'SendTwistJoystickCommand-response"
  "1d982d6ab027b7ae362cacd16bfb339f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SendTwistJoystickCommand-response>)))
  "Returns full string definition for message of type '<SendTwistJoystickCommand-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SendTwistJoystickCommand-response)))
  "Returns full string definition for message of type 'SendTwistJoystickCommand-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SendTwistJoystickCommand-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SendTwistJoystickCommand-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SendTwistJoystickCommand-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SendTwistJoystickCommand)))
  'SendTwistJoystickCommand-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SendTwistJoystickCommand)))
  'SendTwistJoystickCommand-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendTwistJoystickCommand)))
  "Returns string type for a service object of type '<SendTwistJoystickCommand>"
  "kortex_driver/SendTwistJoystickCommand")