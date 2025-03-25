; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SendWrenchJoystickCommand-request.msg.html

(cl:defclass <SendWrenchJoystickCommand-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:WrenchCommand
    :initform (cl:make-instance 'kortex_driver-msg:WrenchCommand)))
)

(cl:defclass SendWrenchJoystickCommand-request (<SendWrenchJoystickCommand-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SendWrenchJoystickCommand-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SendWrenchJoystickCommand-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SendWrenchJoystickCommand-request> is deprecated: use kortex_driver-srv:SendWrenchJoystickCommand-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SendWrenchJoystickCommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SendWrenchJoystickCommand-request>) ostream)
  "Serializes a message object of type '<SendWrenchJoystickCommand-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SendWrenchJoystickCommand-request>) istream)
  "Deserializes a message object of type '<SendWrenchJoystickCommand-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SendWrenchJoystickCommand-request>)))
  "Returns string type for a service object of type '<SendWrenchJoystickCommand-request>"
  "kortex_driver/SendWrenchJoystickCommandRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendWrenchJoystickCommand-request)))
  "Returns string type for a service object of type 'SendWrenchJoystickCommand-request"
  "kortex_driver/SendWrenchJoystickCommandRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SendWrenchJoystickCommand-request>)))
  "Returns md5sum for a message object of type '<SendWrenchJoystickCommand-request>"
  "f2e3d158b1d1ef2944d97f7820b907ad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SendWrenchJoystickCommand-request)))
  "Returns md5sum for a message object of type 'SendWrenchJoystickCommand-request"
  "f2e3d158b1d1ef2944d97f7820b907ad")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SendWrenchJoystickCommand-request>)))
  "Returns full string definition for message of type '<SendWrenchJoystickCommand-request>"
  (cl:format cl:nil "WrenchCommand input~%~%================================================================================~%MSG: kortex_driver/WrenchCommand~%~%uint32 reference_frame~%uint32 mode~%Wrench wrench~%uint32 duration~%================================================================================~%MSG: kortex_driver/Wrench~%~%float32 force_x~%float32 force_y~%float32 force_z~%float32 torque_x~%float32 torque_y~%float32 torque_z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SendWrenchJoystickCommand-request)))
  "Returns full string definition for message of type 'SendWrenchJoystickCommand-request"
  (cl:format cl:nil "WrenchCommand input~%~%================================================================================~%MSG: kortex_driver/WrenchCommand~%~%uint32 reference_frame~%uint32 mode~%Wrench wrench~%uint32 duration~%================================================================================~%MSG: kortex_driver/Wrench~%~%float32 force_x~%float32 force_y~%float32 force_z~%float32 torque_x~%float32 torque_y~%float32 torque_z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SendWrenchJoystickCommand-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SendWrenchJoystickCommand-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SendWrenchJoystickCommand-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SendWrenchJoystickCommand-response.msg.html

(cl:defclass <SendWrenchJoystickCommand-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SendWrenchJoystickCommand-response (<SendWrenchJoystickCommand-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SendWrenchJoystickCommand-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SendWrenchJoystickCommand-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SendWrenchJoystickCommand-response> is deprecated: use kortex_driver-srv:SendWrenchJoystickCommand-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SendWrenchJoystickCommand-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SendWrenchJoystickCommand-response>) ostream)
  "Serializes a message object of type '<SendWrenchJoystickCommand-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SendWrenchJoystickCommand-response>) istream)
  "Deserializes a message object of type '<SendWrenchJoystickCommand-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SendWrenchJoystickCommand-response>)))
  "Returns string type for a service object of type '<SendWrenchJoystickCommand-response>"
  "kortex_driver/SendWrenchJoystickCommandResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendWrenchJoystickCommand-response)))
  "Returns string type for a service object of type 'SendWrenchJoystickCommand-response"
  "kortex_driver/SendWrenchJoystickCommandResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SendWrenchJoystickCommand-response>)))
  "Returns md5sum for a message object of type '<SendWrenchJoystickCommand-response>"
  "f2e3d158b1d1ef2944d97f7820b907ad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SendWrenchJoystickCommand-response)))
  "Returns md5sum for a message object of type 'SendWrenchJoystickCommand-response"
  "f2e3d158b1d1ef2944d97f7820b907ad")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SendWrenchJoystickCommand-response>)))
  "Returns full string definition for message of type '<SendWrenchJoystickCommand-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SendWrenchJoystickCommand-response)))
  "Returns full string definition for message of type 'SendWrenchJoystickCommand-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SendWrenchJoystickCommand-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SendWrenchJoystickCommand-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SendWrenchJoystickCommand-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SendWrenchJoystickCommand)))
  'SendWrenchJoystickCommand-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SendWrenchJoystickCommand)))
  'SendWrenchJoystickCommand-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendWrenchJoystickCommand)))
  "Returns string type for a service object of type '<SendWrenchJoystickCommand>"
  "kortex_driver/SendWrenchJoystickCommand")