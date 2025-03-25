; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SendWrenchCommand-request.msg.html

(cl:defclass <SendWrenchCommand-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:WrenchCommand
    :initform (cl:make-instance 'kortex_driver-msg:WrenchCommand)))
)

(cl:defclass SendWrenchCommand-request (<SendWrenchCommand-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SendWrenchCommand-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SendWrenchCommand-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SendWrenchCommand-request> is deprecated: use kortex_driver-srv:SendWrenchCommand-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SendWrenchCommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SendWrenchCommand-request>) ostream)
  "Serializes a message object of type '<SendWrenchCommand-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SendWrenchCommand-request>) istream)
  "Deserializes a message object of type '<SendWrenchCommand-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SendWrenchCommand-request>)))
  "Returns string type for a service object of type '<SendWrenchCommand-request>"
  "kortex_driver/SendWrenchCommandRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendWrenchCommand-request)))
  "Returns string type for a service object of type 'SendWrenchCommand-request"
  "kortex_driver/SendWrenchCommandRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SendWrenchCommand-request>)))
  "Returns md5sum for a message object of type '<SendWrenchCommand-request>"
  "f2e3d158b1d1ef2944d97f7820b907ad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SendWrenchCommand-request)))
  "Returns md5sum for a message object of type 'SendWrenchCommand-request"
  "f2e3d158b1d1ef2944d97f7820b907ad")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SendWrenchCommand-request>)))
  "Returns full string definition for message of type '<SendWrenchCommand-request>"
  (cl:format cl:nil "WrenchCommand input~%~%================================================================================~%MSG: kortex_driver/WrenchCommand~%~%uint32 reference_frame~%uint32 mode~%Wrench wrench~%uint32 duration~%================================================================================~%MSG: kortex_driver/Wrench~%~%float32 force_x~%float32 force_y~%float32 force_z~%float32 torque_x~%float32 torque_y~%float32 torque_z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SendWrenchCommand-request)))
  "Returns full string definition for message of type 'SendWrenchCommand-request"
  (cl:format cl:nil "WrenchCommand input~%~%================================================================================~%MSG: kortex_driver/WrenchCommand~%~%uint32 reference_frame~%uint32 mode~%Wrench wrench~%uint32 duration~%================================================================================~%MSG: kortex_driver/Wrench~%~%float32 force_x~%float32 force_y~%float32 force_z~%float32 torque_x~%float32 torque_y~%float32 torque_z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SendWrenchCommand-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SendWrenchCommand-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SendWrenchCommand-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SendWrenchCommand-response.msg.html

(cl:defclass <SendWrenchCommand-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SendWrenchCommand-response (<SendWrenchCommand-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SendWrenchCommand-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SendWrenchCommand-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SendWrenchCommand-response> is deprecated: use kortex_driver-srv:SendWrenchCommand-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SendWrenchCommand-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SendWrenchCommand-response>) ostream)
  "Serializes a message object of type '<SendWrenchCommand-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SendWrenchCommand-response>) istream)
  "Deserializes a message object of type '<SendWrenchCommand-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SendWrenchCommand-response>)))
  "Returns string type for a service object of type '<SendWrenchCommand-response>"
  "kortex_driver/SendWrenchCommandResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendWrenchCommand-response)))
  "Returns string type for a service object of type 'SendWrenchCommand-response"
  "kortex_driver/SendWrenchCommandResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SendWrenchCommand-response>)))
  "Returns md5sum for a message object of type '<SendWrenchCommand-response>"
  "f2e3d158b1d1ef2944d97f7820b907ad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SendWrenchCommand-response)))
  "Returns md5sum for a message object of type 'SendWrenchCommand-response"
  "f2e3d158b1d1ef2944d97f7820b907ad")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SendWrenchCommand-response>)))
  "Returns full string definition for message of type '<SendWrenchCommand-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SendWrenchCommand-response)))
  "Returns full string definition for message of type 'SendWrenchCommand-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SendWrenchCommand-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SendWrenchCommand-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SendWrenchCommand-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SendWrenchCommand)))
  'SendWrenchCommand-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SendWrenchCommand)))
  'SendWrenchCommand-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendWrenchCommand)))
  "Returns string type for a service object of type '<SendWrenchCommand>"
  "kortex_driver/SendWrenchCommand")