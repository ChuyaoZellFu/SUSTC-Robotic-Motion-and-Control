; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SendGripperCommand-request.msg.html

(cl:defclass <SendGripperCommand-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:GripperCommand
    :initform (cl:make-instance 'kortex_driver-msg:GripperCommand)))
)

(cl:defclass SendGripperCommand-request (<SendGripperCommand-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SendGripperCommand-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SendGripperCommand-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SendGripperCommand-request> is deprecated: use kortex_driver-srv:SendGripperCommand-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SendGripperCommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SendGripperCommand-request>) ostream)
  "Serializes a message object of type '<SendGripperCommand-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SendGripperCommand-request>) istream)
  "Deserializes a message object of type '<SendGripperCommand-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SendGripperCommand-request>)))
  "Returns string type for a service object of type '<SendGripperCommand-request>"
  "kortex_driver/SendGripperCommandRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendGripperCommand-request)))
  "Returns string type for a service object of type 'SendGripperCommand-request"
  "kortex_driver/SendGripperCommandRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SendGripperCommand-request>)))
  "Returns md5sum for a message object of type '<SendGripperCommand-request>"
  "d750c71a9686ff834d5687f20aaad330")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SendGripperCommand-request)))
  "Returns md5sum for a message object of type 'SendGripperCommand-request"
  "d750c71a9686ff834d5687f20aaad330")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SendGripperCommand-request>)))
  "Returns full string definition for message of type '<SendGripperCommand-request>"
  (cl:format cl:nil "GripperCommand input~%~%================================================================================~%MSG: kortex_driver/GripperCommand~%~%uint32 mode~%Gripper gripper~%uint32 duration~%================================================================================~%MSG: kortex_driver/Gripper~%~%Finger[] finger~%================================================================================~%MSG: kortex_driver/Finger~%~%uint32 finger_identifier~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SendGripperCommand-request)))
  "Returns full string definition for message of type 'SendGripperCommand-request"
  (cl:format cl:nil "GripperCommand input~%~%================================================================================~%MSG: kortex_driver/GripperCommand~%~%uint32 mode~%Gripper gripper~%uint32 duration~%================================================================================~%MSG: kortex_driver/Gripper~%~%Finger[] finger~%================================================================================~%MSG: kortex_driver/Finger~%~%uint32 finger_identifier~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SendGripperCommand-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SendGripperCommand-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SendGripperCommand-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SendGripperCommand-response.msg.html

(cl:defclass <SendGripperCommand-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SendGripperCommand-response (<SendGripperCommand-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SendGripperCommand-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SendGripperCommand-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SendGripperCommand-response> is deprecated: use kortex_driver-srv:SendGripperCommand-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SendGripperCommand-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SendGripperCommand-response>) ostream)
  "Serializes a message object of type '<SendGripperCommand-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SendGripperCommand-response>) istream)
  "Deserializes a message object of type '<SendGripperCommand-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SendGripperCommand-response>)))
  "Returns string type for a service object of type '<SendGripperCommand-response>"
  "kortex_driver/SendGripperCommandResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendGripperCommand-response)))
  "Returns string type for a service object of type 'SendGripperCommand-response"
  "kortex_driver/SendGripperCommandResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SendGripperCommand-response>)))
  "Returns md5sum for a message object of type '<SendGripperCommand-response>"
  "d750c71a9686ff834d5687f20aaad330")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SendGripperCommand-response)))
  "Returns md5sum for a message object of type 'SendGripperCommand-response"
  "d750c71a9686ff834d5687f20aaad330")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SendGripperCommand-response>)))
  "Returns full string definition for message of type '<SendGripperCommand-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SendGripperCommand-response)))
  "Returns full string definition for message of type 'SendGripperCommand-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SendGripperCommand-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SendGripperCommand-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SendGripperCommand-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SendGripperCommand)))
  'SendGripperCommand-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SendGripperCommand)))
  'SendGripperCommand-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendGripperCommand)))
  "Returns string type for a service object of type '<SendGripperCommand>"
  "kortex_driver/SendGripperCommand")