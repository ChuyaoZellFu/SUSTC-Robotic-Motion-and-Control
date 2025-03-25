; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SendJointSpeedsCommand-request.msg.html

(cl:defclass <SendJointSpeedsCommand-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Base_JointSpeeds
    :initform (cl:make-instance 'kortex_driver-msg:Base_JointSpeeds)))
)

(cl:defclass SendJointSpeedsCommand-request (<SendJointSpeedsCommand-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SendJointSpeedsCommand-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SendJointSpeedsCommand-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SendJointSpeedsCommand-request> is deprecated: use kortex_driver-srv:SendJointSpeedsCommand-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SendJointSpeedsCommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SendJointSpeedsCommand-request>) ostream)
  "Serializes a message object of type '<SendJointSpeedsCommand-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SendJointSpeedsCommand-request>) istream)
  "Deserializes a message object of type '<SendJointSpeedsCommand-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SendJointSpeedsCommand-request>)))
  "Returns string type for a service object of type '<SendJointSpeedsCommand-request>"
  "kortex_driver/SendJointSpeedsCommandRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendJointSpeedsCommand-request)))
  "Returns string type for a service object of type 'SendJointSpeedsCommand-request"
  "kortex_driver/SendJointSpeedsCommandRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SendJointSpeedsCommand-request>)))
  "Returns md5sum for a message object of type '<SendJointSpeedsCommand-request>"
  "35bff15135e19b4099e6a92d5e7d08d5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SendJointSpeedsCommand-request)))
  "Returns md5sum for a message object of type 'SendJointSpeedsCommand-request"
  "35bff15135e19b4099e6a92d5e7d08d5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SendJointSpeedsCommand-request>)))
  "Returns full string definition for message of type '<SendJointSpeedsCommand-request>"
  (cl:format cl:nil "Base_JointSpeeds input~%~%================================================================================~%MSG: kortex_driver/Base_JointSpeeds~%~%JointSpeed[] joint_speeds~%uint32 duration~%================================================================================~%MSG: kortex_driver/JointSpeed~%~%uint32 joint_identifier~%float32 value~%uint32 duration~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SendJointSpeedsCommand-request)))
  "Returns full string definition for message of type 'SendJointSpeedsCommand-request"
  (cl:format cl:nil "Base_JointSpeeds input~%~%================================================================================~%MSG: kortex_driver/Base_JointSpeeds~%~%JointSpeed[] joint_speeds~%uint32 duration~%================================================================================~%MSG: kortex_driver/JointSpeed~%~%uint32 joint_identifier~%float32 value~%uint32 duration~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SendJointSpeedsCommand-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SendJointSpeedsCommand-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SendJointSpeedsCommand-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SendJointSpeedsCommand-response.msg.html

(cl:defclass <SendJointSpeedsCommand-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SendJointSpeedsCommand-response (<SendJointSpeedsCommand-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SendJointSpeedsCommand-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SendJointSpeedsCommand-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SendJointSpeedsCommand-response> is deprecated: use kortex_driver-srv:SendJointSpeedsCommand-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SendJointSpeedsCommand-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SendJointSpeedsCommand-response>) ostream)
  "Serializes a message object of type '<SendJointSpeedsCommand-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SendJointSpeedsCommand-response>) istream)
  "Deserializes a message object of type '<SendJointSpeedsCommand-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SendJointSpeedsCommand-response>)))
  "Returns string type for a service object of type '<SendJointSpeedsCommand-response>"
  "kortex_driver/SendJointSpeedsCommandResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendJointSpeedsCommand-response)))
  "Returns string type for a service object of type 'SendJointSpeedsCommand-response"
  "kortex_driver/SendJointSpeedsCommandResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SendJointSpeedsCommand-response>)))
  "Returns md5sum for a message object of type '<SendJointSpeedsCommand-response>"
  "35bff15135e19b4099e6a92d5e7d08d5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SendJointSpeedsCommand-response)))
  "Returns md5sum for a message object of type 'SendJointSpeedsCommand-response"
  "35bff15135e19b4099e6a92d5e7d08d5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SendJointSpeedsCommand-response>)))
  "Returns full string definition for message of type '<SendJointSpeedsCommand-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SendJointSpeedsCommand-response)))
  "Returns full string definition for message of type 'SendJointSpeedsCommand-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SendJointSpeedsCommand-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SendJointSpeedsCommand-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SendJointSpeedsCommand-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SendJointSpeedsCommand)))
  'SendJointSpeedsCommand-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SendJointSpeedsCommand)))
  'SendJointSpeedsCommand-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendJointSpeedsCommand)))
  "Returns string type for a service object of type '<SendJointSpeedsCommand>"
  "kortex_driver/SendJointSpeedsCommand")