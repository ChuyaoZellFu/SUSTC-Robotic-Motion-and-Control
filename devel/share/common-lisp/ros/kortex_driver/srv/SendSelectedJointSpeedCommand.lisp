; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SendSelectedJointSpeedCommand-request.msg.html

(cl:defclass <SendSelectedJointSpeedCommand-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:JointSpeed
    :initform (cl:make-instance 'kortex_driver-msg:JointSpeed)))
)

(cl:defclass SendSelectedJointSpeedCommand-request (<SendSelectedJointSpeedCommand-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SendSelectedJointSpeedCommand-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SendSelectedJointSpeedCommand-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SendSelectedJointSpeedCommand-request> is deprecated: use kortex_driver-srv:SendSelectedJointSpeedCommand-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SendSelectedJointSpeedCommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SendSelectedJointSpeedCommand-request>) ostream)
  "Serializes a message object of type '<SendSelectedJointSpeedCommand-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SendSelectedJointSpeedCommand-request>) istream)
  "Deserializes a message object of type '<SendSelectedJointSpeedCommand-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SendSelectedJointSpeedCommand-request>)))
  "Returns string type for a service object of type '<SendSelectedJointSpeedCommand-request>"
  "kortex_driver/SendSelectedJointSpeedCommandRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendSelectedJointSpeedCommand-request)))
  "Returns string type for a service object of type 'SendSelectedJointSpeedCommand-request"
  "kortex_driver/SendSelectedJointSpeedCommandRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SendSelectedJointSpeedCommand-request>)))
  "Returns md5sum for a message object of type '<SendSelectedJointSpeedCommand-request>"
  "966eae21a761830257b0f20dfcd24c8e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SendSelectedJointSpeedCommand-request)))
  "Returns md5sum for a message object of type 'SendSelectedJointSpeedCommand-request"
  "966eae21a761830257b0f20dfcd24c8e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SendSelectedJointSpeedCommand-request>)))
  "Returns full string definition for message of type '<SendSelectedJointSpeedCommand-request>"
  (cl:format cl:nil "JointSpeed input~%~%================================================================================~%MSG: kortex_driver/JointSpeed~%~%uint32 joint_identifier~%float32 value~%uint32 duration~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SendSelectedJointSpeedCommand-request)))
  "Returns full string definition for message of type 'SendSelectedJointSpeedCommand-request"
  (cl:format cl:nil "JointSpeed input~%~%================================================================================~%MSG: kortex_driver/JointSpeed~%~%uint32 joint_identifier~%float32 value~%uint32 duration~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SendSelectedJointSpeedCommand-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SendSelectedJointSpeedCommand-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SendSelectedJointSpeedCommand-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SendSelectedJointSpeedCommand-response.msg.html

(cl:defclass <SendSelectedJointSpeedCommand-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SendSelectedJointSpeedCommand-response (<SendSelectedJointSpeedCommand-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SendSelectedJointSpeedCommand-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SendSelectedJointSpeedCommand-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SendSelectedJointSpeedCommand-response> is deprecated: use kortex_driver-srv:SendSelectedJointSpeedCommand-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SendSelectedJointSpeedCommand-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SendSelectedJointSpeedCommand-response>) ostream)
  "Serializes a message object of type '<SendSelectedJointSpeedCommand-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SendSelectedJointSpeedCommand-response>) istream)
  "Deserializes a message object of type '<SendSelectedJointSpeedCommand-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SendSelectedJointSpeedCommand-response>)))
  "Returns string type for a service object of type '<SendSelectedJointSpeedCommand-response>"
  "kortex_driver/SendSelectedJointSpeedCommandResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendSelectedJointSpeedCommand-response)))
  "Returns string type for a service object of type 'SendSelectedJointSpeedCommand-response"
  "kortex_driver/SendSelectedJointSpeedCommandResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SendSelectedJointSpeedCommand-response>)))
  "Returns md5sum for a message object of type '<SendSelectedJointSpeedCommand-response>"
  "966eae21a761830257b0f20dfcd24c8e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SendSelectedJointSpeedCommand-response)))
  "Returns md5sum for a message object of type 'SendSelectedJointSpeedCommand-response"
  "966eae21a761830257b0f20dfcd24c8e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SendSelectedJointSpeedCommand-response>)))
  "Returns full string definition for message of type '<SendSelectedJointSpeedCommand-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SendSelectedJointSpeedCommand-response)))
  "Returns full string definition for message of type 'SendSelectedJointSpeedCommand-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SendSelectedJointSpeedCommand-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SendSelectedJointSpeedCommand-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SendSelectedJointSpeedCommand-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SendSelectedJointSpeedCommand)))
  'SendSelectedJointSpeedCommand-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SendSelectedJointSpeedCommand)))
  'SendSelectedJointSpeedCommand-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendSelectedJointSpeedCommand)))
  "Returns string type for a service object of type '<SendSelectedJointSpeedCommand>"
  "kortex_driver/SendSelectedJointSpeedCommand")