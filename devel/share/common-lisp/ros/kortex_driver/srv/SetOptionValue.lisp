; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetOptionValue-request.msg.html

(cl:defclass <SetOptionValue-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:OptionValue
    :initform (cl:make-instance 'kortex_driver-msg:OptionValue)))
)

(cl:defclass SetOptionValue-request (<SetOptionValue-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetOptionValue-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetOptionValue-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetOptionValue-request> is deprecated: use kortex_driver-srv:SetOptionValue-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetOptionValue-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetOptionValue-request>) ostream)
  "Serializes a message object of type '<SetOptionValue-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetOptionValue-request>) istream)
  "Deserializes a message object of type '<SetOptionValue-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetOptionValue-request>)))
  "Returns string type for a service object of type '<SetOptionValue-request>"
  "kortex_driver/SetOptionValueRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetOptionValue-request)))
  "Returns string type for a service object of type 'SetOptionValue-request"
  "kortex_driver/SetOptionValueRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetOptionValue-request>)))
  "Returns md5sum for a message object of type '<SetOptionValue-request>"
  "c37d6886e6fd73ac7a80b3ff3d2909fa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetOptionValue-request)))
  "Returns md5sum for a message object of type 'SetOptionValue-request"
  "c37d6886e6fd73ac7a80b3ff3d2909fa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetOptionValue-request>)))
  "Returns full string definition for message of type '<SetOptionValue-request>"
  (cl:format cl:nil "OptionValue input~%~%================================================================================~%MSG: kortex_driver/OptionValue~%~%uint32 sensor~%uint32 option~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetOptionValue-request)))
  "Returns full string definition for message of type 'SetOptionValue-request"
  (cl:format cl:nil "OptionValue input~%~%================================================================================~%MSG: kortex_driver/OptionValue~%~%uint32 sensor~%uint32 option~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetOptionValue-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetOptionValue-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetOptionValue-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetOptionValue-response.msg.html

(cl:defclass <SetOptionValue-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetOptionValue-response (<SetOptionValue-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetOptionValue-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetOptionValue-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetOptionValue-response> is deprecated: use kortex_driver-srv:SetOptionValue-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetOptionValue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetOptionValue-response>) ostream)
  "Serializes a message object of type '<SetOptionValue-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetOptionValue-response>) istream)
  "Deserializes a message object of type '<SetOptionValue-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetOptionValue-response>)))
  "Returns string type for a service object of type '<SetOptionValue-response>"
  "kortex_driver/SetOptionValueResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetOptionValue-response)))
  "Returns string type for a service object of type 'SetOptionValue-response"
  "kortex_driver/SetOptionValueResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetOptionValue-response>)))
  "Returns md5sum for a message object of type '<SetOptionValue-response>"
  "c37d6886e6fd73ac7a80b3ff3d2909fa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetOptionValue-response)))
  "Returns md5sum for a message object of type 'SetOptionValue-response"
  "c37d6886e6fd73ac7a80b3ff3d2909fa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetOptionValue-response>)))
  "Returns full string definition for message of type '<SetOptionValue-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetOptionValue-response)))
  "Returns full string definition for message of type 'SetOptionValue-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetOptionValue-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetOptionValue-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetOptionValue-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetOptionValue)))
  'SetOptionValue-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetOptionValue)))
  'SetOptionValue-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetOptionValue)))
  "Returns string type for a service object of type '<SetOptionValue>"
  "kortex_driver/SetOptionValue")