; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetTorqueOffset-request.msg.html

(cl:defclass <SetTorqueOffset-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:TorqueOffset
    :initform (cl:make-instance 'kortex_driver-msg:TorqueOffset)))
)

(cl:defclass SetTorqueOffset-request (<SetTorqueOffset-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetTorqueOffset-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetTorqueOffset-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetTorqueOffset-request> is deprecated: use kortex_driver-srv:SetTorqueOffset-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetTorqueOffset-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetTorqueOffset-request>) ostream)
  "Serializes a message object of type '<SetTorqueOffset-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetTorqueOffset-request>) istream)
  "Deserializes a message object of type '<SetTorqueOffset-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetTorqueOffset-request>)))
  "Returns string type for a service object of type '<SetTorqueOffset-request>"
  "kortex_driver/SetTorqueOffsetRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetTorqueOffset-request)))
  "Returns string type for a service object of type 'SetTorqueOffset-request"
  "kortex_driver/SetTorqueOffsetRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetTorqueOffset-request>)))
  "Returns md5sum for a message object of type '<SetTorqueOffset-request>"
  "8732fa2318b2bb30f603b1c319396cd0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetTorqueOffset-request)))
  "Returns md5sum for a message object of type 'SetTorqueOffset-request"
  "8732fa2318b2bb30f603b1c319396cd0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetTorqueOffset-request>)))
  "Returns full string definition for message of type '<SetTorqueOffset-request>"
  (cl:format cl:nil "TorqueOffset input~%~%================================================================================~%MSG: kortex_driver/TorqueOffset~%~%float32 torque_offset~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetTorqueOffset-request)))
  "Returns full string definition for message of type 'SetTorqueOffset-request"
  (cl:format cl:nil "TorqueOffset input~%~%================================================================================~%MSG: kortex_driver/TorqueOffset~%~%float32 torque_offset~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetTorqueOffset-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetTorqueOffset-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetTorqueOffset-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetTorqueOffset-response.msg.html

(cl:defclass <SetTorqueOffset-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetTorqueOffset-response (<SetTorqueOffset-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetTorqueOffset-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetTorqueOffset-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetTorqueOffset-response> is deprecated: use kortex_driver-srv:SetTorqueOffset-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetTorqueOffset-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetTorqueOffset-response>) ostream)
  "Serializes a message object of type '<SetTorqueOffset-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetTorqueOffset-response>) istream)
  "Deserializes a message object of type '<SetTorqueOffset-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetTorqueOffset-response>)))
  "Returns string type for a service object of type '<SetTorqueOffset-response>"
  "kortex_driver/SetTorqueOffsetResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetTorqueOffset-response)))
  "Returns string type for a service object of type 'SetTorqueOffset-response"
  "kortex_driver/SetTorqueOffsetResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetTorqueOffset-response>)))
  "Returns md5sum for a message object of type '<SetTorqueOffset-response>"
  "8732fa2318b2bb30f603b1c319396cd0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetTorqueOffset-response)))
  "Returns md5sum for a message object of type 'SetTorqueOffset-response"
  "8732fa2318b2bb30f603b1c319396cd0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetTorqueOffset-response>)))
  "Returns full string definition for message of type '<SetTorqueOffset-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetTorqueOffset-response)))
  "Returns full string definition for message of type 'SetTorqueOffset-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetTorqueOffset-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetTorqueOffset-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetTorqueOffset-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetTorqueOffset)))
  'SetTorqueOffset-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetTorqueOffset)))
  'SetTorqueOffset-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetTorqueOffset)))
  "Returns string type for a service object of type '<SetTorqueOffset>"
  "kortex_driver/SetTorqueOffset")