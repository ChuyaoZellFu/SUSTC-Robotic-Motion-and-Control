; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude ApplyEmergencyStop-request.msg.html

(cl:defclass <ApplyEmergencyStop-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass ApplyEmergencyStop-request (<ApplyEmergencyStop-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ApplyEmergencyStop-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ApplyEmergencyStop-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ApplyEmergencyStop-request> is deprecated: use kortex_driver-srv:ApplyEmergencyStop-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <ApplyEmergencyStop-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ApplyEmergencyStop-request>) ostream)
  "Serializes a message object of type '<ApplyEmergencyStop-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ApplyEmergencyStop-request>) istream)
  "Deserializes a message object of type '<ApplyEmergencyStop-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ApplyEmergencyStop-request>)))
  "Returns string type for a service object of type '<ApplyEmergencyStop-request>"
  "kortex_driver/ApplyEmergencyStopRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ApplyEmergencyStop-request)))
  "Returns string type for a service object of type 'ApplyEmergencyStop-request"
  "kortex_driver/ApplyEmergencyStopRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ApplyEmergencyStop-request>)))
  "Returns md5sum for a message object of type '<ApplyEmergencyStop-request>"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ApplyEmergencyStop-request)))
  "Returns md5sum for a message object of type 'ApplyEmergencyStop-request"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ApplyEmergencyStop-request>)))
  "Returns full string definition for message of type '<ApplyEmergencyStop-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ApplyEmergencyStop-request)))
  "Returns full string definition for message of type 'ApplyEmergencyStop-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ApplyEmergencyStop-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ApplyEmergencyStop-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ApplyEmergencyStop-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude ApplyEmergencyStop-response.msg.html

(cl:defclass <ApplyEmergencyStop-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass ApplyEmergencyStop-response (<ApplyEmergencyStop-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ApplyEmergencyStop-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ApplyEmergencyStop-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ApplyEmergencyStop-response> is deprecated: use kortex_driver-srv:ApplyEmergencyStop-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <ApplyEmergencyStop-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ApplyEmergencyStop-response>) ostream)
  "Serializes a message object of type '<ApplyEmergencyStop-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ApplyEmergencyStop-response>) istream)
  "Deserializes a message object of type '<ApplyEmergencyStop-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ApplyEmergencyStop-response>)))
  "Returns string type for a service object of type '<ApplyEmergencyStop-response>"
  "kortex_driver/ApplyEmergencyStopResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ApplyEmergencyStop-response)))
  "Returns string type for a service object of type 'ApplyEmergencyStop-response"
  "kortex_driver/ApplyEmergencyStopResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ApplyEmergencyStop-response>)))
  "Returns md5sum for a message object of type '<ApplyEmergencyStop-response>"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ApplyEmergencyStop-response)))
  "Returns md5sum for a message object of type 'ApplyEmergencyStop-response"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ApplyEmergencyStop-response>)))
  "Returns full string definition for message of type '<ApplyEmergencyStop-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ApplyEmergencyStop-response)))
  "Returns full string definition for message of type 'ApplyEmergencyStop-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ApplyEmergencyStop-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ApplyEmergencyStop-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ApplyEmergencyStop-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ApplyEmergencyStop)))
  'ApplyEmergencyStop-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ApplyEmergencyStop)))
  'ApplyEmergencyStop-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ApplyEmergencyStop)))
  "Returns string type for a service object of type '<ApplyEmergencyStop>"
  "kortex_driver/ApplyEmergencyStop")