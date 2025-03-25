; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetGPIOState-request.msg.html

(cl:defclass <SetGPIOState-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:GPIOState
    :initform (cl:make-instance 'kortex_driver-msg:GPIOState)))
)

(cl:defclass SetGPIOState-request (<SetGPIOState-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetGPIOState-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetGPIOState-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetGPIOState-request> is deprecated: use kortex_driver-srv:SetGPIOState-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetGPIOState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetGPIOState-request>) ostream)
  "Serializes a message object of type '<SetGPIOState-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetGPIOState-request>) istream)
  "Deserializes a message object of type '<SetGPIOState-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetGPIOState-request>)))
  "Returns string type for a service object of type '<SetGPIOState-request>"
  "kortex_driver/SetGPIOStateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetGPIOState-request)))
  "Returns string type for a service object of type 'SetGPIOState-request"
  "kortex_driver/SetGPIOStateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetGPIOState-request>)))
  "Returns md5sum for a message object of type '<SetGPIOState-request>"
  "22e0e537e24f27ee3c77b3e42187e849")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetGPIOState-request)))
  "Returns md5sum for a message object of type 'SetGPIOState-request"
  "22e0e537e24f27ee3c77b3e42187e849")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetGPIOState-request>)))
  "Returns full string definition for message of type '<SetGPIOState-request>"
  (cl:format cl:nil "GPIOState input~%~%================================================================================~%MSG: kortex_driver/GPIOState~%~%uint32 identifier~%uint32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetGPIOState-request)))
  "Returns full string definition for message of type 'SetGPIOState-request"
  (cl:format cl:nil "GPIOState input~%~%================================================================================~%MSG: kortex_driver/GPIOState~%~%uint32 identifier~%uint32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetGPIOState-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetGPIOState-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetGPIOState-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetGPIOState-response.msg.html

(cl:defclass <SetGPIOState-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetGPIOState-response (<SetGPIOState-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetGPIOState-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetGPIOState-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetGPIOState-response> is deprecated: use kortex_driver-srv:SetGPIOState-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetGPIOState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetGPIOState-response>) ostream)
  "Serializes a message object of type '<SetGPIOState-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetGPIOState-response>) istream)
  "Deserializes a message object of type '<SetGPIOState-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetGPIOState-response>)))
  "Returns string type for a service object of type '<SetGPIOState-response>"
  "kortex_driver/SetGPIOStateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetGPIOState-response)))
  "Returns string type for a service object of type 'SetGPIOState-response"
  "kortex_driver/SetGPIOStateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetGPIOState-response>)))
  "Returns md5sum for a message object of type '<SetGPIOState-response>"
  "22e0e537e24f27ee3c77b3e42187e849")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetGPIOState-response)))
  "Returns md5sum for a message object of type 'SetGPIOState-response"
  "22e0e537e24f27ee3c77b3e42187e849")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetGPIOState-response>)))
  "Returns full string definition for message of type '<SetGPIOState-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetGPIOState-response)))
  "Returns full string definition for message of type 'SetGPIOState-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetGPIOState-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetGPIOState-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetGPIOState-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetGPIOState)))
  'SetGPIOState-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetGPIOState)))
  'SetGPIOState-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetGPIOState)))
  "Returns string type for a service object of type '<SetGPIOState>"
  "kortex_driver/SetGPIOState")