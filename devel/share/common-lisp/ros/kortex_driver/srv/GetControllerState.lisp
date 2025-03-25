; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetControllerState-request.msg.html

(cl:defclass <GetControllerState-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:ControllerHandle
    :initform (cl:make-instance 'kortex_driver-msg:ControllerHandle)))
)

(cl:defclass GetControllerState-request (<GetControllerState-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetControllerState-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetControllerState-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetControllerState-request> is deprecated: use kortex_driver-srv:GetControllerState-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetControllerState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetControllerState-request>) ostream)
  "Serializes a message object of type '<GetControllerState-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetControllerState-request>) istream)
  "Deserializes a message object of type '<GetControllerState-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetControllerState-request>)))
  "Returns string type for a service object of type '<GetControllerState-request>"
  "kortex_driver/GetControllerStateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetControllerState-request)))
  "Returns string type for a service object of type 'GetControllerState-request"
  "kortex_driver/GetControllerStateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetControllerState-request>)))
  "Returns md5sum for a message object of type '<GetControllerState-request>"
  "c3a3b583b8a6198c6f61bb7c1bde10f0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetControllerState-request)))
  "Returns md5sum for a message object of type 'GetControllerState-request"
  "c3a3b583b8a6198c6f61bb7c1bde10f0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetControllerState-request>)))
  "Returns full string definition for message of type '<GetControllerState-request>"
  (cl:format cl:nil "ControllerHandle input~%~%================================================================================~%MSG: kortex_driver/ControllerHandle~%~%uint32 type~%uint32 controller_identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetControllerState-request)))
  "Returns full string definition for message of type 'GetControllerState-request"
  (cl:format cl:nil "ControllerHandle input~%~%================================================================================~%MSG: kortex_driver/ControllerHandle~%~%uint32 type~%uint32 controller_identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetControllerState-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetControllerState-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetControllerState-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetControllerState-response.msg.html

(cl:defclass <GetControllerState-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:ControllerState
    :initform (cl:make-instance 'kortex_driver-msg:ControllerState)))
)

(cl:defclass GetControllerState-response (<GetControllerState-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetControllerState-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetControllerState-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetControllerState-response> is deprecated: use kortex_driver-srv:GetControllerState-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetControllerState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetControllerState-response>) ostream)
  "Serializes a message object of type '<GetControllerState-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetControllerState-response>) istream)
  "Deserializes a message object of type '<GetControllerState-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetControllerState-response>)))
  "Returns string type for a service object of type '<GetControllerState-response>"
  "kortex_driver/GetControllerStateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetControllerState-response)))
  "Returns string type for a service object of type 'GetControllerState-response"
  "kortex_driver/GetControllerStateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetControllerState-response>)))
  "Returns md5sum for a message object of type '<GetControllerState-response>"
  "c3a3b583b8a6198c6f61bb7c1bde10f0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetControllerState-response)))
  "Returns md5sum for a message object of type 'GetControllerState-response"
  "c3a3b583b8a6198c6f61bb7c1bde10f0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetControllerState-response>)))
  "Returns full string definition for message of type '<GetControllerState-response>"
  (cl:format cl:nil "ControllerState output~%~%================================================================================~%MSG: kortex_driver/ControllerState~%~%ControllerHandle handle~%uint32 event_type~%================================================================================~%MSG: kortex_driver/ControllerHandle~%~%uint32 type~%uint32 controller_identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetControllerState-response)))
  "Returns full string definition for message of type 'GetControllerState-response"
  (cl:format cl:nil "ControllerState output~%~%================================================================================~%MSG: kortex_driver/ControllerState~%~%ControllerHandle handle~%uint32 event_type~%================================================================================~%MSG: kortex_driver/ControllerHandle~%~%uint32 type~%uint32 controller_identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetControllerState-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetControllerState-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetControllerState-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetControllerState)))
  'GetControllerState-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetControllerState)))
  'GetControllerState-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetControllerState)))
  "Returns string type for a service object of type '<GetControllerState>"
  "kortex_driver/GetControllerState")