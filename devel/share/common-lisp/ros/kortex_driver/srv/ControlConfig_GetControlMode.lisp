; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude ControlConfig_GetControlMode-request.msg.html

(cl:defclass <ControlConfig_GetControlMode-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass ControlConfig_GetControlMode-request (<ControlConfig_GetControlMode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlConfig_GetControlMode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlConfig_GetControlMode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ControlConfig_GetControlMode-request> is deprecated: use kortex_driver-srv:ControlConfig_GetControlMode-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <ControlConfig_GetControlMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlConfig_GetControlMode-request>) ostream)
  "Serializes a message object of type '<ControlConfig_GetControlMode-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlConfig_GetControlMode-request>) istream)
  "Deserializes a message object of type '<ControlConfig_GetControlMode-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlConfig_GetControlMode-request>)))
  "Returns string type for a service object of type '<ControlConfig_GetControlMode-request>"
  "kortex_driver/ControlConfig_GetControlModeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlConfig_GetControlMode-request)))
  "Returns string type for a service object of type 'ControlConfig_GetControlMode-request"
  "kortex_driver/ControlConfig_GetControlModeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlConfig_GetControlMode-request>)))
  "Returns md5sum for a message object of type '<ControlConfig_GetControlMode-request>"
  "da0700ae55dc292dd80209569315d03f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlConfig_GetControlMode-request)))
  "Returns md5sum for a message object of type 'ControlConfig_GetControlMode-request"
  "da0700ae55dc292dd80209569315d03f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlConfig_GetControlMode-request>)))
  "Returns full string definition for message of type '<ControlConfig_GetControlMode-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlConfig_GetControlMode-request)))
  "Returns full string definition for message of type 'ControlConfig_GetControlMode-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlConfig_GetControlMode-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlConfig_GetControlMode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlConfig_GetControlMode-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude ControlConfig_GetControlMode-response.msg.html

(cl:defclass <ControlConfig_GetControlMode-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:ControlConfig_ControlModeInformation
    :initform (cl:make-instance 'kortex_driver-msg:ControlConfig_ControlModeInformation)))
)

(cl:defclass ControlConfig_GetControlMode-response (<ControlConfig_GetControlMode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlConfig_GetControlMode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlConfig_GetControlMode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ControlConfig_GetControlMode-response> is deprecated: use kortex_driver-srv:ControlConfig_GetControlMode-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <ControlConfig_GetControlMode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlConfig_GetControlMode-response>) ostream)
  "Serializes a message object of type '<ControlConfig_GetControlMode-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlConfig_GetControlMode-response>) istream)
  "Deserializes a message object of type '<ControlConfig_GetControlMode-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlConfig_GetControlMode-response>)))
  "Returns string type for a service object of type '<ControlConfig_GetControlMode-response>"
  "kortex_driver/ControlConfig_GetControlModeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlConfig_GetControlMode-response)))
  "Returns string type for a service object of type 'ControlConfig_GetControlMode-response"
  "kortex_driver/ControlConfig_GetControlModeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlConfig_GetControlMode-response>)))
  "Returns md5sum for a message object of type '<ControlConfig_GetControlMode-response>"
  "da0700ae55dc292dd80209569315d03f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlConfig_GetControlMode-response)))
  "Returns md5sum for a message object of type 'ControlConfig_GetControlMode-response"
  "da0700ae55dc292dd80209569315d03f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlConfig_GetControlMode-response>)))
  "Returns full string definition for message of type '<ControlConfig_GetControlMode-response>"
  (cl:format cl:nil "ControlConfig_ControlModeInformation output~%~%================================================================================~%MSG: kortex_driver/ControlConfig_ControlModeInformation~%~%uint32 control_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlConfig_GetControlMode-response)))
  "Returns full string definition for message of type 'ControlConfig_GetControlMode-response"
  (cl:format cl:nil "ControlConfig_ControlModeInformation output~%~%================================================================================~%MSG: kortex_driver/ControlConfig_ControlModeInformation~%~%uint32 control_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlConfig_GetControlMode-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlConfig_GetControlMode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlConfig_GetControlMode-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ControlConfig_GetControlMode)))
  'ControlConfig_GetControlMode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ControlConfig_GetControlMode)))
  'ControlConfig_GetControlMode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlConfig_GetControlMode)))
  "Returns string type for a service object of type '<ControlConfig_GetControlMode>"
  "kortex_driver/ControlConfig_GetControlMode")