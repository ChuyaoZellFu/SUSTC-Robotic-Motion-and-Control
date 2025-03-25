; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetControllerConfigurationMode-request.msg.html

(cl:defclass <SetControllerConfigurationMode-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:ControllerConfigurationMode
    :initform (cl:make-instance 'kortex_driver-msg:ControllerConfigurationMode)))
)

(cl:defclass SetControllerConfigurationMode-request (<SetControllerConfigurationMode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetControllerConfigurationMode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetControllerConfigurationMode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetControllerConfigurationMode-request> is deprecated: use kortex_driver-srv:SetControllerConfigurationMode-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetControllerConfigurationMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetControllerConfigurationMode-request>) ostream)
  "Serializes a message object of type '<SetControllerConfigurationMode-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetControllerConfigurationMode-request>) istream)
  "Deserializes a message object of type '<SetControllerConfigurationMode-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetControllerConfigurationMode-request>)))
  "Returns string type for a service object of type '<SetControllerConfigurationMode-request>"
  "kortex_driver/SetControllerConfigurationModeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetControllerConfigurationMode-request)))
  "Returns string type for a service object of type 'SetControllerConfigurationMode-request"
  "kortex_driver/SetControllerConfigurationModeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetControllerConfigurationMode-request>)))
  "Returns md5sum for a message object of type '<SetControllerConfigurationMode-request>"
  "a1e10ce64919ae3ba0f5b2193b693ecb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetControllerConfigurationMode-request)))
  "Returns md5sum for a message object of type 'SetControllerConfigurationMode-request"
  "a1e10ce64919ae3ba0f5b2193b693ecb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetControllerConfigurationMode-request>)))
  "Returns full string definition for message of type '<SetControllerConfigurationMode-request>"
  (cl:format cl:nil "ControllerConfigurationMode input~%~%================================================================================~%MSG: kortex_driver/ControllerConfigurationMode~%~%bool enable~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetControllerConfigurationMode-request)))
  "Returns full string definition for message of type 'SetControllerConfigurationMode-request"
  (cl:format cl:nil "ControllerConfigurationMode input~%~%================================================================================~%MSG: kortex_driver/ControllerConfigurationMode~%~%bool enable~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetControllerConfigurationMode-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetControllerConfigurationMode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetControllerConfigurationMode-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetControllerConfigurationMode-response.msg.html

(cl:defclass <SetControllerConfigurationMode-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetControllerConfigurationMode-response (<SetControllerConfigurationMode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetControllerConfigurationMode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetControllerConfigurationMode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetControllerConfigurationMode-response> is deprecated: use kortex_driver-srv:SetControllerConfigurationMode-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetControllerConfigurationMode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetControllerConfigurationMode-response>) ostream)
  "Serializes a message object of type '<SetControllerConfigurationMode-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetControllerConfigurationMode-response>) istream)
  "Deserializes a message object of type '<SetControllerConfigurationMode-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetControllerConfigurationMode-response>)))
  "Returns string type for a service object of type '<SetControllerConfigurationMode-response>"
  "kortex_driver/SetControllerConfigurationModeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetControllerConfigurationMode-response)))
  "Returns string type for a service object of type 'SetControllerConfigurationMode-response"
  "kortex_driver/SetControllerConfigurationModeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetControllerConfigurationMode-response>)))
  "Returns md5sum for a message object of type '<SetControllerConfigurationMode-response>"
  "a1e10ce64919ae3ba0f5b2193b693ecb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetControllerConfigurationMode-response)))
  "Returns md5sum for a message object of type 'SetControllerConfigurationMode-response"
  "a1e10ce64919ae3ba0f5b2193b693ecb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetControllerConfigurationMode-response>)))
  "Returns full string definition for message of type '<SetControllerConfigurationMode-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetControllerConfigurationMode-response)))
  "Returns full string definition for message of type 'SetControllerConfigurationMode-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetControllerConfigurationMode-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetControllerConfigurationMode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetControllerConfigurationMode-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetControllerConfigurationMode)))
  'SetControllerConfigurationMode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetControllerConfigurationMode)))
  'SetControllerConfigurationMode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetControllerConfigurationMode)))
  "Returns string type for a service object of type '<SetControllerConfigurationMode>"
  "kortex_driver/SetControllerConfigurationMode")