; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetControllerConfigurationMode-request.msg.html

(cl:defclass <GetControllerConfigurationMode-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetControllerConfigurationMode-request (<GetControllerConfigurationMode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetControllerConfigurationMode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetControllerConfigurationMode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetControllerConfigurationMode-request> is deprecated: use kortex_driver-srv:GetControllerConfigurationMode-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetControllerConfigurationMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetControllerConfigurationMode-request>) ostream)
  "Serializes a message object of type '<GetControllerConfigurationMode-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetControllerConfigurationMode-request>) istream)
  "Deserializes a message object of type '<GetControllerConfigurationMode-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetControllerConfigurationMode-request>)))
  "Returns string type for a service object of type '<GetControllerConfigurationMode-request>"
  "kortex_driver/GetControllerConfigurationModeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetControllerConfigurationMode-request)))
  "Returns string type for a service object of type 'GetControllerConfigurationMode-request"
  "kortex_driver/GetControllerConfigurationModeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetControllerConfigurationMode-request>)))
  "Returns md5sum for a message object of type '<GetControllerConfigurationMode-request>"
  "f62e2e25b47b1394acdb18945114f7ef")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetControllerConfigurationMode-request)))
  "Returns md5sum for a message object of type 'GetControllerConfigurationMode-request"
  "f62e2e25b47b1394acdb18945114f7ef")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetControllerConfigurationMode-request>)))
  "Returns full string definition for message of type '<GetControllerConfigurationMode-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetControllerConfigurationMode-request)))
  "Returns full string definition for message of type 'GetControllerConfigurationMode-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetControllerConfigurationMode-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetControllerConfigurationMode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetControllerConfigurationMode-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetControllerConfigurationMode-response.msg.html

(cl:defclass <GetControllerConfigurationMode-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:ControllerConfigurationMode
    :initform (cl:make-instance 'kortex_driver-msg:ControllerConfigurationMode)))
)

(cl:defclass GetControllerConfigurationMode-response (<GetControllerConfigurationMode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetControllerConfigurationMode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetControllerConfigurationMode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetControllerConfigurationMode-response> is deprecated: use kortex_driver-srv:GetControllerConfigurationMode-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetControllerConfigurationMode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetControllerConfigurationMode-response>) ostream)
  "Serializes a message object of type '<GetControllerConfigurationMode-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetControllerConfigurationMode-response>) istream)
  "Deserializes a message object of type '<GetControllerConfigurationMode-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetControllerConfigurationMode-response>)))
  "Returns string type for a service object of type '<GetControllerConfigurationMode-response>"
  "kortex_driver/GetControllerConfigurationModeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetControllerConfigurationMode-response)))
  "Returns string type for a service object of type 'GetControllerConfigurationMode-response"
  "kortex_driver/GetControllerConfigurationModeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetControllerConfigurationMode-response>)))
  "Returns md5sum for a message object of type '<GetControllerConfigurationMode-response>"
  "f62e2e25b47b1394acdb18945114f7ef")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetControllerConfigurationMode-response)))
  "Returns md5sum for a message object of type 'GetControllerConfigurationMode-response"
  "f62e2e25b47b1394acdb18945114f7ef")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetControllerConfigurationMode-response>)))
  "Returns full string definition for message of type '<GetControllerConfigurationMode-response>"
  (cl:format cl:nil "ControllerConfigurationMode output~%~%================================================================================~%MSG: kortex_driver/ControllerConfigurationMode~%~%bool enable~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetControllerConfigurationMode-response)))
  "Returns full string definition for message of type 'GetControllerConfigurationMode-response"
  (cl:format cl:nil "ControllerConfigurationMode output~%~%================================================================================~%MSG: kortex_driver/ControllerConfigurationMode~%~%bool enable~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetControllerConfigurationMode-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetControllerConfigurationMode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetControllerConfigurationMode-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetControllerConfigurationMode)))
  'GetControllerConfigurationMode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetControllerConfigurationMode)))
  'GetControllerConfigurationMode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetControllerConfigurationMode)))
  "Returns string type for a service object of type '<GetControllerConfigurationMode>"
  "kortex_driver/GetControllerConfigurationMode")