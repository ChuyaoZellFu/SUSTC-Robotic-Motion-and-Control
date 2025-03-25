; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetOperatingMode-request.msg.html

(cl:defclass <GetOperatingMode-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetOperatingMode-request (<GetOperatingMode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetOperatingMode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetOperatingMode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetOperatingMode-request> is deprecated: use kortex_driver-srv:GetOperatingMode-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetOperatingMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetOperatingMode-request>) ostream)
  "Serializes a message object of type '<GetOperatingMode-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetOperatingMode-request>) istream)
  "Deserializes a message object of type '<GetOperatingMode-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetOperatingMode-request>)))
  "Returns string type for a service object of type '<GetOperatingMode-request>"
  "kortex_driver/GetOperatingModeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetOperatingMode-request)))
  "Returns string type for a service object of type 'GetOperatingMode-request"
  "kortex_driver/GetOperatingModeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetOperatingMode-request>)))
  "Returns md5sum for a message object of type '<GetOperatingMode-request>"
  "fe14b4b15e0f48ffd6cf705ff5a59c40")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetOperatingMode-request)))
  "Returns md5sum for a message object of type 'GetOperatingMode-request"
  "fe14b4b15e0f48ffd6cf705ff5a59c40")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetOperatingMode-request>)))
  "Returns full string definition for message of type '<GetOperatingMode-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetOperatingMode-request)))
  "Returns full string definition for message of type 'GetOperatingMode-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetOperatingMode-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetOperatingMode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetOperatingMode-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetOperatingMode-response.msg.html

(cl:defclass <GetOperatingMode-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:OperatingModeInformation
    :initform (cl:make-instance 'kortex_driver-msg:OperatingModeInformation)))
)

(cl:defclass GetOperatingMode-response (<GetOperatingMode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetOperatingMode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetOperatingMode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetOperatingMode-response> is deprecated: use kortex_driver-srv:GetOperatingMode-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetOperatingMode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetOperatingMode-response>) ostream)
  "Serializes a message object of type '<GetOperatingMode-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetOperatingMode-response>) istream)
  "Deserializes a message object of type '<GetOperatingMode-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetOperatingMode-response>)))
  "Returns string type for a service object of type '<GetOperatingMode-response>"
  "kortex_driver/GetOperatingModeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetOperatingMode-response)))
  "Returns string type for a service object of type 'GetOperatingMode-response"
  "kortex_driver/GetOperatingModeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetOperatingMode-response>)))
  "Returns md5sum for a message object of type '<GetOperatingMode-response>"
  "fe14b4b15e0f48ffd6cf705ff5a59c40")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetOperatingMode-response)))
  "Returns md5sum for a message object of type 'GetOperatingMode-response"
  "fe14b4b15e0f48ffd6cf705ff5a59c40")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetOperatingMode-response>)))
  "Returns full string definition for message of type '<GetOperatingMode-response>"
  (cl:format cl:nil "OperatingModeInformation output~%~%================================================================================~%MSG: kortex_driver/OperatingModeInformation~%~%uint32 operating_mode~%DeviceHandle device_handle~%================================================================================~%MSG: kortex_driver/DeviceHandle~%~%uint32 device_type~%uint32 device_identifier~%uint32 order~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetOperatingMode-response)))
  "Returns full string definition for message of type 'GetOperatingMode-response"
  (cl:format cl:nil "OperatingModeInformation output~%~%================================================================================~%MSG: kortex_driver/OperatingModeInformation~%~%uint32 operating_mode~%DeviceHandle device_handle~%================================================================================~%MSG: kortex_driver/DeviceHandle~%~%uint32 device_type~%uint32 device_identifier~%uint32 order~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetOperatingMode-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetOperatingMode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetOperatingMode-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetOperatingMode)))
  'GetOperatingMode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetOperatingMode)))
  'GetOperatingMode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetOperatingMode)))
  "Returns string type for a service object of type '<GetOperatingMode>"
  "kortex_driver/GetOperatingMode")