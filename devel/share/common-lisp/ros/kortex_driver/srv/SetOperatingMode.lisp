; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetOperatingMode-request.msg.html

(cl:defclass <SetOperatingMode-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:OperatingModeInformation
    :initform (cl:make-instance 'kortex_driver-msg:OperatingModeInformation)))
)

(cl:defclass SetOperatingMode-request (<SetOperatingMode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetOperatingMode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetOperatingMode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetOperatingMode-request> is deprecated: use kortex_driver-srv:SetOperatingMode-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetOperatingMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetOperatingMode-request>) ostream)
  "Serializes a message object of type '<SetOperatingMode-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetOperatingMode-request>) istream)
  "Deserializes a message object of type '<SetOperatingMode-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetOperatingMode-request>)))
  "Returns string type for a service object of type '<SetOperatingMode-request>"
  "kortex_driver/SetOperatingModeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetOperatingMode-request)))
  "Returns string type for a service object of type 'SetOperatingMode-request"
  "kortex_driver/SetOperatingModeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetOperatingMode-request>)))
  "Returns md5sum for a message object of type '<SetOperatingMode-request>"
  "c4131bf69b8a60653e5e053fa2ce9151")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetOperatingMode-request)))
  "Returns md5sum for a message object of type 'SetOperatingMode-request"
  "c4131bf69b8a60653e5e053fa2ce9151")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetOperatingMode-request>)))
  "Returns full string definition for message of type '<SetOperatingMode-request>"
  (cl:format cl:nil "OperatingModeInformation input~%~%================================================================================~%MSG: kortex_driver/OperatingModeInformation~%~%uint32 operating_mode~%DeviceHandle device_handle~%================================================================================~%MSG: kortex_driver/DeviceHandle~%~%uint32 device_type~%uint32 device_identifier~%uint32 order~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetOperatingMode-request)))
  "Returns full string definition for message of type 'SetOperatingMode-request"
  (cl:format cl:nil "OperatingModeInformation input~%~%================================================================================~%MSG: kortex_driver/OperatingModeInformation~%~%uint32 operating_mode~%DeviceHandle device_handle~%================================================================================~%MSG: kortex_driver/DeviceHandle~%~%uint32 device_type~%uint32 device_identifier~%uint32 order~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetOperatingMode-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetOperatingMode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetOperatingMode-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetOperatingMode-response.msg.html

(cl:defclass <SetOperatingMode-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetOperatingMode-response (<SetOperatingMode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetOperatingMode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetOperatingMode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetOperatingMode-response> is deprecated: use kortex_driver-srv:SetOperatingMode-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetOperatingMode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetOperatingMode-response>) ostream)
  "Serializes a message object of type '<SetOperatingMode-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetOperatingMode-response>) istream)
  "Deserializes a message object of type '<SetOperatingMode-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetOperatingMode-response>)))
  "Returns string type for a service object of type '<SetOperatingMode-response>"
  "kortex_driver/SetOperatingModeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetOperatingMode-response)))
  "Returns string type for a service object of type 'SetOperatingMode-response"
  "kortex_driver/SetOperatingModeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetOperatingMode-response>)))
  "Returns md5sum for a message object of type '<SetOperatingMode-response>"
  "c4131bf69b8a60653e5e053fa2ce9151")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetOperatingMode-response)))
  "Returns md5sum for a message object of type 'SetOperatingMode-response"
  "c4131bf69b8a60653e5e053fa2ce9151")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetOperatingMode-response>)))
  "Returns full string definition for message of type '<SetOperatingMode-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetOperatingMode-response)))
  "Returns full string definition for message of type 'SetOperatingMode-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetOperatingMode-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetOperatingMode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetOperatingMode-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetOperatingMode)))
  'SetOperatingMode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetOperatingMode)))
  'SetOperatingMode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetOperatingMode)))
  "Returns string type for a service object of type '<SetOperatingMode>"
  "kortex_driver/SetOperatingMode")