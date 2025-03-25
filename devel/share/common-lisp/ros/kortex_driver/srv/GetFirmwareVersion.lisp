; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetFirmwareVersion-request.msg.html

(cl:defclass <GetFirmwareVersion-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetFirmwareVersion-request (<GetFirmwareVersion-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetFirmwareVersion-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetFirmwareVersion-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetFirmwareVersion-request> is deprecated: use kortex_driver-srv:GetFirmwareVersion-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetFirmwareVersion-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetFirmwareVersion-request>) ostream)
  "Serializes a message object of type '<GetFirmwareVersion-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetFirmwareVersion-request>) istream)
  "Deserializes a message object of type '<GetFirmwareVersion-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetFirmwareVersion-request>)))
  "Returns string type for a service object of type '<GetFirmwareVersion-request>"
  "kortex_driver/GetFirmwareVersionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetFirmwareVersion-request)))
  "Returns string type for a service object of type 'GetFirmwareVersion-request"
  "kortex_driver/GetFirmwareVersionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetFirmwareVersion-request>)))
  "Returns md5sum for a message object of type '<GetFirmwareVersion-request>"
  "b9c18ab692345d79a149debba533958f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetFirmwareVersion-request)))
  "Returns md5sum for a message object of type 'GetFirmwareVersion-request"
  "b9c18ab692345d79a149debba533958f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetFirmwareVersion-request>)))
  "Returns full string definition for message of type '<GetFirmwareVersion-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetFirmwareVersion-request)))
  "Returns full string definition for message of type 'GetFirmwareVersion-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetFirmwareVersion-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetFirmwareVersion-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetFirmwareVersion-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetFirmwareVersion-response.msg.html

(cl:defclass <GetFirmwareVersion-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:FirmwareVersion
    :initform (cl:make-instance 'kortex_driver-msg:FirmwareVersion)))
)

(cl:defclass GetFirmwareVersion-response (<GetFirmwareVersion-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetFirmwareVersion-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetFirmwareVersion-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetFirmwareVersion-response> is deprecated: use kortex_driver-srv:GetFirmwareVersion-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetFirmwareVersion-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetFirmwareVersion-response>) ostream)
  "Serializes a message object of type '<GetFirmwareVersion-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetFirmwareVersion-response>) istream)
  "Deserializes a message object of type '<GetFirmwareVersion-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetFirmwareVersion-response>)))
  "Returns string type for a service object of type '<GetFirmwareVersion-response>"
  "kortex_driver/GetFirmwareVersionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetFirmwareVersion-response)))
  "Returns string type for a service object of type 'GetFirmwareVersion-response"
  "kortex_driver/GetFirmwareVersionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetFirmwareVersion-response>)))
  "Returns md5sum for a message object of type '<GetFirmwareVersion-response>"
  "b9c18ab692345d79a149debba533958f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetFirmwareVersion-response)))
  "Returns md5sum for a message object of type 'GetFirmwareVersion-response"
  "b9c18ab692345d79a149debba533958f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetFirmwareVersion-response>)))
  "Returns full string definition for message of type '<GetFirmwareVersion-response>"
  (cl:format cl:nil "FirmwareVersion output~%~%================================================================================~%MSG: kortex_driver/FirmwareVersion~%~%uint32 firmware_version~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetFirmwareVersion-response)))
  "Returns full string definition for message of type 'GetFirmwareVersion-response"
  (cl:format cl:nil "FirmwareVersion output~%~%================================================================================~%MSG: kortex_driver/FirmwareVersion~%~%uint32 firmware_version~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetFirmwareVersion-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetFirmwareVersion-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetFirmwareVersion-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetFirmwareVersion)))
  'GetFirmwareVersion-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetFirmwareVersion)))
  'GetFirmwareVersion-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetFirmwareVersion)))
  "Returns string type for a service object of type '<GetFirmwareVersion>"
  "kortex_driver/GetFirmwareVersion")