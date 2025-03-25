; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetDeviceType-request.msg.html

(cl:defclass <GetDeviceType-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetDeviceType-request (<GetDeviceType-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetDeviceType-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetDeviceType-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetDeviceType-request> is deprecated: use kortex_driver-srv:GetDeviceType-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetDeviceType-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetDeviceType-request>) ostream)
  "Serializes a message object of type '<GetDeviceType-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetDeviceType-request>) istream)
  "Deserializes a message object of type '<GetDeviceType-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetDeviceType-request>)))
  "Returns string type for a service object of type '<GetDeviceType-request>"
  "kortex_driver/GetDeviceTypeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDeviceType-request)))
  "Returns string type for a service object of type 'GetDeviceType-request"
  "kortex_driver/GetDeviceTypeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetDeviceType-request>)))
  "Returns md5sum for a message object of type '<GetDeviceType-request>"
  "2d4eec40c5cb478115bd33d8df8d00b7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetDeviceType-request)))
  "Returns md5sum for a message object of type 'GetDeviceType-request"
  "2d4eec40c5cb478115bd33d8df8d00b7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetDeviceType-request>)))
  "Returns full string definition for message of type '<GetDeviceType-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetDeviceType-request)))
  "Returns full string definition for message of type 'GetDeviceType-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetDeviceType-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetDeviceType-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetDeviceType-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetDeviceType-response.msg.html

(cl:defclass <GetDeviceType-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:DeviceType
    :initform (cl:make-instance 'kortex_driver-msg:DeviceType)))
)

(cl:defclass GetDeviceType-response (<GetDeviceType-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetDeviceType-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetDeviceType-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetDeviceType-response> is deprecated: use kortex_driver-srv:GetDeviceType-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetDeviceType-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetDeviceType-response>) ostream)
  "Serializes a message object of type '<GetDeviceType-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetDeviceType-response>) istream)
  "Deserializes a message object of type '<GetDeviceType-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetDeviceType-response>)))
  "Returns string type for a service object of type '<GetDeviceType-response>"
  "kortex_driver/GetDeviceTypeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDeviceType-response)))
  "Returns string type for a service object of type 'GetDeviceType-response"
  "kortex_driver/GetDeviceTypeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetDeviceType-response>)))
  "Returns md5sum for a message object of type '<GetDeviceType-response>"
  "2d4eec40c5cb478115bd33d8df8d00b7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetDeviceType-response)))
  "Returns md5sum for a message object of type 'GetDeviceType-response"
  "2d4eec40c5cb478115bd33d8df8d00b7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetDeviceType-response>)))
  "Returns full string definition for message of type '<GetDeviceType-response>"
  (cl:format cl:nil "DeviceType output~%~%================================================================================~%MSG: kortex_driver/DeviceType~%~%uint32 device_type~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetDeviceType-response)))
  "Returns full string definition for message of type 'GetDeviceType-response"
  (cl:format cl:nil "DeviceType output~%~%================================================================================~%MSG: kortex_driver/DeviceType~%~%uint32 device_type~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetDeviceType-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetDeviceType-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetDeviceType-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetDeviceType)))
  'GetDeviceType-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetDeviceType)))
  'GetDeviceType-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDeviceType)))
  "Returns string type for a service object of type '<GetDeviceType>"
  "kortex_driver/GetDeviceType")