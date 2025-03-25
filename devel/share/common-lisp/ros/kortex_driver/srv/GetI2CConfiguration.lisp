; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetI2CConfiguration-request.msg.html

(cl:defclass <GetI2CConfiguration-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:I2CDeviceIdentification
    :initform (cl:make-instance 'kortex_driver-msg:I2CDeviceIdentification)))
)

(cl:defclass GetI2CConfiguration-request (<GetI2CConfiguration-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetI2CConfiguration-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetI2CConfiguration-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetI2CConfiguration-request> is deprecated: use kortex_driver-srv:GetI2CConfiguration-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetI2CConfiguration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetI2CConfiguration-request>) ostream)
  "Serializes a message object of type '<GetI2CConfiguration-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetI2CConfiguration-request>) istream)
  "Deserializes a message object of type '<GetI2CConfiguration-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetI2CConfiguration-request>)))
  "Returns string type for a service object of type '<GetI2CConfiguration-request>"
  "kortex_driver/GetI2CConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetI2CConfiguration-request)))
  "Returns string type for a service object of type 'GetI2CConfiguration-request"
  "kortex_driver/GetI2CConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetI2CConfiguration-request>)))
  "Returns md5sum for a message object of type '<GetI2CConfiguration-request>"
  "d50c46d9bec318dcd7281869a6e5606a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetI2CConfiguration-request)))
  "Returns md5sum for a message object of type 'GetI2CConfiguration-request"
  "d50c46d9bec318dcd7281869a6e5606a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetI2CConfiguration-request>)))
  "Returns full string definition for message of type '<GetI2CConfiguration-request>"
  (cl:format cl:nil "I2CDeviceIdentification input~%~%================================================================================~%MSG: kortex_driver/I2CDeviceIdentification~%~%uint32 device~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetI2CConfiguration-request)))
  "Returns full string definition for message of type 'GetI2CConfiguration-request"
  (cl:format cl:nil "I2CDeviceIdentification input~%~%================================================================================~%MSG: kortex_driver/I2CDeviceIdentification~%~%uint32 device~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetI2CConfiguration-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetI2CConfiguration-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetI2CConfiguration-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetI2CConfiguration-response.msg.html

(cl:defclass <GetI2CConfiguration-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:I2CConfiguration
    :initform (cl:make-instance 'kortex_driver-msg:I2CConfiguration)))
)

(cl:defclass GetI2CConfiguration-response (<GetI2CConfiguration-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetI2CConfiguration-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetI2CConfiguration-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetI2CConfiguration-response> is deprecated: use kortex_driver-srv:GetI2CConfiguration-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetI2CConfiguration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetI2CConfiguration-response>) ostream)
  "Serializes a message object of type '<GetI2CConfiguration-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetI2CConfiguration-response>) istream)
  "Deserializes a message object of type '<GetI2CConfiguration-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetI2CConfiguration-response>)))
  "Returns string type for a service object of type '<GetI2CConfiguration-response>"
  "kortex_driver/GetI2CConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetI2CConfiguration-response)))
  "Returns string type for a service object of type 'GetI2CConfiguration-response"
  "kortex_driver/GetI2CConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetI2CConfiguration-response>)))
  "Returns md5sum for a message object of type '<GetI2CConfiguration-response>"
  "d50c46d9bec318dcd7281869a6e5606a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetI2CConfiguration-response)))
  "Returns md5sum for a message object of type 'GetI2CConfiguration-response"
  "d50c46d9bec318dcd7281869a6e5606a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetI2CConfiguration-response>)))
  "Returns full string definition for message of type '<GetI2CConfiguration-response>"
  (cl:format cl:nil "I2CConfiguration output~%~%================================================================================~%MSG: kortex_driver/I2CConfiguration~%~%uint32 device~%bool enabled~%uint32 mode~%uint32 addressing~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetI2CConfiguration-response)))
  "Returns full string definition for message of type 'GetI2CConfiguration-response"
  (cl:format cl:nil "I2CConfiguration output~%~%================================================================================~%MSG: kortex_driver/I2CConfiguration~%~%uint32 device~%bool enabled~%uint32 mode~%uint32 addressing~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetI2CConfiguration-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetI2CConfiguration-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetI2CConfiguration-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetI2CConfiguration)))
  'GetI2CConfiguration-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetI2CConfiguration)))
  'GetI2CConfiguration-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetI2CConfiguration)))
  "Returns string type for a service object of type '<GetI2CConfiguration>"
  "kortex_driver/GetI2CConfiguration")