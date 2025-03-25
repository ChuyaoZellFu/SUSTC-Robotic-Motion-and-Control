; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude I2CReadRegister-request.msg.html

(cl:defclass <I2CReadRegister-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:I2CReadRegisterParameter
    :initform (cl:make-instance 'kortex_driver-msg:I2CReadRegisterParameter)))
)

(cl:defclass I2CReadRegister-request (<I2CReadRegister-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <I2CReadRegister-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'I2CReadRegister-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<I2CReadRegister-request> is deprecated: use kortex_driver-srv:I2CReadRegister-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <I2CReadRegister-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <I2CReadRegister-request>) ostream)
  "Serializes a message object of type '<I2CReadRegister-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <I2CReadRegister-request>) istream)
  "Deserializes a message object of type '<I2CReadRegister-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<I2CReadRegister-request>)))
  "Returns string type for a service object of type '<I2CReadRegister-request>"
  "kortex_driver/I2CReadRegisterRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'I2CReadRegister-request)))
  "Returns string type for a service object of type 'I2CReadRegister-request"
  "kortex_driver/I2CReadRegisterRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<I2CReadRegister-request>)))
  "Returns md5sum for a message object of type '<I2CReadRegister-request>"
  "4bdf02ffe8fd0ef6ff8749422c787f42")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'I2CReadRegister-request)))
  "Returns md5sum for a message object of type 'I2CReadRegister-request"
  "4bdf02ffe8fd0ef6ff8749422c787f42")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<I2CReadRegister-request>)))
  "Returns full string definition for message of type '<I2CReadRegister-request>"
  (cl:format cl:nil "I2CReadRegisterParameter input~%~%================================================================================~%MSG: kortex_driver/I2CReadRegisterParameter~%~%uint32 device~%uint32 device_address~%uint32 register_address~%uint32 register_address_size~%uint32 size~%uint32 timeout~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'I2CReadRegister-request)))
  "Returns full string definition for message of type 'I2CReadRegister-request"
  (cl:format cl:nil "I2CReadRegisterParameter input~%~%================================================================================~%MSG: kortex_driver/I2CReadRegisterParameter~%~%uint32 device~%uint32 device_address~%uint32 register_address~%uint32 register_address_size~%uint32 size~%uint32 timeout~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <I2CReadRegister-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <I2CReadRegister-request>))
  "Converts a ROS message object to a list"
  (cl:list 'I2CReadRegister-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude I2CReadRegister-response.msg.html

(cl:defclass <I2CReadRegister-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:I2CData
    :initform (cl:make-instance 'kortex_driver-msg:I2CData)))
)

(cl:defclass I2CReadRegister-response (<I2CReadRegister-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <I2CReadRegister-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'I2CReadRegister-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<I2CReadRegister-response> is deprecated: use kortex_driver-srv:I2CReadRegister-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <I2CReadRegister-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <I2CReadRegister-response>) ostream)
  "Serializes a message object of type '<I2CReadRegister-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <I2CReadRegister-response>) istream)
  "Deserializes a message object of type '<I2CReadRegister-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<I2CReadRegister-response>)))
  "Returns string type for a service object of type '<I2CReadRegister-response>"
  "kortex_driver/I2CReadRegisterResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'I2CReadRegister-response)))
  "Returns string type for a service object of type 'I2CReadRegister-response"
  "kortex_driver/I2CReadRegisterResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<I2CReadRegister-response>)))
  "Returns md5sum for a message object of type '<I2CReadRegister-response>"
  "4bdf02ffe8fd0ef6ff8749422c787f42")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'I2CReadRegister-response)))
  "Returns md5sum for a message object of type 'I2CReadRegister-response"
  "4bdf02ffe8fd0ef6ff8749422c787f42")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<I2CReadRegister-response>)))
  "Returns full string definition for message of type '<I2CReadRegister-response>"
  (cl:format cl:nil "I2CData output~%~%================================================================================~%MSG: kortex_driver/I2CData~%~%uint8[] data~%uint32 size~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'I2CReadRegister-response)))
  "Returns full string definition for message of type 'I2CReadRegister-response"
  (cl:format cl:nil "I2CData output~%~%================================================================================~%MSG: kortex_driver/I2CData~%~%uint8[] data~%uint32 size~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <I2CReadRegister-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <I2CReadRegister-response>))
  "Converts a ROS message object to a list"
  (cl:list 'I2CReadRegister-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'I2CReadRegister)))
  'I2CReadRegister-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'I2CReadRegister)))
  'I2CReadRegister-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'I2CReadRegister)))
  "Returns string type for a service object of type '<I2CReadRegister>"
  "kortex_driver/I2CReadRegister")