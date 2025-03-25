; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude I2CWriteRegister-request.msg.html

(cl:defclass <I2CWriteRegister-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:I2CWriteRegisterParameter
    :initform (cl:make-instance 'kortex_driver-msg:I2CWriteRegisterParameter)))
)

(cl:defclass I2CWriteRegister-request (<I2CWriteRegister-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <I2CWriteRegister-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'I2CWriteRegister-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<I2CWriteRegister-request> is deprecated: use kortex_driver-srv:I2CWriteRegister-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <I2CWriteRegister-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <I2CWriteRegister-request>) ostream)
  "Serializes a message object of type '<I2CWriteRegister-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <I2CWriteRegister-request>) istream)
  "Deserializes a message object of type '<I2CWriteRegister-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<I2CWriteRegister-request>)))
  "Returns string type for a service object of type '<I2CWriteRegister-request>"
  "kortex_driver/I2CWriteRegisterRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'I2CWriteRegister-request)))
  "Returns string type for a service object of type 'I2CWriteRegister-request"
  "kortex_driver/I2CWriteRegisterRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<I2CWriteRegister-request>)))
  "Returns md5sum for a message object of type '<I2CWriteRegister-request>"
  "51d6dc72e2a979ebcd0930e5191f7f87")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'I2CWriteRegister-request)))
  "Returns md5sum for a message object of type 'I2CWriteRegister-request"
  "51d6dc72e2a979ebcd0930e5191f7f87")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<I2CWriteRegister-request>)))
  "Returns full string definition for message of type '<I2CWriteRegister-request>"
  (cl:format cl:nil "I2CWriteRegisterParameter input~%~%================================================================================~%MSG: kortex_driver/I2CWriteRegisterParameter~%~%uint32 device~%uint32 device_address~%uint32 register_address~%uint32 register_address_size~%uint32 timeout~%I2CData data~%================================================================================~%MSG: kortex_driver/I2CData~%~%uint8[] data~%uint32 size~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'I2CWriteRegister-request)))
  "Returns full string definition for message of type 'I2CWriteRegister-request"
  (cl:format cl:nil "I2CWriteRegisterParameter input~%~%================================================================================~%MSG: kortex_driver/I2CWriteRegisterParameter~%~%uint32 device~%uint32 device_address~%uint32 register_address~%uint32 register_address_size~%uint32 timeout~%I2CData data~%================================================================================~%MSG: kortex_driver/I2CData~%~%uint8[] data~%uint32 size~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <I2CWriteRegister-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <I2CWriteRegister-request>))
  "Converts a ROS message object to a list"
  (cl:list 'I2CWriteRegister-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude I2CWriteRegister-response.msg.html

(cl:defclass <I2CWriteRegister-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass I2CWriteRegister-response (<I2CWriteRegister-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <I2CWriteRegister-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'I2CWriteRegister-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<I2CWriteRegister-response> is deprecated: use kortex_driver-srv:I2CWriteRegister-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <I2CWriteRegister-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <I2CWriteRegister-response>) ostream)
  "Serializes a message object of type '<I2CWriteRegister-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <I2CWriteRegister-response>) istream)
  "Deserializes a message object of type '<I2CWriteRegister-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<I2CWriteRegister-response>)))
  "Returns string type for a service object of type '<I2CWriteRegister-response>"
  "kortex_driver/I2CWriteRegisterResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'I2CWriteRegister-response)))
  "Returns string type for a service object of type 'I2CWriteRegister-response"
  "kortex_driver/I2CWriteRegisterResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<I2CWriteRegister-response>)))
  "Returns md5sum for a message object of type '<I2CWriteRegister-response>"
  "51d6dc72e2a979ebcd0930e5191f7f87")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'I2CWriteRegister-response)))
  "Returns md5sum for a message object of type 'I2CWriteRegister-response"
  "51d6dc72e2a979ebcd0930e5191f7f87")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<I2CWriteRegister-response>)))
  "Returns full string definition for message of type '<I2CWriteRegister-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'I2CWriteRegister-response)))
  "Returns full string definition for message of type 'I2CWriteRegister-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <I2CWriteRegister-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <I2CWriteRegister-response>))
  "Converts a ROS message object to a list"
  (cl:list 'I2CWriteRegister-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'I2CWriteRegister)))
  'I2CWriteRegister-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'I2CWriteRegister)))
  'I2CWriteRegister-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'I2CWriteRegister)))
  "Returns string type for a service object of type '<I2CWriteRegister>"
  "kortex_driver/I2CWriteRegister")