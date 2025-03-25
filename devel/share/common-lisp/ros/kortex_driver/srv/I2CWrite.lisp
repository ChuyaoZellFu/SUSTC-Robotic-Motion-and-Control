; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude I2CWrite-request.msg.html

(cl:defclass <I2CWrite-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:I2CWriteParameter
    :initform (cl:make-instance 'kortex_driver-msg:I2CWriteParameter)))
)

(cl:defclass I2CWrite-request (<I2CWrite-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <I2CWrite-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'I2CWrite-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<I2CWrite-request> is deprecated: use kortex_driver-srv:I2CWrite-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <I2CWrite-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <I2CWrite-request>) ostream)
  "Serializes a message object of type '<I2CWrite-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <I2CWrite-request>) istream)
  "Deserializes a message object of type '<I2CWrite-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<I2CWrite-request>)))
  "Returns string type for a service object of type '<I2CWrite-request>"
  "kortex_driver/I2CWriteRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'I2CWrite-request)))
  "Returns string type for a service object of type 'I2CWrite-request"
  "kortex_driver/I2CWriteRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<I2CWrite-request>)))
  "Returns md5sum for a message object of type '<I2CWrite-request>"
  "976795fac6276cbe1f1d899c54c1e043")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'I2CWrite-request)))
  "Returns md5sum for a message object of type 'I2CWrite-request"
  "976795fac6276cbe1f1d899c54c1e043")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<I2CWrite-request>)))
  "Returns full string definition for message of type '<I2CWrite-request>"
  (cl:format cl:nil "I2CWriteParameter input~%~%================================================================================~%MSG: kortex_driver/I2CWriteParameter~%~%uint32 device~%uint32 device_address~%uint32 timeout~%I2CData data~%================================================================================~%MSG: kortex_driver/I2CData~%~%uint8[] data~%uint32 size~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'I2CWrite-request)))
  "Returns full string definition for message of type 'I2CWrite-request"
  (cl:format cl:nil "I2CWriteParameter input~%~%================================================================================~%MSG: kortex_driver/I2CWriteParameter~%~%uint32 device~%uint32 device_address~%uint32 timeout~%I2CData data~%================================================================================~%MSG: kortex_driver/I2CData~%~%uint8[] data~%uint32 size~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <I2CWrite-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <I2CWrite-request>))
  "Converts a ROS message object to a list"
  (cl:list 'I2CWrite-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude I2CWrite-response.msg.html

(cl:defclass <I2CWrite-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass I2CWrite-response (<I2CWrite-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <I2CWrite-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'I2CWrite-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<I2CWrite-response> is deprecated: use kortex_driver-srv:I2CWrite-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <I2CWrite-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <I2CWrite-response>) ostream)
  "Serializes a message object of type '<I2CWrite-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <I2CWrite-response>) istream)
  "Deserializes a message object of type '<I2CWrite-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<I2CWrite-response>)))
  "Returns string type for a service object of type '<I2CWrite-response>"
  "kortex_driver/I2CWriteResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'I2CWrite-response)))
  "Returns string type for a service object of type 'I2CWrite-response"
  "kortex_driver/I2CWriteResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<I2CWrite-response>)))
  "Returns md5sum for a message object of type '<I2CWrite-response>"
  "976795fac6276cbe1f1d899c54c1e043")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'I2CWrite-response)))
  "Returns md5sum for a message object of type 'I2CWrite-response"
  "976795fac6276cbe1f1d899c54c1e043")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<I2CWrite-response>)))
  "Returns full string definition for message of type '<I2CWrite-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'I2CWrite-response)))
  "Returns full string definition for message of type 'I2CWrite-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <I2CWrite-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <I2CWrite-response>))
  "Converts a ROS message object to a list"
  (cl:list 'I2CWrite-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'I2CWrite)))
  'I2CWrite-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'I2CWrite)))
  'I2CWrite-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'I2CWrite)))
  "Returns string type for a service object of type '<I2CWrite>"
  "kortex_driver/I2CWrite")