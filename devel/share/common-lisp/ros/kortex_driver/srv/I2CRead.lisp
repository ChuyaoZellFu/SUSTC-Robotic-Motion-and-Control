; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude I2CRead-request.msg.html

(cl:defclass <I2CRead-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:I2CReadParameter
    :initform (cl:make-instance 'kortex_driver-msg:I2CReadParameter)))
)

(cl:defclass I2CRead-request (<I2CRead-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <I2CRead-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'I2CRead-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<I2CRead-request> is deprecated: use kortex_driver-srv:I2CRead-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <I2CRead-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <I2CRead-request>) ostream)
  "Serializes a message object of type '<I2CRead-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <I2CRead-request>) istream)
  "Deserializes a message object of type '<I2CRead-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<I2CRead-request>)))
  "Returns string type for a service object of type '<I2CRead-request>"
  "kortex_driver/I2CReadRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'I2CRead-request)))
  "Returns string type for a service object of type 'I2CRead-request"
  "kortex_driver/I2CReadRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<I2CRead-request>)))
  "Returns md5sum for a message object of type '<I2CRead-request>"
  "76252f712912deacb218cb19eaf79993")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'I2CRead-request)))
  "Returns md5sum for a message object of type 'I2CRead-request"
  "76252f712912deacb218cb19eaf79993")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<I2CRead-request>)))
  "Returns full string definition for message of type '<I2CRead-request>"
  (cl:format cl:nil "I2CReadParameter input~%~%================================================================================~%MSG: kortex_driver/I2CReadParameter~%~%uint32 device~%uint32 device_address~%uint32 size~%uint32 timeout~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'I2CRead-request)))
  "Returns full string definition for message of type 'I2CRead-request"
  (cl:format cl:nil "I2CReadParameter input~%~%================================================================================~%MSG: kortex_driver/I2CReadParameter~%~%uint32 device~%uint32 device_address~%uint32 size~%uint32 timeout~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <I2CRead-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <I2CRead-request>))
  "Converts a ROS message object to a list"
  (cl:list 'I2CRead-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude I2CRead-response.msg.html

(cl:defclass <I2CRead-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:I2CData
    :initform (cl:make-instance 'kortex_driver-msg:I2CData)))
)

(cl:defclass I2CRead-response (<I2CRead-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <I2CRead-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'I2CRead-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<I2CRead-response> is deprecated: use kortex_driver-srv:I2CRead-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <I2CRead-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <I2CRead-response>) ostream)
  "Serializes a message object of type '<I2CRead-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <I2CRead-response>) istream)
  "Deserializes a message object of type '<I2CRead-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<I2CRead-response>)))
  "Returns string type for a service object of type '<I2CRead-response>"
  "kortex_driver/I2CReadResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'I2CRead-response)))
  "Returns string type for a service object of type 'I2CRead-response"
  "kortex_driver/I2CReadResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<I2CRead-response>)))
  "Returns md5sum for a message object of type '<I2CRead-response>"
  "76252f712912deacb218cb19eaf79993")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'I2CRead-response)))
  "Returns md5sum for a message object of type 'I2CRead-response"
  "76252f712912deacb218cb19eaf79993")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<I2CRead-response>)))
  "Returns full string definition for message of type '<I2CRead-response>"
  (cl:format cl:nil "I2CData output~%~%================================================================================~%MSG: kortex_driver/I2CData~%~%uint8[] data~%uint32 size~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'I2CRead-response)))
  "Returns full string definition for message of type 'I2CRead-response"
  (cl:format cl:nil "I2CData output~%~%================================================================================~%MSG: kortex_driver/I2CData~%~%uint8[] data~%uint32 size~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <I2CRead-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <I2CRead-response>))
  "Converts a ROS message object to a list"
  (cl:list 'I2CRead-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'I2CRead)))
  'I2CRead-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'I2CRead)))
  'I2CRead-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'I2CRead)))
  "Returns string type for a service object of type '<I2CRead>"
  "kortex_driver/I2CRead")