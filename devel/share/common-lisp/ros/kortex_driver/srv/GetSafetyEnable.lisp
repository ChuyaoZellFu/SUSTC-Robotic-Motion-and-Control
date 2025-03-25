; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetSafetyEnable-request.msg.html

(cl:defclass <GetSafetyEnable-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:SafetyHandle
    :initform (cl:make-instance 'kortex_driver-msg:SafetyHandle)))
)

(cl:defclass GetSafetyEnable-request (<GetSafetyEnable-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetSafetyEnable-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetSafetyEnable-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetSafetyEnable-request> is deprecated: use kortex_driver-srv:GetSafetyEnable-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetSafetyEnable-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetSafetyEnable-request>) ostream)
  "Serializes a message object of type '<GetSafetyEnable-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetSafetyEnable-request>) istream)
  "Deserializes a message object of type '<GetSafetyEnable-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetSafetyEnable-request>)))
  "Returns string type for a service object of type '<GetSafetyEnable-request>"
  "kortex_driver/GetSafetyEnableRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSafetyEnable-request)))
  "Returns string type for a service object of type 'GetSafetyEnable-request"
  "kortex_driver/GetSafetyEnableRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetSafetyEnable-request>)))
  "Returns md5sum for a message object of type '<GetSafetyEnable-request>"
  "d579190d4e318f6ca36b725f2e3b7043")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetSafetyEnable-request)))
  "Returns md5sum for a message object of type 'GetSafetyEnable-request"
  "d579190d4e318f6ca36b725f2e3b7043")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetSafetyEnable-request>)))
  "Returns full string definition for message of type '<GetSafetyEnable-request>"
  (cl:format cl:nil "SafetyHandle input~%~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetSafetyEnable-request)))
  "Returns full string definition for message of type 'GetSafetyEnable-request"
  (cl:format cl:nil "SafetyHandle input~%~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetSafetyEnable-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetSafetyEnable-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetSafetyEnable-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetSafetyEnable-response.msg.html

(cl:defclass <GetSafetyEnable-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:SafetyEnable
    :initform (cl:make-instance 'kortex_driver-msg:SafetyEnable)))
)

(cl:defclass GetSafetyEnable-response (<GetSafetyEnable-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetSafetyEnable-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetSafetyEnable-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetSafetyEnable-response> is deprecated: use kortex_driver-srv:GetSafetyEnable-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetSafetyEnable-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetSafetyEnable-response>) ostream)
  "Serializes a message object of type '<GetSafetyEnable-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetSafetyEnable-response>) istream)
  "Deserializes a message object of type '<GetSafetyEnable-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetSafetyEnable-response>)))
  "Returns string type for a service object of type '<GetSafetyEnable-response>"
  "kortex_driver/GetSafetyEnableResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSafetyEnable-response)))
  "Returns string type for a service object of type 'GetSafetyEnable-response"
  "kortex_driver/GetSafetyEnableResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetSafetyEnable-response>)))
  "Returns md5sum for a message object of type '<GetSafetyEnable-response>"
  "d579190d4e318f6ca36b725f2e3b7043")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetSafetyEnable-response)))
  "Returns md5sum for a message object of type 'GetSafetyEnable-response"
  "d579190d4e318f6ca36b725f2e3b7043")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetSafetyEnable-response>)))
  "Returns full string definition for message of type '<GetSafetyEnable-response>"
  (cl:format cl:nil "SafetyEnable output~%~%================================================================================~%MSG: kortex_driver/SafetyEnable~%~%SafetyHandle handle~%bool enable~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetSafetyEnable-response)))
  "Returns full string definition for message of type 'GetSafetyEnable-response"
  (cl:format cl:nil "SafetyEnable output~%~%================================================================================~%MSG: kortex_driver/SafetyEnable~%~%SafetyHandle handle~%bool enable~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetSafetyEnable-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetSafetyEnable-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetSafetyEnable-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetSafetyEnable)))
  'GetSafetyEnable-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetSafetyEnable)))
  'GetSafetyEnable-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSafetyEnable)))
  "Returns string type for a service object of type '<GetSafetyEnable>"
  "kortex_driver/GetSafetyEnable")