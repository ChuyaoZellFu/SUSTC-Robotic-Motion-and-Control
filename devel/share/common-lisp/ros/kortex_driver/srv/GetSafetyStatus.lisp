; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetSafetyStatus-request.msg.html

(cl:defclass <GetSafetyStatus-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:SafetyHandle
    :initform (cl:make-instance 'kortex_driver-msg:SafetyHandle)))
)

(cl:defclass GetSafetyStatus-request (<GetSafetyStatus-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetSafetyStatus-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetSafetyStatus-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetSafetyStatus-request> is deprecated: use kortex_driver-srv:GetSafetyStatus-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetSafetyStatus-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetSafetyStatus-request>) ostream)
  "Serializes a message object of type '<GetSafetyStatus-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetSafetyStatus-request>) istream)
  "Deserializes a message object of type '<GetSafetyStatus-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetSafetyStatus-request>)))
  "Returns string type for a service object of type '<GetSafetyStatus-request>"
  "kortex_driver/GetSafetyStatusRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSafetyStatus-request)))
  "Returns string type for a service object of type 'GetSafetyStatus-request"
  "kortex_driver/GetSafetyStatusRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetSafetyStatus-request>)))
  "Returns md5sum for a message object of type '<GetSafetyStatus-request>"
  "0b50aa56e0379c3a0c319bd4962efe2e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetSafetyStatus-request)))
  "Returns md5sum for a message object of type 'GetSafetyStatus-request"
  "0b50aa56e0379c3a0c319bd4962efe2e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetSafetyStatus-request>)))
  "Returns full string definition for message of type '<GetSafetyStatus-request>"
  (cl:format cl:nil "SafetyHandle input~%~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetSafetyStatus-request)))
  "Returns full string definition for message of type 'GetSafetyStatus-request"
  (cl:format cl:nil "SafetyHandle input~%~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetSafetyStatus-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetSafetyStatus-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetSafetyStatus-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetSafetyStatus-response.msg.html

(cl:defclass <GetSafetyStatus-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:SafetyStatus
    :initform (cl:make-instance 'kortex_driver-msg:SafetyStatus)))
)

(cl:defclass GetSafetyStatus-response (<GetSafetyStatus-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetSafetyStatus-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetSafetyStatus-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetSafetyStatus-response> is deprecated: use kortex_driver-srv:GetSafetyStatus-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetSafetyStatus-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetSafetyStatus-response>) ostream)
  "Serializes a message object of type '<GetSafetyStatus-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetSafetyStatus-response>) istream)
  "Deserializes a message object of type '<GetSafetyStatus-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetSafetyStatus-response>)))
  "Returns string type for a service object of type '<GetSafetyStatus-response>"
  "kortex_driver/GetSafetyStatusResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSafetyStatus-response)))
  "Returns string type for a service object of type 'GetSafetyStatus-response"
  "kortex_driver/GetSafetyStatusResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetSafetyStatus-response>)))
  "Returns md5sum for a message object of type '<GetSafetyStatus-response>"
  "0b50aa56e0379c3a0c319bd4962efe2e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetSafetyStatus-response)))
  "Returns md5sum for a message object of type 'GetSafetyStatus-response"
  "0b50aa56e0379c3a0c319bd4962efe2e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetSafetyStatus-response>)))
  "Returns full string definition for message of type '<GetSafetyStatus-response>"
  (cl:format cl:nil "SafetyStatus output~%~%================================================================================~%MSG: kortex_driver/SafetyStatus~%~%uint32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetSafetyStatus-response)))
  "Returns full string definition for message of type 'GetSafetyStatus-response"
  (cl:format cl:nil "SafetyStatus output~%~%================================================================================~%MSG: kortex_driver/SafetyStatus~%~%uint32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetSafetyStatus-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetSafetyStatus-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetSafetyStatus-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetSafetyStatus)))
  'GetSafetyStatus-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetSafetyStatus)))
  'GetSafetyStatus-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSafetyStatus)))
  "Returns string type for a service object of type '<GetSafetyStatus>"
  "kortex_driver/GetSafetyStatus")