; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetSafetyEnable-request.msg.html

(cl:defclass <SetSafetyEnable-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:SafetyEnable
    :initform (cl:make-instance 'kortex_driver-msg:SafetyEnable)))
)

(cl:defclass SetSafetyEnable-request (<SetSafetyEnable-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetSafetyEnable-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetSafetyEnable-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetSafetyEnable-request> is deprecated: use kortex_driver-srv:SetSafetyEnable-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetSafetyEnable-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetSafetyEnable-request>) ostream)
  "Serializes a message object of type '<SetSafetyEnable-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetSafetyEnable-request>) istream)
  "Deserializes a message object of type '<SetSafetyEnable-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetSafetyEnable-request>)))
  "Returns string type for a service object of type '<SetSafetyEnable-request>"
  "kortex_driver/SetSafetyEnableRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSafetyEnable-request)))
  "Returns string type for a service object of type 'SetSafetyEnable-request"
  "kortex_driver/SetSafetyEnableRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetSafetyEnable-request>)))
  "Returns md5sum for a message object of type '<SetSafetyEnable-request>"
  "81bf2032f72f340390e3e57ea5d159db")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetSafetyEnable-request)))
  "Returns md5sum for a message object of type 'SetSafetyEnable-request"
  "81bf2032f72f340390e3e57ea5d159db")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetSafetyEnable-request>)))
  "Returns full string definition for message of type '<SetSafetyEnable-request>"
  (cl:format cl:nil "SafetyEnable input~%~%================================================================================~%MSG: kortex_driver/SafetyEnable~%~%SafetyHandle handle~%bool enable~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetSafetyEnable-request)))
  "Returns full string definition for message of type 'SetSafetyEnable-request"
  (cl:format cl:nil "SafetyEnable input~%~%================================================================================~%MSG: kortex_driver/SafetyEnable~%~%SafetyHandle handle~%bool enable~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetSafetyEnable-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetSafetyEnable-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetSafetyEnable-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetSafetyEnable-response.msg.html

(cl:defclass <SetSafetyEnable-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetSafetyEnable-response (<SetSafetyEnable-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetSafetyEnable-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetSafetyEnable-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetSafetyEnable-response> is deprecated: use kortex_driver-srv:SetSafetyEnable-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetSafetyEnable-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetSafetyEnable-response>) ostream)
  "Serializes a message object of type '<SetSafetyEnable-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetSafetyEnable-response>) istream)
  "Deserializes a message object of type '<SetSafetyEnable-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetSafetyEnable-response>)))
  "Returns string type for a service object of type '<SetSafetyEnable-response>"
  "kortex_driver/SetSafetyEnableResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSafetyEnable-response)))
  "Returns string type for a service object of type 'SetSafetyEnable-response"
  "kortex_driver/SetSafetyEnableResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetSafetyEnable-response>)))
  "Returns md5sum for a message object of type '<SetSafetyEnable-response>"
  "81bf2032f72f340390e3e57ea5d159db")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetSafetyEnable-response)))
  "Returns md5sum for a message object of type 'SetSafetyEnable-response"
  "81bf2032f72f340390e3e57ea5d159db")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetSafetyEnable-response>)))
  "Returns full string definition for message of type '<SetSafetyEnable-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetSafetyEnable-response)))
  "Returns full string definition for message of type 'SetSafetyEnable-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetSafetyEnable-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetSafetyEnable-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetSafetyEnable-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetSafetyEnable)))
  'SetSafetyEnable-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetSafetyEnable)))
  'SetSafetyEnable-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSafetyEnable)))
  "Returns string type for a service object of type '<SetSafetyEnable>"
  "kortex_driver/SetSafetyEnable")