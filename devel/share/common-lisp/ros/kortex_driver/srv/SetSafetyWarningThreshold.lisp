; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetSafetyWarningThreshold-request.msg.html

(cl:defclass <SetSafetyWarningThreshold-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:SafetyThreshold
    :initform (cl:make-instance 'kortex_driver-msg:SafetyThreshold)))
)

(cl:defclass SetSafetyWarningThreshold-request (<SetSafetyWarningThreshold-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetSafetyWarningThreshold-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetSafetyWarningThreshold-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetSafetyWarningThreshold-request> is deprecated: use kortex_driver-srv:SetSafetyWarningThreshold-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetSafetyWarningThreshold-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetSafetyWarningThreshold-request>) ostream)
  "Serializes a message object of type '<SetSafetyWarningThreshold-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetSafetyWarningThreshold-request>) istream)
  "Deserializes a message object of type '<SetSafetyWarningThreshold-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetSafetyWarningThreshold-request>)))
  "Returns string type for a service object of type '<SetSafetyWarningThreshold-request>"
  "kortex_driver/SetSafetyWarningThresholdRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSafetyWarningThreshold-request)))
  "Returns string type for a service object of type 'SetSafetyWarningThreshold-request"
  "kortex_driver/SetSafetyWarningThresholdRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetSafetyWarningThreshold-request>)))
  "Returns md5sum for a message object of type '<SetSafetyWarningThreshold-request>"
  "0d6527f8c0e63583f63fafeb1405804a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetSafetyWarningThreshold-request)))
  "Returns md5sum for a message object of type 'SetSafetyWarningThreshold-request"
  "0d6527f8c0e63583f63fafeb1405804a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetSafetyWarningThreshold-request>)))
  "Returns full string definition for message of type '<SetSafetyWarningThreshold-request>"
  (cl:format cl:nil "SafetyThreshold input~%~%================================================================================~%MSG: kortex_driver/SafetyThreshold~%~%SafetyHandle handle~%float32 value~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetSafetyWarningThreshold-request)))
  "Returns full string definition for message of type 'SetSafetyWarningThreshold-request"
  (cl:format cl:nil "SafetyThreshold input~%~%================================================================================~%MSG: kortex_driver/SafetyThreshold~%~%SafetyHandle handle~%float32 value~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetSafetyWarningThreshold-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetSafetyWarningThreshold-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetSafetyWarningThreshold-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetSafetyWarningThreshold-response.msg.html

(cl:defclass <SetSafetyWarningThreshold-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetSafetyWarningThreshold-response (<SetSafetyWarningThreshold-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetSafetyWarningThreshold-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetSafetyWarningThreshold-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetSafetyWarningThreshold-response> is deprecated: use kortex_driver-srv:SetSafetyWarningThreshold-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetSafetyWarningThreshold-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetSafetyWarningThreshold-response>) ostream)
  "Serializes a message object of type '<SetSafetyWarningThreshold-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetSafetyWarningThreshold-response>) istream)
  "Deserializes a message object of type '<SetSafetyWarningThreshold-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetSafetyWarningThreshold-response>)))
  "Returns string type for a service object of type '<SetSafetyWarningThreshold-response>"
  "kortex_driver/SetSafetyWarningThresholdResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSafetyWarningThreshold-response)))
  "Returns string type for a service object of type 'SetSafetyWarningThreshold-response"
  "kortex_driver/SetSafetyWarningThresholdResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetSafetyWarningThreshold-response>)))
  "Returns md5sum for a message object of type '<SetSafetyWarningThreshold-response>"
  "0d6527f8c0e63583f63fafeb1405804a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetSafetyWarningThreshold-response)))
  "Returns md5sum for a message object of type 'SetSafetyWarningThreshold-response"
  "0d6527f8c0e63583f63fafeb1405804a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetSafetyWarningThreshold-response>)))
  "Returns full string definition for message of type '<SetSafetyWarningThreshold-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetSafetyWarningThreshold-response)))
  "Returns full string definition for message of type 'SetSafetyWarningThreshold-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetSafetyWarningThreshold-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetSafetyWarningThreshold-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetSafetyWarningThreshold-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetSafetyWarningThreshold)))
  'SetSafetyWarningThreshold-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetSafetyWarningThreshold)))
  'SetSafetyWarningThreshold-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSafetyWarningThreshold)))
  "Returns string type for a service object of type '<SetSafetyWarningThreshold>"
  "kortex_driver/SetSafetyWarningThreshold")