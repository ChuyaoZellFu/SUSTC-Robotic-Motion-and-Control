; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetSafetyErrorThreshold-request.msg.html

(cl:defclass <SetSafetyErrorThreshold-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:SafetyThreshold
    :initform (cl:make-instance 'kortex_driver-msg:SafetyThreshold)))
)

(cl:defclass SetSafetyErrorThreshold-request (<SetSafetyErrorThreshold-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetSafetyErrorThreshold-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetSafetyErrorThreshold-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetSafetyErrorThreshold-request> is deprecated: use kortex_driver-srv:SetSafetyErrorThreshold-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetSafetyErrorThreshold-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetSafetyErrorThreshold-request>) ostream)
  "Serializes a message object of type '<SetSafetyErrorThreshold-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetSafetyErrorThreshold-request>) istream)
  "Deserializes a message object of type '<SetSafetyErrorThreshold-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetSafetyErrorThreshold-request>)))
  "Returns string type for a service object of type '<SetSafetyErrorThreshold-request>"
  "kortex_driver/SetSafetyErrorThresholdRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSafetyErrorThreshold-request)))
  "Returns string type for a service object of type 'SetSafetyErrorThreshold-request"
  "kortex_driver/SetSafetyErrorThresholdRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetSafetyErrorThreshold-request>)))
  "Returns md5sum for a message object of type '<SetSafetyErrorThreshold-request>"
  "0d6527f8c0e63583f63fafeb1405804a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetSafetyErrorThreshold-request)))
  "Returns md5sum for a message object of type 'SetSafetyErrorThreshold-request"
  "0d6527f8c0e63583f63fafeb1405804a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetSafetyErrorThreshold-request>)))
  "Returns full string definition for message of type '<SetSafetyErrorThreshold-request>"
  (cl:format cl:nil "SafetyThreshold input~%~%================================================================================~%MSG: kortex_driver/SafetyThreshold~%~%SafetyHandle handle~%float32 value~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetSafetyErrorThreshold-request)))
  "Returns full string definition for message of type 'SetSafetyErrorThreshold-request"
  (cl:format cl:nil "SafetyThreshold input~%~%================================================================================~%MSG: kortex_driver/SafetyThreshold~%~%SafetyHandle handle~%float32 value~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetSafetyErrorThreshold-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetSafetyErrorThreshold-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetSafetyErrorThreshold-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetSafetyErrorThreshold-response.msg.html

(cl:defclass <SetSafetyErrorThreshold-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetSafetyErrorThreshold-response (<SetSafetyErrorThreshold-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetSafetyErrorThreshold-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetSafetyErrorThreshold-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetSafetyErrorThreshold-response> is deprecated: use kortex_driver-srv:SetSafetyErrorThreshold-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetSafetyErrorThreshold-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetSafetyErrorThreshold-response>) ostream)
  "Serializes a message object of type '<SetSafetyErrorThreshold-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetSafetyErrorThreshold-response>) istream)
  "Deserializes a message object of type '<SetSafetyErrorThreshold-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetSafetyErrorThreshold-response>)))
  "Returns string type for a service object of type '<SetSafetyErrorThreshold-response>"
  "kortex_driver/SetSafetyErrorThresholdResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSafetyErrorThreshold-response)))
  "Returns string type for a service object of type 'SetSafetyErrorThreshold-response"
  "kortex_driver/SetSafetyErrorThresholdResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetSafetyErrorThreshold-response>)))
  "Returns md5sum for a message object of type '<SetSafetyErrorThreshold-response>"
  "0d6527f8c0e63583f63fafeb1405804a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetSafetyErrorThreshold-response)))
  "Returns md5sum for a message object of type 'SetSafetyErrorThreshold-response"
  "0d6527f8c0e63583f63fafeb1405804a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetSafetyErrorThreshold-response>)))
  "Returns full string definition for message of type '<SetSafetyErrorThreshold-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetSafetyErrorThreshold-response)))
  "Returns full string definition for message of type 'SetSafetyErrorThreshold-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetSafetyErrorThreshold-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetSafetyErrorThreshold-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetSafetyErrorThreshold-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetSafetyErrorThreshold)))
  'SetSafetyErrorThreshold-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetSafetyErrorThreshold)))
  'SetSafetyErrorThreshold-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSafetyErrorThreshold)))
  "Returns string type for a service object of type '<SetSafetyErrorThreshold>"
  "kortex_driver/SetSafetyErrorThreshold")