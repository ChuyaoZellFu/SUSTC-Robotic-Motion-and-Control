; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetRunMode-request.msg.html

(cl:defclass <SetRunMode-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:RunMode
    :initform (cl:make-instance 'kortex_driver-msg:RunMode)))
)

(cl:defclass SetRunMode-request (<SetRunMode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetRunMode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetRunMode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetRunMode-request> is deprecated: use kortex_driver-srv:SetRunMode-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetRunMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetRunMode-request>) ostream)
  "Serializes a message object of type '<SetRunMode-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetRunMode-request>) istream)
  "Deserializes a message object of type '<SetRunMode-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetRunMode-request>)))
  "Returns string type for a service object of type '<SetRunMode-request>"
  "kortex_driver/SetRunModeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRunMode-request)))
  "Returns string type for a service object of type 'SetRunMode-request"
  "kortex_driver/SetRunModeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetRunMode-request>)))
  "Returns md5sum for a message object of type '<SetRunMode-request>"
  "9b8a67627c37962f119f97d74affc2cb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetRunMode-request)))
  "Returns md5sum for a message object of type 'SetRunMode-request"
  "9b8a67627c37962f119f97d74affc2cb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetRunMode-request>)))
  "Returns full string definition for message of type '<SetRunMode-request>"
  (cl:format cl:nil "RunMode input~%~%================================================================================~%MSG: kortex_driver/RunMode~%~%uint32 run_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetRunMode-request)))
  "Returns full string definition for message of type 'SetRunMode-request"
  (cl:format cl:nil "RunMode input~%~%================================================================================~%MSG: kortex_driver/RunMode~%~%uint32 run_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetRunMode-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetRunMode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetRunMode-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetRunMode-response.msg.html

(cl:defclass <SetRunMode-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetRunMode-response (<SetRunMode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetRunMode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetRunMode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetRunMode-response> is deprecated: use kortex_driver-srv:SetRunMode-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetRunMode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetRunMode-response>) ostream)
  "Serializes a message object of type '<SetRunMode-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetRunMode-response>) istream)
  "Deserializes a message object of type '<SetRunMode-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetRunMode-response>)))
  "Returns string type for a service object of type '<SetRunMode-response>"
  "kortex_driver/SetRunModeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRunMode-response)))
  "Returns string type for a service object of type 'SetRunMode-response"
  "kortex_driver/SetRunModeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetRunMode-response>)))
  "Returns md5sum for a message object of type '<SetRunMode-response>"
  "9b8a67627c37962f119f97d74affc2cb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetRunMode-response)))
  "Returns md5sum for a message object of type 'SetRunMode-response"
  "9b8a67627c37962f119f97d74affc2cb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetRunMode-response>)))
  "Returns full string definition for message of type '<SetRunMode-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetRunMode-response)))
  "Returns full string definition for message of type 'SetRunMode-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetRunMode-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetRunMode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetRunMode-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetRunMode)))
  'SetRunMode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetRunMode)))
  'SetRunMode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRunMode)))
  "Returns string type for a service object of type '<SetRunMode>"
  "kortex_driver/SetRunMode")