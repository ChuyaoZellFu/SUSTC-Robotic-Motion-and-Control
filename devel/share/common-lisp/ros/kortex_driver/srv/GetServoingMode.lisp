; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetServoingMode-request.msg.html

(cl:defclass <GetServoingMode-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetServoingMode-request (<GetServoingMode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetServoingMode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetServoingMode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetServoingMode-request> is deprecated: use kortex_driver-srv:GetServoingMode-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetServoingMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetServoingMode-request>) ostream)
  "Serializes a message object of type '<GetServoingMode-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetServoingMode-request>) istream)
  "Deserializes a message object of type '<GetServoingMode-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetServoingMode-request>)))
  "Returns string type for a service object of type '<GetServoingMode-request>"
  "kortex_driver/GetServoingModeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetServoingMode-request)))
  "Returns string type for a service object of type 'GetServoingMode-request"
  "kortex_driver/GetServoingModeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetServoingMode-request>)))
  "Returns md5sum for a message object of type '<GetServoingMode-request>"
  "530753c2004b520e20de588dc61fb225")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetServoingMode-request)))
  "Returns md5sum for a message object of type 'GetServoingMode-request"
  "530753c2004b520e20de588dc61fb225")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetServoingMode-request>)))
  "Returns full string definition for message of type '<GetServoingMode-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetServoingMode-request)))
  "Returns full string definition for message of type 'GetServoingMode-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetServoingMode-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetServoingMode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetServoingMode-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetServoingMode-response.msg.html

(cl:defclass <GetServoingMode-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:ServoingModeInformation
    :initform (cl:make-instance 'kortex_driver-msg:ServoingModeInformation)))
)

(cl:defclass GetServoingMode-response (<GetServoingMode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetServoingMode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetServoingMode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetServoingMode-response> is deprecated: use kortex_driver-srv:GetServoingMode-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetServoingMode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetServoingMode-response>) ostream)
  "Serializes a message object of type '<GetServoingMode-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetServoingMode-response>) istream)
  "Deserializes a message object of type '<GetServoingMode-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetServoingMode-response>)))
  "Returns string type for a service object of type '<GetServoingMode-response>"
  "kortex_driver/GetServoingModeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetServoingMode-response)))
  "Returns string type for a service object of type 'GetServoingMode-response"
  "kortex_driver/GetServoingModeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetServoingMode-response>)))
  "Returns md5sum for a message object of type '<GetServoingMode-response>"
  "530753c2004b520e20de588dc61fb225")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetServoingMode-response)))
  "Returns md5sum for a message object of type 'GetServoingMode-response"
  "530753c2004b520e20de588dc61fb225")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetServoingMode-response>)))
  "Returns full string definition for message of type '<GetServoingMode-response>"
  (cl:format cl:nil "ServoingModeInformation output~%~%================================================================================~%MSG: kortex_driver/ServoingModeInformation~%~%uint32 servoing_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetServoingMode-response)))
  "Returns full string definition for message of type 'GetServoingMode-response"
  (cl:format cl:nil "ServoingModeInformation output~%~%================================================================================~%MSG: kortex_driver/ServoingModeInformation~%~%uint32 servoing_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetServoingMode-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetServoingMode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetServoingMode-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetServoingMode)))
  'GetServoingMode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetServoingMode)))
  'GetServoingMode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetServoingMode)))
  "Returns string type for a service object of type '<GetServoingMode>"
  "kortex_driver/GetServoingMode")