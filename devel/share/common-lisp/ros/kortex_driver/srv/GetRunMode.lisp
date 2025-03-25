; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetRunMode-request.msg.html

(cl:defclass <GetRunMode-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetRunMode-request (<GetRunMode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetRunMode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetRunMode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetRunMode-request> is deprecated: use kortex_driver-srv:GetRunMode-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetRunMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetRunMode-request>) ostream)
  "Serializes a message object of type '<GetRunMode-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetRunMode-request>) istream)
  "Deserializes a message object of type '<GetRunMode-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetRunMode-request>)))
  "Returns string type for a service object of type '<GetRunMode-request>"
  "kortex_driver/GetRunModeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRunMode-request)))
  "Returns string type for a service object of type 'GetRunMode-request"
  "kortex_driver/GetRunModeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetRunMode-request>)))
  "Returns md5sum for a message object of type '<GetRunMode-request>"
  "dd5c8711b1fd2894a1cd52ad67ed6ff2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetRunMode-request)))
  "Returns md5sum for a message object of type 'GetRunMode-request"
  "dd5c8711b1fd2894a1cd52ad67ed6ff2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetRunMode-request>)))
  "Returns full string definition for message of type '<GetRunMode-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetRunMode-request)))
  "Returns full string definition for message of type 'GetRunMode-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetRunMode-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetRunMode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetRunMode-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetRunMode-response.msg.html

(cl:defclass <GetRunMode-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:RunMode
    :initform (cl:make-instance 'kortex_driver-msg:RunMode)))
)

(cl:defclass GetRunMode-response (<GetRunMode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetRunMode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetRunMode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetRunMode-response> is deprecated: use kortex_driver-srv:GetRunMode-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetRunMode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetRunMode-response>) ostream)
  "Serializes a message object of type '<GetRunMode-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetRunMode-response>) istream)
  "Deserializes a message object of type '<GetRunMode-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetRunMode-response>)))
  "Returns string type for a service object of type '<GetRunMode-response>"
  "kortex_driver/GetRunModeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRunMode-response)))
  "Returns string type for a service object of type 'GetRunMode-response"
  "kortex_driver/GetRunModeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetRunMode-response>)))
  "Returns md5sum for a message object of type '<GetRunMode-response>"
  "dd5c8711b1fd2894a1cd52ad67ed6ff2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetRunMode-response)))
  "Returns md5sum for a message object of type 'GetRunMode-response"
  "dd5c8711b1fd2894a1cd52ad67ed6ff2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetRunMode-response>)))
  "Returns full string definition for message of type '<GetRunMode-response>"
  (cl:format cl:nil "RunMode output~%~%================================================================================~%MSG: kortex_driver/RunMode~%~%uint32 run_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetRunMode-response)))
  "Returns full string definition for message of type 'GetRunMode-response"
  (cl:format cl:nil "RunMode output~%~%================================================================================~%MSG: kortex_driver/RunMode~%~%uint32 run_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetRunMode-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetRunMode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetRunMode-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetRunMode)))
  'GetRunMode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetRunMode)))
  'GetRunMode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRunMode)))
  "Returns string type for a service object of type '<GetRunMode>"
  "kortex_driver/GetRunMode")