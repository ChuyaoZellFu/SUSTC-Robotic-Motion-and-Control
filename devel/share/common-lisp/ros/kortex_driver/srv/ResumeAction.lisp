; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude ResumeAction-request.msg.html

(cl:defclass <ResumeAction-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass ResumeAction-request (<ResumeAction-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResumeAction-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResumeAction-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ResumeAction-request> is deprecated: use kortex_driver-srv:ResumeAction-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <ResumeAction-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResumeAction-request>) ostream)
  "Serializes a message object of type '<ResumeAction-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResumeAction-request>) istream)
  "Deserializes a message object of type '<ResumeAction-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResumeAction-request>)))
  "Returns string type for a service object of type '<ResumeAction-request>"
  "kortex_driver/ResumeActionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResumeAction-request)))
  "Returns string type for a service object of type 'ResumeAction-request"
  "kortex_driver/ResumeActionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResumeAction-request>)))
  "Returns md5sum for a message object of type '<ResumeAction-request>"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResumeAction-request)))
  "Returns md5sum for a message object of type 'ResumeAction-request"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResumeAction-request>)))
  "Returns full string definition for message of type '<ResumeAction-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResumeAction-request)))
  "Returns full string definition for message of type 'ResumeAction-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResumeAction-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResumeAction-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ResumeAction-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude ResumeAction-response.msg.html

(cl:defclass <ResumeAction-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass ResumeAction-response (<ResumeAction-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResumeAction-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResumeAction-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ResumeAction-response> is deprecated: use kortex_driver-srv:ResumeAction-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <ResumeAction-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResumeAction-response>) ostream)
  "Serializes a message object of type '<ResumeAction-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResumeAction-response>) istream)
  "Deserializes a message object of type '<ResumeAction-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResumeAction-response>)))
  "Returns string type for a service object of type '<ResumeAction-response>"
  "kortex_driver/ResumeActionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResumeAction-response)))
  "Returns string type for a service object of type 'ResumeAction-response"
  "kortex_driver/ResumeActionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResumeAction-response>)))
  "Returns md5sum for a message object of type '<ResumeAction-response>"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResumeAction-response)))
  "Returns md5sum for a message object of type 'ResumeAction-response"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResumeAction-response>)))
  "Returns full string definition for message of type '<ResumeAction-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResumeAction-response)))
  "Returns full string definition for message of type 'ResumeAction-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResumeAction-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResumeAction-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ResumeAction-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ResumeAction)))
  'ResumeAction-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ResumeAction)))
  'ResumeAction-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResumeAction)))
  "Returns string type for a service object of type '<ResumeAction>"
  "kortex_driver/ResumeAction")