; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude StopTeaching-request.msg.html

(cl:defclass <StopTeaching-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass StopTeaching-request (<StopTeaching-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StopTeaching-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StopTeaching-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<StopTeaching-request> is deprecated: use kortex_driver-srv:StopTeaching-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <StopTeaching-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StopTeaching-request>) ostream)
  "Serializes a message object of type '<StopTeaching-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StopTeaching-request>) istream)
  "Deserializes a message object of type '<StopTeaching-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StopTeaching-request>)))
  "Returns string type for a service object of type '<StopTeaching-request>"
  "kortex_driver/StopTeachingRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopTeaching-request)))
  "Returns string type for a service object of type 'StopTeaching-request"
  "kortex_driver/StopTeachingRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StopTeaching-request>)))
  "Returns md5sum for a message object of type '<StopTeaching-request>"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StopTeaching-request)))
  "Returns md5sum for a message object of type 'StopTeaching-request"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StopTeaching-request>)))
  "Returns full string definition for message of type '<StopTeaching-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StopTeaching-request)))
  "Returns full string definition for message of type 'StopTeaching-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StopTeaching-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StopTeaching-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StopTeaching-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude StopTeaching-response.msg.html

(cl:defclass <StopTeaching-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass StopTeaching-response (<StopTeaching-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StopTeaching-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StopTeaching-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<StopTeaching-response> is deprecated: use kortex_driver-srv:StopTeaching-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <StopTeaching-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StopTeaching-response>) ostream)
  "Serializes a message object of type '<StopTeaching-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StopTeaching-response>) istream)
  "Deserializes a message object of type '<StopTeaching-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StopTeaching-response>)))
  "Returns string type for a service object of type '<StopTeaching-response>"
  "kortex_driver/StopTeachingResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopTeaching-response)))
  "Returns string type for a service object of type 'StopTeaching-response"
  "kortex_driver/StopTeachingResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StopTeaching-response>)))
  "Returns md5sum for a message object of type '<StopTeaching-response>"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StopTeaching-response)))
  "Returns md5sum for a message object of type 'StopTeaching-response"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StopTeaching-response>)))
  "Returns full string definition for message of type '<StopTeaching-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StopTeaching-response)))
  "Returns full string definition for message of type 'StopTeaching-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StopTeaching-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StopTeaching-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StopTeaching-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StopTeaching)))
  'StopTeaching-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StopTeaching)))
  'StopTeaching-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopTeaching)))
  "Returns string type for a service object of type '<StopTeaching>"
  "kortex_driver/StopTeaching")