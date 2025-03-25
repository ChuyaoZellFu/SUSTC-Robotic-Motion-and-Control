; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetActivatedControlLoop-request.msg.html

(cl:defclass <GetActivatedControlLoop-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetActivatedControlLoop-request (<GetActivatedControlLoop-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetActivatedControlLoop-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetActivatedControlLoop-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetActivatedControlLoop-request> is deprecated: use kortex_driver-srv:GetActivatedControlLoop-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetActivatedControlLoop-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetActivatedControlLoop-request>) ostream)
  "Serializes a message object of type '<GetActivatedControlLoop-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetActivatedControlLoop-request>) istream)
  "Deserializes a message object of type '<GetActivatedControlLoop-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetActivatedControlLoop-request>)))
  "Returns string type for a service object of type '<GetActivatedControlLoop-request>"
  "kortex_driver/GetActivatedControlLoopRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetActivatedControlLoop-request)))
  "Returns string type for a service object of type 'GetActivatedControlLoop-request"
  "kortex_driver/GetActivatedControlLoopRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetActivatedControlLoop-request>)))
  "Returns md5sum for a message object of type '<GetActivatedControlLoop-request>"
  "acf43fee1c712ad0288037798483900b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetActivatedControlLoop-request)))
  "Returns md5sum for a message object of type 'GetActivatedControlLoop-request"
  "acf43fee1c712ad0288037798483900b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetActivatedControlLoop-request>)))
  "Returns full string definition for message of type '<GetActivatedControlLoop-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetActivatedControlLoop-request)))
  "Returns full string definition for message of type 'GetActivatedControlLoop-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetActivatedControlLoop-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetActivatedControlLoop-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetActivatedControlLoop-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetActivatedControlLoop-response.msg.html

(cl:defclass <GetActivatedControlLoop-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:ControlLoop
    :initform (cl:make-instance 'kortex_driver-msg:ControlLoop)))
)

(cl:defclass GetActivatedControlLoop-response (<GetActivatedControlLoop-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetActivatedControlLoop-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetActivatedControlLoop-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetActivatedControlLoop-response> is deprecated: use kortex_driver-srv:GetActivatedControlLoop-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetActivatedControlLoop-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetActivatedControlLoop-response>) ostream)
  "Serializes a message object of type '<GetActivatedControlLoop-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetActivatedControlLoop-response>) istream)
  "Deserializes a message object of type '<GetActivatedControlLoop-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetActivatedControlLoop-response>)))
  "Returns string type for a service object of type '<GetActivatedControlLoop-response>"
  "kortex_driver/GetActivatedControlLoopResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetActivatedControlLoop-response)))
  "Returns string type for a service object of type 'GetActivatedControlLoop-response"
  "kortex_driver/GetActivatedControlLoopResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetActivatedControlLoop-response>)))
  "Returns md5sum for a message object of type '<GetActivatedControlLoop-response>"
  "acf43fee1c712ad0288037798483900b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetActivatedControlLoop-response)))
  "Returns md5sum for a message object of type 'GetActivatedControlLoop-response"
  "acf43fee1c712ad0288037798483900b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetActivatedControlLoop-response>)))
  "Returns full string definition for message of type '<GetActivatedControlLoop-response>"
  (cl:format cl:nil "ControlLoop output~%~%================================================================================~%MSG: kortex_driver/ControlLoop~%~%uint32 control_loop~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetActivatedControlLoop-response)))
  "Returns full string definition for message of type 'GetActivatedControlLoop-response"
  (cl:format cl:nil "ControlLoop output~%~%================================================================================~%MSG: kortex_driver/ControlLoop~%~%uint32 control_loop~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetActivatedControlLoop-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetActivatedControlLoop-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetActivatedControlLoop-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetActivatedControlLoop)))
  'GetActivatedControlLoop-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetActivatedControlLoop)))
  'GetActivatedControlLoop-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetActivatedControlLoop)))
  "Returns string type for a service object of type '<GetActivatedControlLoop>"
  "kortex_driver/GetActivatedControlLoop")