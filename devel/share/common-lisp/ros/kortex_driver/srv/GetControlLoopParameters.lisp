; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetControlLoopParameters-request.msg.html

(cl:defclass <GetControlLoopParameters-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:LoopSelection
    :initform (cl:make-instance 'kortex_driver-msg:LoopSelection)))
)

(cl:defclass GetControlLoopParameters-request (<GetControlLoopParameters-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetControlLoopParameters-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetControlLoopParameters-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetControlLoopParameters-request> is deprecated: use kortex_driver-srv:GetControlLoopParameters-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetControlLoopParameters-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetControlLoopParameters-request>) ostream)
  "Serializes a message object of type '<GetControlLoopParameters-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetControlLoopParameters-request>) istream)
  "Deserializes a message object of type '<GetControlLoopParameters-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetControlLoopParameters-request>)))
  "Returns string type for a service object of type '<GetControlLoopParameters-request>"
  "kortex_driver/GetControlLoopParametersRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetControlLoopParameters-request)))
  "Returns string type for a service object of type 'GetControlLoopParameters-request"
  "kortex_driver/GetControlLoopParametersRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetControlLoopParameters-request>)))
  "Returns md5sum for a message object of type '<GetControlLoopParameters-request>"
  "fe660929cab3528eb46e3ce651e5ce58")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetControlLoopParameters-request)))
  "Returns md5sum for a message object of type 'GetControlLoopParameters-request"
  "fe660929cab3528eb46e3ce651e5ce58")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetControlLoopParameters-request>)))
  "Returns full string definition for message of type '<GetControlLoopParameters-request>"
  (cl:format cl:nil "LoopSelection input~%~%================================================================================~%MSG: kortex_driver/LoopSelection~%~%uint32 loop_selection~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetControlLoopParameters-request)))
  "Returns full string definition for message of type 'GetControlLoopParameters-request"
  (cl:format cl:nil "LoopSelection input~%~%================================================================================~%MSG: kortex_driver/LoopSelection~%~%uint32 loop_selection~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetControlLoopParameters-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetControlLoopParameters-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetControlLoopParameters-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetControlLoopParameters-response.msg.html

(cl:defclass <GetControlLoopParameters-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:ControlLoopParameters
    :initform (cl:make-instance 'kortex_driver-msg:ControlLoopParameters)))
)

(cl:defclass GetControlLoopParameters-response (<GetControlLoopParameters-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetControlLoopParameters-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetControlLoopParameters-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetControlLoopParameters-response> is deprecated: use kortex_driver-srv:GetControlLoopParameters-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetControlLoopParameters-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetControlLoopParameters-response>) ostream)
  "Serializes a message object of type '<GetControlLoopParameters-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetControlLoopParameters-response>) istream)
  "Deserializes a message object of type '<GetControlLoopParameters-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetControlLoopParameters-response>)))
  "Returns string type for a service object of type '<GetControlLoopParameters-response>"
  "kortex_driver/GetControlLoopParametersResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetControlLoopParameters-response)))
  "Returns string type for a service object of type 'GetControlLoopParameters-response"
  "kortex_driver/GetControlLoopParametersResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetControlLoopParameters-response>)))
  "Returns md5sum for a message object of type '<GetControlLoopParameters-response>"
  "fe660929cab3528eb46e3ce651e5ce58")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetControlLoopParameters-response)))
  "Returns md5sum for a message object of type 'GetControlLoopParameters-response"
  "fe660929cab3528eb46e3ce651e5ce58")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetControlLoopParameters-response>)))
  "Returns full string definition for message of type '<GetControlLoopParameters-response>"
  (cl:format cl:nil "ControlLoopParameters output~%~%================================================================================~%MSG: kortex_driver/ControlLoopParameters~%~%uint32 loop_selection~%float32 error_saturation~%float32 output_saturation~%float32[] kAz~%float32[] kBz~%float32 error_dead_band~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetControlLoopParameters-response)))
  "Returns full string definition for message of type 'GetControlLoopParameters-response"
  (cl:format cl:nil "ControlLoopParameters output~%~%================================================================================~%MSG: kortex_driver/ControlLoopParameters~%~%uint32 loop_selection~%float32 error_saturation~%float32 output_saturation~%float32[] kAz~%float32[] kBz~%float32 error_dead_band~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetControlLoopParameters-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetControlLoopParameters-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetControlLoopParameters-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetControlLoopParameters)))
  'GetControlLoopParameters-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetControlLoopParameters)))
  'GetControlLoopParameters-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetControlLoopParameters)))
  "Returns string type for a service object of type '<GetControlLoopParameters>"
  "kortex_driver/GetControlLoopParameters")