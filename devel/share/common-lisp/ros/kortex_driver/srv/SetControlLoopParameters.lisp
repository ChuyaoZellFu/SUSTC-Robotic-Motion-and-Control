; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetControlLoopParameters-request.msg.html

(cl:defclass <SetControlLoopParameters-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:ControlLoopParameters
    :initform (cl:make-instance 'kortex_driver-msg:ControlLoopParameters)))
)

(cl:defclass SetControlLoopParameters-request (<SetControlLoopParameters-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetControlLoopParameters-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetControlLoopParameters-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetControlLoopParameters-request> is deprecated: use kortex_driver-srv:SetControlLoopParameters-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetControlLoopParameters-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetControlLoopParameters-request>) ostream)
  "Serializes a message object of type '<SetControlLoopParameters-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetControlLoopParameters-request>) istream)
  "Deserializes a message object of type '<SetControlLoopParameters-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetControlLoopParameters-request>)))
  "Returns string type for a service object of type '<SetControlLoopParameters-request>"
  "kortex_driver/SetControlLoopParametersRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetControlLoopParameters-request)))
  "Returns string type for a service object of type 'SetControlLoopParameters-request"
  "kortex_driver/SetControlLoopParametersRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetControlLoopParameters-request>)))
  "Returns md5sum for a message object of type '<SetControlLoopParameters-request>"
  "233c2c0defb94e1f2d65e518d2df07b0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetControlLoopParameters-request)))
  "Returns md5sum for a message object of type 'SetControlLoopParameters-request"
  "233c2c0defb94e1f2d65e518d2df07b0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetControlLoopParameters-request>)))
  "Returns full string definition for message of type '<SetControlLoopParameters-request>"
  (cl:format cl:nil "ControlLoopParameters input~%~%================================================================================~%MSG: kortex_driver/ControlLoopParameters~%~%uint32 loop_selection~%float32 error_saturation~%float32 output_saturation~%float32[] kAz~%float32[] kBz~%float32 error_dead_band~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetControlLoopParameters-request)))
  "Returns full string definition for message of type 'SetControlLoopParameters-request"
  (cl:format cl:nil "ControlLoopParameters input~%~%================================================================================~%MSG: kortex_driver/ControlLoopParameters~%~%uint32 loop_selection~%float32 error_saturation~%float32 output_saturation~%float32[] kAz~%float32[] kBz~%float32 error_dead_band~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetControlLoopParameters-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetControlLoopParameters-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetControlLoopParameters-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetControlLoopParameters-response.msg.html

(cl:defclass <SetControlLoopParameters-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetControlLoopParameters-response (<SetControlLoopParameters-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetControlLoopParameters-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetControlLoopParameters-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetControlLoopParameters-response> is deprecated: use kortex_driver-srv:SetControlLoopParameters-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetControlLoopParameters-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetControlLoopParameters-response>) ostream)
  "Serializes a message object of type '<SetControlLoopParameters-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetControlLoopParameters-response>) istream)
  "Deserializes a message object of type '<SetControlLoopParameters-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetControlLoopParameters-response>)))
  "Returns string type for a service object of type '<SetControlLoopParameters-response>"
  "kortex_driver/SetControlLoopParametersResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetControlLoopParameters-response)))
  "Returns string type for a service object of type 'SetControlLoopParameters-response"
  "kortex_driver/SetControlLoopParametersResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetControlLoopParameters-response>)))
  "Returns md5sum for a message object of type '<SetControlLoopParameters-response>"
  "233c2c0defb94e1f2d65e518d2df07b0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetControlLoopParameters-response)))
  "Returns md5sum for a message object of type 'SetControlLoopParameters-response"
  "233c2c0defb94e1f2d65e518d2df07b0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetControlLoopParameters-response>)))
  "Returns full string definition for message of type '<SetControlLoopParameters-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetControlLoopParameters-response)))
  "Returns full string definition for message of type 'SetControlLoopParameters-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetControlLoopParameters-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetControlLoopParameters-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetControlLoopParameters-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetControlLoopParameters)))
  'SetControlLoopParameters-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetControlLoopParameters)))
  'SetControlLoopParameters-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetControlLoopParameters)))
  "Returns string type for a service object of type '<SetControlLoopParameters>"
  "kortex_driver/SetControlLoopParameters")