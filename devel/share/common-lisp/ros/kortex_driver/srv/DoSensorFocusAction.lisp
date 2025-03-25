; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude DoSensorFocusAction-request.msg.html

(cl:defclass <DoSensorFocusAction-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:SensorFocusAction
    :initform (cl:make-instance 'kortex_driver-msg:SensorFocusAction)))
)

(cl:defclass DoSensorFocusAction-request (<DoSensorFocusAction-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DoSensorFocusAction-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DoSensorFocusAction-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<DoSensorFocusAction-request> is deprecated: use kortex_driver-srv:DoSensorFocusAction-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <DoSensorFocusAction-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DoSensorFocusAction-request>) ostream)
  "Serializes a message object of type '<DoSensorFocusAction-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DoSensorFocusAction-request>) istream)
  "Deserializes a message object of type '<DoSensorFocusAction-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DoSensorFocusAction-request>)))
  "Returns string type for a service object of type '<DoSensorFocusAction-request>"
  "kortex_driver/DoSensorFocusActionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DoSensorFocusAction-request)))
  "Returns string type for a service object of type 'DoSensorFocusAction-request"
  "kortex_driver/DoSensorFocusActionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DoSensorFocusAction-request>)))
  "Returns md5sum for a message object of type '<DoSensorFocusAction-request>"
  "a727a16574ce67bd4de8fba7da857f9f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DoSensorFocusAction-request)))
  "Returns md5sum for a message object of type 'DoSensorFocusAction-request"
  "a727a16574ce67bd4de8fba7da857f9f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DoSensorFocusAction-request>)))
  "Returns full string definition for message of type '<DoSensorFocusAction-request>"
  (cl:format cl:nil "SensorFocusAction input~%~%================================================================================~%MSG: kortex_driver/SensorFocusAction~%~%uint32 sensor~%uint32 focus_action~%SensorFocusAction_action_parameters oneof_action_parameters~%================================================================================~%MSG: kortex_driver/SensorFocusAction_action_parameters~%~%FocusPoint[] focus_point~%ManualFocus[] manual_focus~%================================================================================~%MSG: kortex_driver/FocusPoint~%~%uint32 x~%uint32 y~%================================================================================~%MSG: kortex_driver/ManualFocus~%~%uint32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DoSensorFocusAction-request)))
  "Returns full string definition for message of type 'DoSensorFocusAction-request"
  (cl:format cl:nil "SensorFocusAction input~%~%================================================================================~%MSG: kortex_driver/SensorFocusAction~%~%uint32 sensor~%uint32 focus_action~%SensorFocusAction_action_parameters oneof_action_parameters~%================================================================================~%MSG: kortex_driver/SensorFocusAction_action_parameters~%~%FocusPoint[] focus_point~%ManualFocus[] manual_focus~%================================================================================~%MSG: kortex_driver/FocusPoint~%~%uint32 x~%uint32 y~%================================================================================~%MSG: kortex_driver/ManualFocus~%~%uint32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DoSensorFocusAction-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DoSensorFocusAction-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DoSensorFocusAction-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude DoSensorFocusAction-response.msg.html

(cl:defclass <DoSensorFocusAction-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass DoSensorFocusAction-response (<DoSensorFocusAction-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DoSensorFocusAction-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DoSensorFocusAction-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<DoSensorFocusAction-response> is deprecated: use kortex_driver-srv:DoSensorFocusAction-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <DoSensorFocusAction-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DoSensorFocusAction-response>) ostream)
  "Serializes a message object of type '<DoSensorFocusAction-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DoSensorFocusAction-response>) istream)
  "Deserializes a message object of type '<DoSensorFocusAction-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DoSensorFocusAction-response>)))
  "Returns string type for a service object of type '<DoSensorFocusAction-response>"
  "kortex_driver/DoSensorFocusActionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DoSensorFocusAction-response)))
  "Returns string type for a service object of type 'DoSensorFocusAction-response"
  "kortex_driver/DoSensorFocusActionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DoSensorFocusAction-response>)))
  "Returns md5sum for a message object of type '<DoSensorFocusAction-response>"
  "a727a16574ce67bd4de8fba7da857f9f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DoSensorFocusAction-response)))
  "Returns md5sum for a message object of type 'DoSensorFocusAction-response"
  "a727a16574ce67bd4de8fba7da857f9f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DoSensorFocusAction-response>)))
  "Returns full string definition for message of type '<DoSensorFocusAction-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DoSensorFocusAction-response)))
  "Returns full string definition for message of type 'DoSensorFocusAction-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DoSensorFocusAction-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DoSensorFocusAction-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DoSensorFocusAction-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DoSensorFocusAction)))
  'DoSensorFocusAction-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DoSensorFocusAction)))
  'DoSensorFocusAction-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DoSensorFocusAction)))
  "Returns string type for a service object of type '<DoSensorFocusAction>"
  "kortex_driver/DoSensorFocusAction")