; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetActivatedControlLoop-request.msg.html

(cl:defclass <SetActivatedControlLoop-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:ControlLoop
    :initform (cl:make-instance 'kortex_driver-msg:ControlLoop)))
)

(cl:defclass SetActivatedControlLoop-request (<SetActivatedControlLoop-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetActivatedControlLoop-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetActivatedControlLoop-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetActivatedControlLoop-request> is deprecated: use kortex_driver-srv:SetActivatedControlLoop-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetActivatedControlLoop-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetActivatedControlLoop-request>) ostream)
  "Serializes a message object of type '<SetActivatedControlLoop-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetActivatedControlLoop-request>) istream)
  "Deserializes a message object of type '<SetActivatedControlLoop-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetActivatedControlLoop-request>)))
  "Returns string type for a service object of type '<SetActivatedControlLoop-request>"
  "kortex_driver/SetActivatedControlLoopRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetActivatedControlLoop-request)))
  "Returns string type for a service object of type 'SetActivatedControlLoop-request"
  "kortex_driver/SetActivatedControlLoopRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetActivatedControlLoop-request>)))
  "Returns md5sum for a message object of type '<SetActivatedControlLoop-request>"
  "0f6f5ae628430443bb031e43516b6adc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetActivatedControlLoop-request)))
  "Returns md5sum for a message object of type 'SetActivatedControlLoop-request"
  "0f6f5ae628430443bb031e43516b6adc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetActivatedControlLoop-request>)))
  "Returns full string definition for message of type '<SetActivatedControlLoop-request>"
  (cl:format cl:nil "ControlLoop input~%~%================================================================================~%MSG: kortex_driver/ControlLoop~%~%uint32 control_loop~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetActivatedControlLoop-request)))
  "Returns full string definition for message of type 'SetActivatedControlLoop-request"
  (cl:format cl:nil "ControlLoop input~%~%================================================================================~%MSG: kortex_driver/ControlLoop~%~%uint32 control_loop~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetActivatedControlLoop-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetActivatedControlLoop-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetActivatedControlLoop-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetActivatedControlLoop-response.msg.html

(cl:defclass <SetActivatedControlLoop-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetActivatedControlLoop-response (<SetActivatedControlLoop-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetActivatedControlLoop-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetActivatedControlLoop-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetActivatedControlLoop-response> is deprecated: use kortex_driver-srv:SetActivatedControlLoop-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetActivatedControlLoop-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetActivatedControlLoop-response>) ostream)
  "Serializes a message object of type '<SetActivatedControlLoop-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetActivatedControlLoop-response>) istream)
  "Deserializes a message object of type '<SetActivatedControlLoop-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetActivatedControlLoop-response>)))
  "Returns string type for a service object of type '<SetActivatedControlLoop-response>"
  "kortex_driver/SetActivatedControlLoopResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetActivatedControlLoop-response)))
  "Returns string type for a service object of type 'SetActivatedControlLoop-response"
  "kortex_driver/SetActivatedControlLoopResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetActivatedControlLoop-response>)))
  "Returns md5sum for a message object of type '<SetActivatedControlLoop-response>"
  "0f6f5ae628430443bb031e43516b6adc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetActivatedControlLoop-response)))
  "Returns md5sum for a message object of type 'SetActivatedControlLoop-response"
  "0f6f5ae628430443bb031e43516b6adc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetActivatedControlLoop-response>)))
  "Returns full string definition for message of type '<SetActivatedControlLoop-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetActivatedControlLoop-response)))
  "Returns full string definition for message of type 'SetActivatedControlLoop-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetActivatedControlLoop-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetActivatedControlLoop-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetActivatedControlLoop-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetActivatedControlLoop)))
  'SetActivatedControlLoop-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetActivatedControlLoop)))
  'SetActivatedControlLoop-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetActivatedControlLoop)))
  "Returns string type for a service object of type '<SetActivatedControlLoop>"
  "kortex_driver/SetActivatedControlLoop")