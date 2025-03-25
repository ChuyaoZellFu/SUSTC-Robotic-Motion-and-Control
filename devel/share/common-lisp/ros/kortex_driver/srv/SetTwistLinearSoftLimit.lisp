; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetTwistLinearSoftLimit-request.msg.html

(cl:defclass <SetTwistLinearSoftLimit-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:TwistLinearSoftLimit
    :initform (cl:make-instance 'kortex_driver-msg:TwistLinearSoftLimit)))
)

(cl:defclass SetTwistLinearSoftLimit-request (<SetTwistLinearSoftLimit-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetTwistLinearSoftLimit-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetTwistLinearSoftLimit-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetTwistLinearSoftLimit-request> is deprecated: use kortex_driver-srv:SetTwistLinearSoftLimit-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetTwistLinearSoftLimit-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetTwistLinearSoftLimit-request>) ostream)
  "Serializes a message object of type '<SetTwistLinearSoftLimit-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetTwistLinearSoftLimit-request>) istream)
  "Deserializes a message object of type '<SetTwistLinearSoftLimit-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetTwistLinearSoftLimit-request>)))
  "Returns string type for a service object of type '<SetTwistLinearSoftLimit-request>"
  "kortex_driver/SetTwistLinearSoftLimitRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetTwistLinearSoftLimit-request)))
  "Returns string type for a service object of type 'SetTwistLinearSoftLimit-request"
  "kortex_driver/SetTwistLinearSoftLimitRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetTwistLinearSoftLimit-request>)))
  "Returns md5sum for a message object of type '<SetTwistLinearSoftLimit-request>"
  "1641ba0fe5229f88d9cda0ee47099b0f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetTwistLinearSoftLimit-request)))
  "Returns md5sum for a message object of type 'SetTwistLinearSoftLimit-request"
  "1641ba0fe5229f88d9cda0ee47099b0f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetTwistLinearSoftLimit-request>)))
  "Returns full string definition for message of type '<SetTwistLinearSoftLimit-request>"
  (cl:format cl:nil "TwistLinearSoftLimit input~%~%================================================================================~%MSG: kortex_driver/TwistLinearSoftLimit~%~%uint32 control_mode~%float32 twist_linear_soft_limit~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetTwistLinearSoftLimit-request)))
  "Returns full string definition for message of type 'SetTwistLinearSoftLimit-request"
  (cl:format cl:nil "TwistLinearSoftLimit input~%~%================================================================================~%MSG: kortex_driver/TwistLinearSoftLimit~%~%uint32 control_mode~%float32 twist_linear_soft_limit~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetTwistLinearSoftLimit-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetTwistLinearSoftLimit-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetTwistLinearSoftLimit-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetTwistLinearSoftLimit-response.msg.html

(cl:defclass <SetTwistLinearSoftLimit-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetTwistLinearSoftLimit-response (<SetTwistLinearSoftLimit-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetTwistLinearSoftLimit-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetTwistLinearSoftLimit-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetTwistLinearSoftLimit-response> is deprecated: use kortex_driver-srv:SetTwistLinearSoftLimit-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetTwistLinearSoftLimit-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetTwistLinearSoftLimit-response>) ostream)
  "Serializes a message object of type '<SetTwistLinearSoftLimit-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetTwistLinearSoftLimit-response>) istream)
  "Deserializes a message object of type '<SetTwistLinearSoftLimit-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetTwistLinearSoftLimit-response>)))
  "Returns string type for a service object of type '<SetTwistLinearSoftLimit-response>"
  "kortex_driver/SetTwistLinearSoftLimitResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetTwistLinearSoftLimit-response)))
  "Returns string type for a service object of type 'SetTwistLinearSoftLimit-response"
  "kortex_driver/SetTwistLinearSoftLimitResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetTwistLinearSoftLimit-response>)))
  "Returns md5sum for a message object of type '<SetTwistLinearSoftLimit-response>"
  "1641ba0fe5229f88d9cda0ee47099b0f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetTwistLinearSoftLimit-response)))
  "Returns md5sum for a message object of type 'SetTwistLinearSoftLimit-response"
  "1641ba0fe5229f88d9cda0ee47099b0f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetTwistLinearSoftLimit-response>)))
  "Returns full string definition for message of type '<SetTwistLinearSoftLimit-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetTwistLinearSoftLimit-response)))
  "Returns full string definition for message of type 'SetTwistLinearSoftLimit-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetTwistLinearSoftLimit-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetTwistLinearSoftLimit-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetTwistLinearSoftLimit-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetTwistLinearSoftLimit)))
  'SetTwistLinearSoftLimit-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetTwistLinearSoftLimit)))
  'SetTwistLinearSoftLimit-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetTwistLinearSoftLimit)))
  "Returns string type for a service object of type '<SetTwistLinearSoftLimit>"
  "kortex_driver/SetTwistLinearSoftLimit")