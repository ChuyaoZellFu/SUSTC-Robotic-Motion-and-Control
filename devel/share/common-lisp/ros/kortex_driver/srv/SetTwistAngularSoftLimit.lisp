; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetTwistAngularSoftLimit-request.msg.html

(cl:defclass <SetTwistAngularSoftLimit-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:TwistAngularSoftLimit
    :initform (cl:make-instance 'kortex_driver-msg:TwistAngularSoftLimit)))
)

(cl:defclass SetTwistAngularSoftLimit-request (<SetTwistAngularSoftLimit-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetTwistAngularSoftLimit-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetTwistAngularSoftLimit-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetTwistAngularSoftLimit-request> is deprecated: use kortex_driver-srv:SetTwistAngularSoftLimit-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetTwistAngularSoftLimit-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetTwistAngularSoftLimit-request>) ostream)
  "Serializes a message object of type '<SetTwistAngularSoftLimit-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetTwistAngularSoftLimit-request>) istream)
  "Deserializes a message object of type '<SetTwistAngularSoftLimit-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetTwistAngularSoftLimit-request>)))
  "Returns string type for a service object of type '<SetTwistAngularSoftLimit-request>"
  "kortex_driver/SetTwistAngularSoftLimitRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetTwistAngularSoftLimit-request)))
  "Returns string type for a service object of type 'SetTwistAngularSoftLimit-request"
  "kortex_driver/SetTwistAngularSoftLimitRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetTwistAngularSoftLimit-request>)))
  "Returns md5sum for a message object of type '<SetTwistAngularSoftLimit-request>"
  "3b523deda9069339b963e073a605096e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetTwistAngularSoftLimit-request)))
  "Returns md5sum for a message object of type 'SetTwistAngularSoftLimit-request"
  "3b523deda9069339b963e073a605096e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetTwistAngularSoftLimit-request>)))
  "Returns full string definition for message of type '<SetTwistAngularSoftLimit-request>"
  (cl:format cl:nil "TwistAngularSoftLimit input~%~%================================================================================~%MSG: kortex_driver/TwistAngularSoftLimit~%~%uint32 control_mode~%float32 twist_angular_soft_limit~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetTwistAngularSoftLimit-request)))
  "Returns full string definition for message of type 'SetTwistAngularSoftLimit-request"
  (cl:format cl:nil "TwistAngularSoftLimit input~%~%================================================================================~%MSG: kortex_driver/TwistAngularSoftLimit~%~%uint32 control_mode~%float32 twist_angular_soft_limit~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetTwistAngularSoftLimit-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetTwistAngularSoftLimit-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetTwistAngularSoftLimit-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetTwistAngularSoftLimit-response.msg.html

(cl:defclass <SetTwistAngularSoftLimit-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetTwistAngularSoftLimit-response (<SetTwistAngularSoftLimit-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetTwistAngularSoftLimit-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetTwistAngularSoftLimit-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetTwistAngularSoftLimit-response> is deprecated: use kortex_driver-srv:SetTwistAngularSoftLimit-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetTwistAngularSoftLimit-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetTwistAngularSoftLimit-response>) ostream)
  "Serializes a message object of type '<SetTwistAngularSoftLimit-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetTwistAngularSoftLimit-response>) istream)
  "Deserializes a message object of type '<SetTwistAngularSoftLimit-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetTwistAngularSoftLimit-response>)))
  "Returns string type for a service object of type '<SetTwistAngularSoftLimit-response>"
  "kortex_driver/SetTwistAngularSoftLimitResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetTwistAngularSoftLimit-response)))
  "Returns string type for a service object of type 'SetTwistAngularSoftLimit-response"
  "kortex_driver/SetTwistAngularSoftLimitResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetTwistAngularSoftLimit-response>)))
  "Returns md5sum for a message object of type '<SetTwistAngularSoftLimit-response>"
  "3b523deda9069339b963e073a605096e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetTwistAngularSoftLimit-response)))
  "Returns md5sum for a message object of type 'SetTwistAngularSoftLimit-response"
  "3b523deda9069339b963e073a605096e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetTwistAngularSoftLimit-response>)))
  "Returns full string definition for message of type '<SetTwistAngularSoftLimit-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetTwistAngularSoftLimit-response)))
  "Returns full string definition for message of type 'SetTwistAngularSoftLimit-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetTwistAngularSoftLimit-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetTwistAngularSoftLimit-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetTwistAngularSoftLimit-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetTwistAngularSoftLimit)))
  'SetTwistAngularSoftLimit-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetTwistAngularSoftLimit)))
  'SetTwistAngularSoftLimit-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetTwistAngularSoftLimit)))
  "Returns string type for a service object of type '<SetTwistAngularSoftLimit>"
  "kortex_driver/SetTwistAngularSoftLimit")