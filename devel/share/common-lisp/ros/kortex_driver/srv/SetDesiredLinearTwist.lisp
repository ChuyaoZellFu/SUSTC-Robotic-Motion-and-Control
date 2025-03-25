; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetDesiredLinearTwist-request.msg.html

(cl:defclass <SetDesiredLinearTwist-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:LinearTwist
    :initform (cl:make-instance 'kortex_driver-msg:LinearTwist)))
)

(cl:defclass SetDesiredLinearTwist-request (<SetDesiredLinearTwist-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetDesiredLinearTwist-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetDesiredLinearTwist-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetDesiredLinearTwist-request> is deprecated: use kortex_driver-srv:SetDesiredLinearTwist-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetDesiredLinearTwist-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetDesiredLinearTwist-request>) ostream)
  "Serializes a message object of type '<SetDesiredLinearTwist-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetDesiredLinearTwist-request>) istream)
  "Deserializes a message object of type '<SetDesiredLinearTwist-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetDesiredLinearTwist-request>)))
  "Returns string type for a service object of type '<SetDesiredLinearTwist-request>"
  "kortex_driver/SetDesiredLinearTwistRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetDesiredLinearTwist-request)))
  "Returns string type for a service object of type 'SetDesiredLinearTwist-request"
  "kortex_driver/SetDesiredLinearTwistRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetDesiredLinearTwist-request>)))
  "Returns md5sum for a message object of type '<SetDesiredLinearTwist-request>"
  "2fd07256b791580cac358c271b7f0f2a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetDesiredLinearTwist-request)))
  "Returns md5sum for a message object of type 'SetDesiredLinearTwist-request"
  "2fd07256b791580cac358c271b7f0f2a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetDesiredLinearTwist-request>)))
  "Returns full string definition for message of type '<SetDesiredLinearTwist-request>"
  (cl:format cl:nil "LinearTwist input~%~%================================================================================~%MSG: kortex_driver/LinearTwist~%~%float32 linear~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetDesiredLinearTwist-request)))
  "Returns full string definition for message of type 'SetDesiredLinearTwist-request"
  (cl:format cl:nil "LinearTwist input~%~%================================================================================~%MSG: kortex_driver/LinearTwist~%~%float32 linear~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetDesiredLinearTwist-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetDesiredLinearTwist-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetDesiredLinearTwist-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetDesiredLinearTwist-response.msg.html

(cl:defclass <SetDesiredLinearTwist-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetDesiredLinearTwist-response (<SetDesiredLinearTwist-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetDesiredLinearTwist-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetDesiredLinearTwist-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetDesiredLinearTwist-response> is deprecated: use kortex_driver-srv:SetDesiredLinearTwist-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetDesiredLinearTwist-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetDesiredLinearTwist-response>) ostream)
  "Serializes a message object of type '<SetDesiredLinearTwist-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetDesiredLinearTwist-response>) istream)
  "Deserializes a message object of type '<SetDesiredLinearTwist-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetDesiredLinearTwist-response>)))
  "Returns string type for a service object of type '<SetDesiredLinearTwist-response>"
  "kortex_driver/SetDesiredLinearTwistResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetDesiredLinearTwist-response)))
  "Returns string type for a service object of type 'SetDesiredLinearTwist-response"
  "kortex_driver/SetDesiredLinearTwistResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetDesiredLinearTwist-response>)))
  "Returns md5sum for a message object of type '<SetDesiredLinearTwist-response>"
  "2fd07256b791580cac358c271b7f0f2a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetDesiredLinearTwist-response)))
  "Returns md5sum for a message object of type 'SetDesiredLinearTwist-response"
  "2fd07256b791580cac358c271b7f0f2a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetDesiredLinearTwist-response>)))
  "Returns full string definition for message of type '<SetDesiredLinearTwist-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetDesiredLinearTwist-response)))
  "Returns full string definition for message of type 'SetDesiredLinearTwist-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetDesiredLinearTwist-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetDesiredLinearTwist-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetDesiredLinearTwist-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetDesiredLinearTwist)))
  'SetDesiredLinearTwist-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetDesiredLinearTwist)))
  'SetDesiredLinearTwist-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetDesiredLinearTwist)))
  "Returns string type for a service object of type '<SetDesiredLinearTwist>"
  "kortex_driver/SetDesiredLinearTwist")