; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetDesiredAngularTwist-request.msg.html

(cl:defclass <SetDesiredAngularTwist-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:AngularTwist
    :initform (cl:make-instance 'kortex_driver-msg:AngularTwist)))
)

(cl:defclass SetDesiredAngularTwist-request (<SetDesiredAngularTwist-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetDesiredAngularTwist-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetDesiredAngularTwist-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetDesiredAngularTwist-request> is deprecated: use kortex_driver-srv:SetDesiredAngularTwist-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetDesiredAngularTwist-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetDesiredAngularTwist-request>) ostream)
  "Serializes a message object of type '<SetDesiredAngularTwist-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetDesiredAngularTwist-request>) istream)
  "Deserializes a message object of type '<SetDesiredAngularTwist-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetDesiredAngularTwist-request>)))
  "Returns string type for a service object of type '<SetDesiredAngularTwist-request>"
  "kortex_driver/SetDesiredAngularTwistRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetDesiredAngularTwist-request)))
  "Returns string type for a service object of type 'SetDesiredAngularTwist-request"
  "kortex_driver/SetDesiredAngularTwistRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetDesiredAngularTwist-request>)))
  "Returns md5sum for a message object of type '<SetDesiredAngularTwist-request>"
  "74bb25440e69b788f4018de65b9d27f9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetDesiredAngularTwist-request)))
  "Returns md5sum for a message object of type 'SetDesiredAngularTwist-request"
  "74bb25440e69b788f4018de65b9d27f9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetDesiredAngularTwist-request>)))
  "Returns full string definition for message of type '<SetDesiredAngularTwist-request>"
  (cl:format cl:nil "AngularTwist input~%~%================================================================================~%MSG: kortex_driver/AngularTwist~%~%float32 angular~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetDesiredAngularTwist-request)))
  "Returns full string definition for message of type 'SetDesiredAngularTwist-request"
  (cl:format cl:nil "AngularTwist input~%~%================================================================================~%MSG: kortex_driver/AngularTwist~%~%float32 angular~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetDesiredAngularTwist-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetDesiredAngularTwist-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetDesiredAngularTwist-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetDesiredAngularTwist-response.msg.html

(cl:defclass <SetDesiredAngularTwist-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetDesiredAngularTwist-response (<SetDesiredAngularTwist-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetDesiredAngularTwist-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetDesiredAngularTwist-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetDesiredAngularTwist-response> is deprecated: use kortex_driver-srv:SetDesiredAngularTwist-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetDesiredAngularTwist-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetDesiredAngularTwist-response>) ostream)
  "Serializes a message object of type '<SetDesiredAngularTwist-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetDesiredAngularTwist-response>) istream)
  "Deserializes a message object of type '<SetDesiredAngularTwist-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetDesiredAngularTwist-response>)))
  "Returns string type for a service object of type '<SetDesiredAngularTwist-response>"
  "kortex_driver/SetDesiredAngularTwistResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetDesiredAngularTwist-response)))
  "Returns string type for a service object of type 'SetDesiredAngularTwist-response"
  "kortex_driver/SetDesiredAngularTwistResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetDesiredAngularTwist-response>)))
  "Returns md5sum for a message object of type '<SetDesiredAngularTwist-response>"
  "74bb25440e69b788f4018de65b9d27f9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetDesiredAngularTwist-response)))
  "Returns md5sum for a message object of type 'SetDesiredAngularTwist-response"
  "74bb25440e69b788f4018de65b9d27f9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetDesiredAngularTwist-response>)))
  "Returns full string definition for message of type '<SetDesiredAngularTwist-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetDesiredAngularTwist-response)))
  "Returns full string definition for message of type 'SetDesiredAngularTwist-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetDesiredAngularTwist-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetDesiredAngularTwist-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetDesiredAngularTwist-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetDesiredAngularTwist)))
  'SetDesiredAngularTwist-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetDesiredAngularTwist)))
  'SetDesiredAngularTwist-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetDesiredAngularTwist)))
  "Returns string type for a service object of type '<SetDesiredAngularTwist>"
  "kortex_driver/SetDesiredAngularTwist")