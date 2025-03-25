; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetMeasuredCartesianPose-request.msg.html

(cl:defclass <GetMeasuredCartesianPose-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetMeasuredCartesianPose-request (<GetMeasuredCartesianPose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetMeasuredCartesianPose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetMeasuredCartesianPose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetMeasuredCartesianPose-request> is deprecated: use kortex_driver-srv:GetMeasuredCartesianPose-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetMeasuredCartesianPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetMeasuredCartesianPose-request>) ostream)
  "Serializes a message object of type '<GetMeasuredCartesianPose-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetMeasuredCartesianPose-request>) istream)
  "Deserializes a message object of type '<GetMeasuredCartesianPose-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetMeasuredCartesianPose-request>)))
  "Returns string type for a service object of type '<GetMeasuredCartesianPose-request>"
  "kortex_driver/GetMeasuredCartesianPoseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetMeasuredCartesianPose-request)))
  "Returns string type for a service object of type 'GetMeasuredCartesianPose-request"
  "kortex_driver/GetMeasuredCartesianPoseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetMeasuredCartesianPose-request>)))
  "Returns md5sum for a message object of type '<GetMeasuredCartesianPose-request>"
  "4eaf9f25cbe269662009a9ad397d3f07")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetMeasuredCartesianPose-request)))
  "Returns md5sum for a message object of type 'GetMeasuredCartesianPose-request"
  "4eaf9f25cbe269662009a9ad397d3f07")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetMeasuredCartesianPose-request>)))
  "Returns full string definition for message of type '<GetMeasuredCartesianPose-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetMeasuredCartesianPose-request)))
  "Returns full string definition for message of type 'GetMeasuredCartesianPose-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetMeasuredCartesianPose-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetMeasuredCartesianPose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetMeasuredCartesianPose-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetMeasuredCartesianPose-response.msg.html

(cl:defclass <GetMeasuredCartesianPose-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Pose
    :initform (cl:make-instance 'kortex_driver-msg:Pose)))
)

(cl:defclass GetMeasuredCartesianPose-response (<GetMeasuredCartesianPose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetMeasuredCartesianPose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetMeasuredCartesianPose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetMeasuredCartesianPose-response> is deprecated: use kortex_driver-srv:GetMeasuredCartesianPose-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetMeasuredCartesianPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetMeasuredCartesianPose-response>) ostream)
  "Serializes a message object of type '<GetMeasuredCartesianPose-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetMeasuredCartesianPose-response>) istream)
  "Deserializes a message object of type '<GetMeasuredCartesianPose-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetMeasuredCartesianPose-response>)))
  "Returns string type for a service object of type '<GetMeasuredCartesianPose-response>"
  "kortex_driver/GetMeasuredCartesianPoseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetMeasuredCartesianPose-response)))
  "Returns string type for a service object of type 'GetMeasuredCartesianPose-response"
  "kortex_driver/GetMeasuredCartesianPoseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetMeasuredCartesianPose-response>)))
  "Returns md5sum for a message object of type '<GetMeasuredCartesianPose-response>"
  "4eaf9f25cbe269662009a9ad397d3f07")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetMeasuredCartesianPose-response)))
  "Returns md5sum for a message object of type 'GetMeasuredCartesianPose-response"
  "4eaf9f25cbe269662009a9ad397d3f07")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetMeasuredCartesianPose-response>)))
  "Returns full string definition for message of type '<GetMeasuredCartesianPose-response>"
  (cl:format cl:nil "Pose output~%~%================================================================================~%MSG: kortex_driver/Pose~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetMeasuredCartesianPose-response)))
  "Returns full string definition for message of type 'GetMeasuredCartesianPose-response"
  (cl:format cl:nil "Pose output~%~%================================================================================~%MSG: kortex_driver/Pose~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetMeasuredCartesianPose-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetMeasuredCartesianPose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetMeasuredCartesianPose-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetMeasuredCartesianPose)))
  'GetMeasuredCartesianPose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetMeasuredCartesianPose)))
  'GetMeasuredCartesianPose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetMeasuredCartesianPose)))
  "Returns string type for a service object of type '<GetMeasuredCartesianPose>"
  "kortex_driver/GetMeasuredCartesianPose")