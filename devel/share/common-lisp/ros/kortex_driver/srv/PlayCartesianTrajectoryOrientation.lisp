; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude PlayCartesianTrajectoryOrientation-request.msg.html

(cl:defclass <PlayCartesianTrajectoryOrientation-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:ConstrainedOrientation
    :initform (cl:make-instance 'kortex_driver-msg:ConstrainedOrientation)))
)

(cl:defclass PlayCartesianTrajectoryOrientation-request (<PlayCartesianTrajectoryOrientation-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlayCartesianTrajectoryOrientation-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlayCartesianTrajectoryOrientation-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<PlayCartesianTrajectoryOrientation-request> is deprecated: use kortex_driver-srv:PlayCartesianTrajectoryOrientation-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <PlayCartesianTrajectoryOrientation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlayCartesianTrajectoryOrientation-request>) ostream)
  "Serializes a message object of type '<PlayCartesianTrajectoryOrientation-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlayCartesianTrajectoryOrientation-request>) istream)
  "Deserializes a message object of type '<PlayCartesianTrajectoryOrientation-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlayCartesianTrajectoryOrientation-request>)))
  "Returns string type for a service object of type '<PlayCartesianTrajectoryOrientation-request>"
  "kortex_driver/PlayCartesianTrajectoryOrientationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlayCartesianTrajectoryOrientation-request)))
  "Returns string type for a service object of type 'PlayCartesianTrajectoryOrientation-request"
  "kortex_driver/PlayCartesianTrajectoryOrientationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlayCartesianTrajectoryOrientation-request>)))
  "Returns md5sum for a message object of type '<PlayCartesianTrajectoryOrientation-request>"
  "118c3171acc8dd69436f0f28b7925e31")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlayCartesianTrajectoryOrientation-request)))
  "Returns md5sum for a message object of type 'PlayCartesianTrajectoryOrientation-request"
  "118c3171acc8dd69436f0f28b7925e31")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlayCartesianTrajectoryOrientation-request>)))
  "Returns full string definition for message of type '<PlayCartesianTrajectoryOrientation-request>"
  (cl:format cl:nil "ConstrainedOrientation input~%~%================================================================================~%MSG: kortex_driver/ConstrainedOrientation~%~%Orientation target_orientation~%CartesianTrajectoryConstraint constraint~%================================================================================~%MSG: kortex_driver/Orientation~%~%float32 theta_x~%float32 theta_y~%float32 theta_z~%================================================================================~%MSG: kortex_driver/CartesianTrajectoryConstraint~%~%CartesianTrajectoryConstraint_type oneof_type~%================================================================================~%MSG: kortex_driver/CartesianTrajectoryConstraint_type~%~%CartesianSpeed[] speed~%uint32[] duration~%================================================================================~%MSG: kortex_driver/CartesianSpeed~%~%float32 translation~%float32 orientation~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlayCartesianTrajectoryOrientation-request)))
  "Returns full string definition for message of type 'PlayCartesianTrajectoryOrientation-request"
  (cl:format cl:nil "ConstrainedOrientation input~%~%================================================================================~%MSG: kortex_driver/ConstrainedOrientation~%~%Orientation target_orientation~%CartesianTrajectoryConstraint constraint~%================================================================================~%MSG: kortex_driver/Orientation~%~%float32 theta_x~%float32 theta_y~%float32 theta_z~%================================================================================~%MSG: kortex_driver/CartesianTrajectoryConstraint~%~%CartesianTrajectoryConstraint_type oneof_type~%================================================================================~%MSG: kortex_driver/CartesianTrajectoryConstraint_type~%~%CartesianSpeed[] speed~%uint32[] duration~%================================================================================~%MSG: kortex_driver/CartesianSpeed~%~%float32 translation~%float32 orientation~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlayCartesianTrajectoryOrientation-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlayCartesianTrajectoryOrientation-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PlayCartesianTrajectoryOrientation-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude PlayCartesianTrajectoryOrientation-response.msg.html

(cl:defclass <PlayCartesianTrajectoryOrientation-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass PlayCartesianTrajectoryOrientation-response (<PlayCartesianTrajectoryOrientation-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlayCartesianTrajectoryOrientation-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlayCartesianTrajectoryOrientation-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<PlayCartesianTrajectoryOrientation-response> is deprecated: use kortex_driver-srv:PlayCartesianTrajectoryOrientation-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <PlayCartesianTrajectoryOrientation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlayCartesianTrajectoryOrientation-response>) ostream)
  "Serializes a message object of type '<PlayCartesianTrajectoryOrientation-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlayCartesianTrajectoryOrientation-response>) istream)
  "Deserializes a message object of type '<PlayCartesianTrajectoryOrientation-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlayCartesianTrajectoryOrientation-response>)))
  "Returns string type for a service object of type '<PlayCartesianTrajectoryOrientation-response>"
  "kortex_driver/PlayCartesianTrajectoryOrientationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlayCartesianTrajectoryOrientation-response)))
  "Returns string type for a service object of type 'PlayCartesianTrajectoryOrientation-response"
  "kortex_driver/PlayCartesianTrajectoryOrientationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlayCartesianTrajectoryOrientation-response>)))
  "Returns md5sum for a message object of type '<PlayCartesianTrajectoryOrientation-response>"
  "118c3171acc8dd69436f0f28b7925e31")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlayCartesianTrajectoryOrientation-response)))
  "Returns md5sum for a message object of type 'PlayCartesianTrajectoryOrientation-response"
  "118c3171acc8dd69436f0f28b7925e31")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlayCartesianTrajectoryOrientation-response>)))
  "Returns full string definition for message of type '<PlayCartesianTrajectoryOrientation-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlayCartesianTrajectoryOrientation-response)))
  "Returns full string definition for message of type 'PlayCartesianTrajectoryOrientation-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlayCartesianTrajectoryOrientation-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlayCartesianTrajectoryOrientation-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PlayCartesianTrajectoryOrientation-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PlayCartesianTrajectoryOrientation)))
  'PlayCartesianTrajectoryOrientation-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PlayCartesianTrajectoryOrientation)))
  'PlayCartesianTrajectoryOrientation-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlayCartesianTrajectoryOrientation)))
  "Returns string type for a service object of type '<PlayCartesianTrajectoryOrientation>"
  "kortex_driver/PlayCartesianTrajectoryOrientation")