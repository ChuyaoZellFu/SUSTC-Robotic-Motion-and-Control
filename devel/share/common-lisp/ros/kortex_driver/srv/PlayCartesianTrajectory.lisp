; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude PlayCartesianTrajectory-request.msg.html

(cl:defclass <PlayCartesianTrajectory-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:ConstrainedPose
    :initform (cl:make-instance 'kortex_driver-msg:ConstrainedPose)))
)

(cl:defclass PlayCartesianTrajectory-request (<PlayCartesianTrajectory-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlayCartesianTrajectory-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlayCartesianTrajectory-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<PlayCartesianTrajectory-request> is deprecated: use kortex_driver-srv:PlayCartesianTrajectory-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <PlayCartesianTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlayCartesianTrajectory-request>) ostream)
  "Serializes a message object of type '<PlayCartesianTrajectory-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlayCartesianTrajectory-request>) istream)
  "Deserializes a message object of type '<PlayCartesianTrajectory-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlayCartesianTrajectory-request>)))
  "Returns string type for a service object of type '<PlayCartesianTrajectory-request>"
  "kortex_driver/PlayCartesianTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlayCartesianTrajectory-request)))
  "Returns string type for a service object of type 'PlayCartesianTrajectory-request"
  "kortex_driver/PlayCartesianTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlayCartesianTrajectory-request>)))
  "Returns md5sum for a message object of type '<PlayCartesianTrajectory-request>"
  "464f528a5fe3cfaa9d04f8e718f01a5c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlayCartesianTrajectory-request)))
  "Returns md5sum for a message object of type 'PlayCartesianTrajectory-request"
  "464f528a5fe3cfaa9d04f8e718f01a5c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlayCartesianTrajectory-request>)))
  "Returns full string definition for message of type '<PlayCartesianTrajectory-request>"
  (cl:format cl:nil "ConstrainedPose input~%~%================================================================================~%MSG: kortex_driver/ConstrainedPose~%~%Pose target_pose~%CartesianTrajectoryConstraint constraint~%================================================================================~%MSG: kortex_driver/Pose~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%================================================================================~%MSG: kortex_driver/CartesianTrajectoryConstraint~%~%CartesianTrajectoryConstraint_type oneof_type~%================================================================================~%MSG: kortex_driver/CartesianTrajectoryConstraint_type~%~%CartesianSpeed[] speed~%uint32[] duration~%================================================================================~%MSG: kortex_driver/CartesianSpeed~%~%float32 translation~%float32 orientation~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlayCartesianTrajectory-request)))
  "Returns full string definition for message of type 'PlayCartesianTrajectory-request"
  (cl:format cl:nil "ConstrainedPose input~%~%================================================================================~%MSG: kortex_driver/ConstrainedPose~%~%Pose target_pose~%CartesianTrajectoryConstraint constraint~%================================================================================~%MSG: kortex_driver/Pose~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%================================================================================~%MSG: kortex_driver/CartesianTrajectoryConstraint~%~%CartesianTrajectoryConstraint_type oneof_type~%================================================================================~%MSG: kortex_driver/CartesianTrajectoryConstraint_type~%~%CartesianSpeed[] speed~%uint32[] duration~%================================================================================~%MSG: kortex_driver/CartesianSpeed~%~%float32 translation~%float32 orientation~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlayCartesianTrajectory-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlayCartesianTrajectory-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PlayCartesianTrajectory-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude PlayCartesianTrajectory-response.msg.html

(cl:defclass <PlayCartesianTrajectory-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass PlayCartesianTrajectory-response (<PlayCartesianTrajectory-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlayCartesianTrajectory-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlayCartesianTrajectory-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<PlayCartesianTrajectory-response> is deprecated: use kortex_driver-srv:PlayCartesianTrajectory-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <PlayCartesianTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlayCartesianTrajectory-response>) ostream)
  "Serializes a message object of type '<PlayCartesianTrajectory-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlayCartesianTrajectory-response>) istream)
  "Deserializes a message object of type '<PlayCartesianTrajectory-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlayCartesianTrajectory-response>)))
  "Returns string type for a service object of type '<PlayCartesianTrajectory-response>"
  "kortex_driver/PlayCartesianTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlayCartesianTrajectory-response)))
  "Returns string type for a service object of type 'PlayCartesianTrajectory-response"
  "kortex_driver/PlayCartesianTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlayCartesianTrajectory-response>)))
  "Returns md5sum for a message object of type '<PlayCartesianTrajectory-response>"
  "464f528a5fe3cfaa9d04f8e718f01a5c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlayCartesianTrajectory-response)))
  "Returns md5sum for a message object of type 'PlayCartesianTrajectory-response"
  "464f528a5fe3cfaa9d04f8e718f01a5c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlayCartesianTrajectory-response>)))
  "Returns full string definition for message of type '<PlayCartesianTrajectory-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlayCartesianTrajectory-response)))
  "Returns full string definition for message of type 'PlayCartesianTrajectory-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlayCartesianTrajectory-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlayCartesianTrajectory-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PlayCartesianTrajectory-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PlayCartesianTrajectory)))
  'PlayCartesianTrajectory-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PlayCartesianTrajectory)))
  'PlayCartesianTrajectory-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlayCartesianTrajectory)))
  "Returns string type for a service object of type '<PlayCartesianTrajectory>"
  "kortex_driver/PlayCartesianTrajectory")