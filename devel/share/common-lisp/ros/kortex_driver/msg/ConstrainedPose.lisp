; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ConstrainedPose.msg.html

(cl:defclass <ConstrainedPose> (roslisp-msg-protocol:ros-message)
  ((target_pose
    :reader target_pose
    :initarg :target_pose
    :type kortex_driver-msg:Pose
    :initform (cl:make-instance 'kortex_driver-msg:Pose))
   (constraint
    :reader constraint
    :initarg :constraint
    :type kortex_driver-msg:CartesianTrajectoryConstraint
    :initform (cl:make-instance 'kortex_driver-msg:CartesianTrajectoryConstraint)))
)

(cl:defclass ConstrainedPose (<ConstrainedPose>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConstrainedPose>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConstrainedPose)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ConstrainedPose> is deprecated: use kortex_driver-msg:ConstrainedPose instead.")))

(cl:ensure-generic-function 'target_pose-val :lambda-list '(m))
(cl:defmethod target_pose-val ((m <ConstrainedPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:target_pose-val is deprecated.  Use kortex_driver-msg:target_pose instead.")
  (target_pose m))

(cl:ensure-generic-function 'constraint-val :lambda-list '(m))
(cl:defmethod constraint-val ((m <ConstrainedPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:constraint-val is deprecated.  Use kortex_driver-msg:constraint instead.")
  (constraint m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConstrainedPose>) ostream)
  "Serializes a message object of type '<ConstrainedPose>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'target_pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'constraint) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConstrainedPose>) istream)
  "Deserializes a message object of type '<ConstrainedPose>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'target_pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'constraint) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConstrainedPose>)))
  "Returns string type for a message object of type '<ConstrainedPose>"
  "kortex_driver/ConstrainedPose")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConstrainedPose)))
  "Returns string type for a message object of type 'ConstrainedPose"
  "kortex_driver/ConstrainedPose")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConstrainedPose>)))
  "Returns md5sum for a message object of type '<ConstrainedPose>"
  "201b7431451c1b450ae81fca8778070f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConstrainedPose)))
  "Returns md5sum for a message object of type 'ConstrainedPose"
  "201b7431451c1b450ae81fca8778070f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConstrainedPose>)))
  "Returns full string definition for message of type '<ConstrainedPose>"
  (cl:format cl:nil "~%Pose target_pose~%CartesianTrajectoryConstraint constraint~%================================================================================~%MSG: kortex_driver/Pose~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%================================================================================~%MSG: kortex_driver/CartesianTrajectoryConstraint~%~%CartesianTrajectoryConstraint_type oneof_type~%================================================================================~%MSG: kortex_driver/CartesianTrajectoryConstraint_type~%~%CartesianSpeed[] speed~%uint32[] duration~%================================================================================~%MSG: kortex_driver/CartesianSpeed~%~%float32 translation~%float32 orientation~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConstrainedPose)))
  "Returns full string definition for message of type 'ConstrainedPose"
  (cl:format cl:nil "~%Pose target_pose~%CartesianTrajectoryConstraint constraint~%================================================================================~%MSG: kortex_driver/Pose~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%================================================================================~%MSG: kortex_driver/CartesianTrajectoryConstraint~%~%CartesianTrajectoryConstraint_type oneof_type~%================================================================================~%MSG: kortex_driver/CartesianTrajectoryConstraint_type~%~%CartesianSpeed[] speed~%uint32[] duration~%================================================================================~%MSG: kortex_driver/CartesianSpeed~%~%float32 translation~%float32 orientation~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConstrainedPose>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'target_pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'constraint))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConstrainedPose>))
  "Converts a ROS message object to a list"
  (cl:list 'ConstrainedPose
    (cl:cons ':target_pose (target_pose msg))
    (cl:cons ':constraint (constraint msg))
))
