; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ConstrainedJointAngles.msg.html

(cl:defclass <ConstrainedJointAngles> (roslisp-msg-protocol:ros-message)
  ((joint_angles
    :reader joint_angles
    :initarg :joint_angles
    :type kortex_driver-msg:JointAngles
    :initform (cl:make-instance 'kortex_driver-msg:JointAngles))
   (constraint
    :reader constraint
    :initarg :constraint
    :type kortex_driver-msg:JointTrajectoryConstraint
    :initform (cl:make-instance 'kortex_driver-msg:JointTrajectoryConstraint)))
)

(cl:defclass ConstrainedJointAngles (<ConstrainedJointAngles>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConstrainedJointAngles>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConstrainedJointAngles)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ConstrainedJointAngles> is deprecated: use kortex_driver-msg:ConstrainedJointAngles instead.")))

(cl:ensure-generic-function 'joint_angles-val :lambda-list '(m))
(cl:defmethod joint_angles-val ((m <ConstrainedJointAngles>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:joint_angles-val is deprecated.  Use kortex_driver-msg:joint_angles instead.")
  (joint_angles m))

(cl:ensure-generic-function 'constraint-val :lambda-list '(m))
(cl:defmethod constraint-val ((m <ConstrainedJointAngles>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:constraint-val is deprecated.  Use kortex_driver-msg:constraint instead.")
  (constraint m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConstrainedJointAngles>) ostream)
  "Serializes a message object of type '<ConstrainedJointAngles>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'joint_angles) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'constraint) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConstrainedJointAngles>) istream)
  "Deserializes a message object of type '<ConstrainedJointAngles>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'joint_angles) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'constraint) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConstrainedJointAngles>)))
  "Returns string type for a message object of type '<ConstrainedJointAngles>"
  "kortex_driver/ConstrainedJointAngles")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConstrainedJointAngles)))
  "Returns string type for a message object of type 'ConstrainedJointAngles"
  "kortex_driver/ConstrainedJointAngles")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConstrainedJointAngles>)))
  "Returns md5sum for a message object of type '<ConstrainedJointAngles>"
  "75faad16493b9d4c290ef3b7fb5a8947")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConstrainedJointAngles)))
  "Returns md5sum for a message object of type 'ConstrainedJointAngles"
  "75faad16493b9d4c290ef3b7fb5a8947")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConstrainedJointAngles>)))
  "Returns full string definition for message of type '<ConstrainedJointAngles>"
  (cl:format cl:nil "~%JointAngles joint_angles~%JointTrajectoryConstraint constraint~%================================================================================~%MSG: kortex_driver/JointAngles~%~%JointAngle[] joint_angles~%================================================================================~%MSG: kortex_driver/JointAngle~%~%uint32 joint_identifier~%float32 value~%================================================================================~%MSG: kortex_driver/JointTrajectoryConstraint~%~%uint32 type~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConstrainedJointAngles)))
  "Returns full string definition for message of type 'ConstrainedJointAngles"
  (cl:format cl:nil "~%JointAngles joint_angles~%JointTrajectoryConstraint constraint~%================================================================================~%MSG: kortex_driver/JointAngles~%~%JointAngle[] joint_angles~%================================================================================~%MSG: kortex_driver/JointAngle~%~%uint32 joint_identifier~%float32 value~%================================================================================~%MSG: kortex_driver/JointTrajectoryConstraint~%~%uint32 type~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConstrainedJointAngles>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'joint_angles))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'constraint))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConstrainedJointAngles>))
  "Converts a ROS message object to a list"
  (cl:list 'ConstrainedJointAngles
    (cl:cons ':joint_angles (joint_angles msg))
    (cl:cons ':constraint (constraint msg))
))
