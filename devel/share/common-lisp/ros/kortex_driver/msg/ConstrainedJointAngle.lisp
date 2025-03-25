; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ConstrainedJointAngle.msg.html

(cl:defclass <ConstrainedJointAngle> (roslisp-msg-protocol:ros-message)
  ((joint_identifier
    :reader joint_identifier
    :initarg :joint_identifier
    :type cl:integer
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:float
    :initform 0.0)
   (constraint
    :reader constraint
    :initarg :constraint
    :type kortex_driver-msg:JointTrajectoryConstraint
    :initform (cl:make-instance 'kortex_driver-msg:JointTrajectoryConstraint)))
)

(cl:defclass ConstrainedJointAngle (<ConstrainedJointAngle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConstrainedJointAngle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConstrainedJointAngle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ConstrainedJointAngle> is deprecated: use kortex_driver-msg:ConstrainedJointAngle instead.")))

(cl:ensure-generic-function 'joint_identifier-val :lambda-list '(m))
(cl:defmethod joint_identifier-val ((m <ConstrainedJointAngle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:joint_identifier-val is deprecated.  Use kortex_driver-msg:joint_identifier instead.")
  (joint_identifier m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <ConstrainedJointAngle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:value-val is deprecated.  Use kortex_driver-msg:value instead.")
  (value m))

(cl:ensure-generic-function 'constraint-val :lambda-list '(m))
(cl:defmethod constraint-val ((m <ConstrainedJointAngle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:constraint-val is deprecated.  Use kortex_driver-msg:constraint instead.")
  (constraint m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConstrainedJointAngle>) ostream)
  "Serializes a message object of type '<ConstrainedJointAngle>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'joint_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'joint_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'joint_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'joint_identifier)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'constraint) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConstrainedJointAngle>) istream)
  "Deserializes a message object of type '<ConstrainedJointAngle>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'joint_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'joint_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'joint_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'joint_identifier)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'value) (roslisp-utils:decode-single-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'constraint) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConstrainedJointAngle>)))
  "Returns string type for a message object of type '<ConstrainedJointAngle>"
  "kortex_driver/ConstrainedJointAngle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConstrainedJointAngle)))
  "Returns string type for a message object of type 'ConstrainedJointAngle"
  "kortex_driver/ConstrainedJointAngle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConstrainedJointAngle>)))
  "Returns md5sum for a message object of type '<ConstrainedJointAngle>"
  "6fb07b64148f47061948eb6c6ef38e00")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConstrainedJointAngle)))
  "Returns md5sum for a message object of type 'ConstrainedJointAngle"
  "6fb07b64148f47061948eb6c6ef38e00")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConstrainedJointAngle>)))
  "Returns full string definition for message of type '<ConstrainedJointAngle>"
  (cl:format cl:nil "~%uint32 joint_identifier~%float32 value~%JointTrajectoryConstraint constraint~%================================================================================~%MSG: kortex_driver/JointTrajectoryConstraint~%~%uint32 type~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConstrainedJointAngle)))
  "Returns full string definition for message of type 'ConstrainedJointAngle"
  (cl:format cl:nil "~%uint32 joint_identifier~%float32 value~%JointTrajectoryConstraint constraint~%================================================================================~%MSG: kortex_driver/JointTrajectoryConstraint~%~%uint32 type~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConstrainedJointAngle>))
  (cl:+ 0
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'constraint))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConstrainedJointAngle>))
  "Converts a ROS message object to a list"
  (cl:list 'ConstrainedJointAngle
    (cl:cons ':joint_identifier (joint_identifier msg))
    (cl:cons ':value (value msg))
    (cl:cons ':constraint (constraint msg))
))
