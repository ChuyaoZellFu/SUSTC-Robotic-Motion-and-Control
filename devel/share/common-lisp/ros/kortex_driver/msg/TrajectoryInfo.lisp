; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude TrajectoryInfo.msg.html

(cl:defclass <TrajectoryInfo> (roslisp-msg-protocol:ros-message)
  ((trajectory_info_type
    :reader trajectory_info_type
    :initarg :trajectory_info_type
    :type cl:integer
    :initform 0)
   (waypoint_index
    :reader waypoint_index
    :initarg :waypoint_index
    :type cl:integer
    :initform 0)
   (joint_index
    :reader joint_index
    :initarg :joint_index
    :type cl:integer
    :initform 0))
)

(cl:defclass TrajectoryInfo (<TrajectoryInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrajectoryInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrajectoryInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<TrajectoryInfo> is deprecated: use kortex_driver-msg:TrajectoryInfo instead.")))

(cl:ensure-generic-function 'trajectory_info_type-val :lambda-list '(m))
(cl:defmethod trajectory_info_type-val ((m <TrajectoryInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:trajectory_info_type-val is deprecated.  Use kortex_driver-msg:trajectory_info_type instead.")
  (trajectory_info_type m))

(cl:ensure-generic-function 'waypoint_index-val :lambda-list '(m))
(cl:defmethod waypoint_index-val ((m <TrajectoryInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:waypoint_index-val is deprecated.  Use kortex_driver-msg:waypoint_index instead.")
  (waypoint_index m))

(cl:ensure-generic-function 'joint_index-val :lambda-list '(m))
(cl:defmethod joint_index-val ((m <TrajectoryInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:joint_index-val is deprecated.  Use kortex_driver-msg:joint_index instead.")
  (joint_index m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrajectoryInfo>) ostream)
  "Serializes a message object of type '<TrajectoryInfo>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'trajectory_info_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'trajectory_info_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'trajectory_info_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'trajectory_info_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'waypoint_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'waypoint_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'waypoint_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'waypoint_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'joint_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'joint_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'joint_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'joint_index)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrajectoryInfo>) istream)
  "Deserializes a message object of type '<TrajectoryInfo>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'trajectory_info_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'trajectory_info_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'trajectory_info_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'trajectory_info_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'waypoint_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'waypoint_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'waypoint_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'waypoint_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'joint_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'joint_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'joint_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'joint_index)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrajectoryInfo>)))
  "Returns string type for a message object of type '<TrajectoryInfo>"
  "kortex_driver/TrajectoryInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrajectoryInfo)))
  "Returns string type for a message object of type 'TrajectoryInfo"
  "kortex_driver/TrajectoryInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrajectoryInfo>)))
  "Returns md5sum for a message object of type '<TrajectoryInfo>"
  "0eff35f5790d1aa2c620bfca263340d6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrajectoryInfo)))
  "Returns md5sum for a message object of type 'TrajectoryInfo"
  "0eff35f5790d1aa2c620bfca263340d6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrajectoryInfo>)))
  "Returns full string definition for message of type '<TrajectoryInfo>"
  (cl:format cl:nil "~%uint32 trajectory_info_type~%uint32 waypoint_index~%uint32 joint_index~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrajectoryInfo)))
  "Returns full string definition for message of type 'TrajectoryInfo"
  (cl:format cl:nil "~%uint32 trajectory_info_type~%uint32 waypoint_index~%uint32 joint_index~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrajectoryInfo>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrajectoryInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'TrajectoryInfo
    (cl:cons ':trajectory_info_type (trajectory_info_type msg))
    (cl:cons ':waypoint_index (waypoint_index msg))
    (cl:cons ':joint_index (joint_index msg))
))
