; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude BaseCyclic_Feedback.msg.html

(cl:defclass <BaseCyclic_Feedback> (roslisp-msg-protocol:ros-message)
  ((frame_id
    :reader frame_id
    :initarg :frame_id
    :type cl:integer
    :initform 0)
   (base
    :reader base
    :initarg :base
    :type kortex_driver-msg:BaseFeedback
    :initform (cl:make-instance 'kortex_driver-msg:BaseFeedback))
   (actuators
    :reader actuators
    :initarg :actuators
    :type (cl:vector kortex_driver-msg:ActuatorFeedback)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:ActuatorFeedback :initial-element (cl:make-instance 'kortex_driver-msg:ActuatorFeedback)))
   (interconnect
    :reader interconnect
    :initarg :interconnect
    :type kortex_driver-msg:InterconnectCyclic_Feedback
    :initform (cl:make-instance 'kortex_driver-msg:InterconnectCyclic_Feedback)))
)

(cl:defclass BaseCyclic_Feedback (<BaseCyclic_Feedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BaseCyclic_Feedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BaseCyclic_Feedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<BaseCyclic_Feedback> is deprecated: use kortex_driver-msg:BaseCyclic_Feedback instead.")))

(cl:ensure-generic-function 'frame_id-val :lambda-list '(m))
(cl:defmethod frame_id-val ((m <BaseCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:frame_id-val is deprecated.  Use kortex_driver-msg:frame_id instead.")
  (frame_id m))

(cl:ensure-generic-function 'base-val :lambda-list '(m))
(cl:defmethod base-val ((m <BaseCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:base-val is deprecated.  Use kortex_driver-msg:base instead.")
  (base m))

(cl:ensure-generic-function 'actuators-val :lambda-list '(m))
(cl:defmethod actuators-val ((m <BaseCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:actuators-val is deprecated.  Use kortex_driver-msg:actuators instead.")
  (actuators m))

(cl:ensure-generic-function 'interconnect-val :lambda-list '(m))
(cl:defmethod interconnect-val ((m <BaseCyclic_Feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:interconnect-val is deprecated.  Use kortex_driver-msg:interconnect instead.")
  (interconnect m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BaseCyclic_Feedback>) ostream)
  "Serializes a message object of type '<BaseCyclic_Feedback>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'frame_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'frame_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'frame_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'frame_id)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'base) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'actuators))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'actuators))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'interconnect) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BaseCyclic_Feedback>) istream)
  "Deserializes a message object of type '<BaseCyclic_Feedback>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'frame_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'frame_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'frame_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'frame_id)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'base) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'actuators) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'actuators)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:ActuatorFeedback))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'interconnect) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BaseCyclic_Feedback>)))
  "Returns string type for a message object of type '<BaseCyclic_Feedback>"
  "kortex_driver/BaseCyclic_Feedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BaseCyclic_Feedback)))
  "Returns string type for a message object of type 'BaseCyclic_Feedback"
  "kortex_driver/BaseCyclic_Feedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BaseCyclic_Feedback>)))
  "Returns md5sum for a message object of type '<BaseCyclic_Feedback>"
  "5f56809f61aebc2804399d7366b38f65")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BaseCyclic_Feedback)))
  "Returns md5sum for a message object of type 'BaseCyclic_Feedback"
  "5f56809f61aebc2804399d7366b38f65")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BaseCyclic_Feedback>)))
  "Returns full string definition for message of type '<BaseCyclic_Feedback>"
  (cl:format cl:nil "~%uint32 frame_id~%BaseFeedback base~%ActuatorFeedback[] actuators~%InterconnectCyclic_Feedback interconnect~%================================================================================~%MSG: kortex_driver/BaseFeedback~%~%uint32 active_state_connection_identifier~%uint32 active_state~%float32 arm_voltage~%float32 arm_current~%float32 temperature_cpu~%float32 temperature_ambient~%float32 imu_acceleration_x~%float32 imu_acceleration_y~%float32 imu_acceleration_z~%float32 imu_angular_velocity_x~%float32 imu_angular_velocity_y~%float32 imu_angular_velocity_z~%float32 tool_pose_x~%float32 tool_pose_y~%float32 tool_pose_z~%float32 tool_pose_theta_x~%float32 tool_pose_theta_y~%float32 tool_pose_theta_z~%float32 tool_twist_linear_x~%float32 tool_twist_linear_y~%float32 tool_twist_linear_z~%float32 tool_twist_angular_x~%float32 tool_twist_angular_y~%float32 tool_twist_angular_z~%float32 tool_external_wrench_force_x~%float32 tool_external_wrench_force_y~%float32 tool_external_wrench_force_z~%float32 tool_external_wrench_torque_x~%float32 tool_external_wrench_torque_y~%float32 tool_external_wrench_torque_z~%uint32 fault_bank_a~%uint32 fault_bank_b~%uint32 warning_bank_a~%uint32 warning_bank_b~%float32 commanded_tool_pose_x~%float32 commanded_tool_pose_y~%float32 commanded_tool_pose_z~%float32 commanded_tool_pose_theta_x~%float32 commanded_tool_pose_theta_y~%float32 commanded_tool_pose_theta_z~%================================================================================~%MSG: kortex_driver/ActuatorFeedback~%~%uint32 command_id~%uint32 status_flags~%uint32 jitter_comm~%float32 position~%float32 velocity~%float32 torque~%float32 current_motor~%float32 voltage~%float32 temperature_motor~%float32 temperature_core~%uint32 fault_bank_a~%uint32 fault_bank_b~%uint32 warning_bank_a~%uint32 warning_bank_b~%================================================================================~%MSG: kortex_driver/InterconnectCyclic_Feedback~%~%InterconnectCyclic_MessageId feedback_id~%uint32 status_flags~%uint32 jitter_comm~%float32 imu_acceleration_x~%float32 imu_acceleration_y~%float32 imu_acceleration_z~%float32 imu_angular_velocity_x~%float32 imu_angular_velocity_y~%float32 imu_angular_velocity_z~%float32 voltage~%float32 temperature_core~%uint32 fault_bank_a~%uint32 fault_bank_b~%uint32 warning_bank_a~%uint32 warning_bank_b~%InterconnectCyclic_Feedback_tool_feedback oneof_tool_feedback~%================================================================================~%MSG: kortex_driver/InterconnectCyclic_MessageId~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/InterconnectCyclic_Feedback_tool_feedback~%~%GripperCyclic_Feedback[] gripper_feedback~%================================================================================~%MSG: kortex_driver/GripperCyclic_Feedback~%~%GripperCyclic_MessageId feedback_id~%uint32 status_flags~%uint32 fault_bank_a~%uint32 fault_bank_b~%uint32 warning_bank_a~%uint32 warning_bank_b~%MotorFeedback[] motor~%================================================================================~%MSG: kortex_driver/GripperCyclic_MessageId~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/MotorFeedback~%~%uint32 motor_id~%float32 position~%float32 velocity~%float32 current_motor~%float32 voltage~%float32 temperature_motor~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BaseCyclic_Feedback)))
  "Returns full string definition for message of type 'BaseCyclic_Feedback"
  (cl:format cl:nil "~%uint32 frame_id~%BaseFeedback base~%ActuatorFeedback[] actuators~%InterconnectCyclic_Feedback interconnect~%================================================================================~%MSG: kortex_driver/BaseFeedback~%~%uint32 active_state_connection_identifier~%uint32 active_state~%float32 arm_voltage~%float32 arm_current~%float32 temperature_cpu~%float32 temperature_ambient~%float32 imu_acceleration_x~%float32 imu_acceleration_y~%float32 imu_acceleration_z~%float32 imu_angular_velocity_x~%float32 imu_angular_velocity_y~%float32 imu_angular_velocity_z~%float32 tool_pose_x~%float32 tool_pose_y~%float32 tool_pose_z~%float32 tool_pose_theta_x~%float32 tool_pose_theta_y~%float32 tool_pose_theta_z~%float32 tool_twist_linear_x~%float32 tool_twist_linear_y~%float32 tool_twist_linear_z~%float32 tool_twist_angular_x~%float32 tool_twist_angular_y~%float32 tool_twist_angular_z~%float32 tool_external_wrench_force_x~%float32 tool_external_wrench_force_y~%float32 tool_external_wrench_force_z~%float32 tool_external_wrench_torque_x~%float32 tool_external_wrench_torque_y~%float32 tool_external_wrench_torque_z~%uint32 fault_bank_a~%uint32 fault_bank_b~%uint32 warning_bank_a~%uint32 warning_bank_b~%float32 commanded_tool_pose_x~%float32 commanded_tool_pose_y~%float32 commanded_tool_pose_z~%float32 commanded_tool_pose_theta_x~%float32 commanded_tool_pose_theta_y~%float32 commanded_tool_pose_theta_z~%================================================================================~%MSG: kortex_driver/ActuatorFeedback~%~%uint32 command_id~%uint32 status_flags~%uint32 jitter_comm~%float32 position~%float32 velocity~%float32 torque~%float32 current_motor~%float32 voltage~%float32 temperature_motor~%float32 temperature_core~%uint32 fault_bank_a~%uint32 fault_bank_b~%uint32 warning_bank_a~%uint32 warning_bank_b~%================================================================================~%MSG: kortex_driver/InterconnectCyclic_Feedback~%~%InterconnectCyclic_MessageId feedback_id~%uint32 status_flags~%uint32 jitter_comm~%float32 imu_acceleration_x~%float32 imu_acceleration_y~%float32 imu_acceleration_z~%float32 imu_angular_velocity_x~%float32 imu_angular_velocity_y~%float32 imu_angular_velocity_z~%float32 voltage~%float32 temperature_core~%uint32 fault_bank_a~%uint32 fault_bank_b~%uint32 warning_bank_a~%uint32 warning_bank_b~%InterconnectCyclic_Feedback_tool_feedback oneof_tool_feedback~%================================================================================~%MSG: kortex_driver/InterconnectCyclic_MessageId~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/InterconnectCyclic_Feedback_tool_feedback~%~%GripperCyclic_Feedback[] gripper_feedback~%================================================================================~%MSG: kortex_driver/GripperCyclic_Feedback~%~%GripperCyclic_MessageId feedback_id~%uint32 status_flags~%uint32 fault_bank_a~%uint32 fault_bank_b~%uint32 warning_bank_a~%uint32 warning_bank_b~%MotorFeedback[] motor~%================================================================================~%MSG: kortex_driver/GripperCyclic_MessageId~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/MotorFeedback~%~%uint32 motor_id~%float32 position~%float32 velocity~%float32 current_motor~%float32 voltage~%float32 temperature_motor~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BaseCyclic_Feedback>))
  (cl:+ 0
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'base))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'actuators) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'interconnect))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BaseCyclic_Feedback>))
  "Converts a ROS message object to a list"
  (cl:list 'BaseCyclic_Feedback
    (cl:cons ':frame_id (frame_id msg))
    (cl:cons ':base (base msg))
    (cl:cons ':actuators (actuators msg))
    (cl:cons ':interconnect (interconnect msg))
))
