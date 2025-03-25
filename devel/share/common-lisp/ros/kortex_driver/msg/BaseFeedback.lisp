; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude BaseFeedback.msg.html

(cl:defclass <BaseFeedback> (roslisp-msg-protocol:ros-message)
  ((active_state_connection_identifier
    :reader active_state_connection_identifier
    :initarg :active_state_connection_identifier
    :type cl:integer
    :initform 0)
   (active_state
    :reader active_state
    :initarg :active_state
    :type cl:integer
    :initform 0)
   (arm_voltage
    :reader arm_voltage
    :initarg :arm_voltage
    :type cl:float
    :initform 0.0)
   (arm_current
    :reader arm_current
    :initarg :arm_current
    :type cl:float
    :initform 0.0)
   (temperature_cpu
    :reader temperature_cpu
    :initarg :temperature_cpu
    :type cl:float
    :initform 0.0)
   (temperature_ambient
    :reader temperature_ambient
    :initarg :temperature_ambient
    :type cl:float
    :initform 0.0)
   (imu_acceleration_x
    :reader imu_acceleration_x
    :initarg :imu_acceleration_x
    :type cl:float
    :initform 0.0)
   (imu_acceleration_y
    :reader imu_acceleration_y
    :initarg :imu_acceleration_y
    :type cl:float
    :initform 0.0)
   (imu_acceleration_z
    :reader imu_acceleration_z
    :initarg :imu_acceleration_z
    :type cl:float
    :initform 0.0)
   (imu_angular_velocity_x
    :reader imu_angular_velocity_x
    :initarg :imu_angular_velocity_x
    :type cl:float
    :initform 0.0)
   (imu_angular_velocity_y
    :reader imu_angular_velocity_y
    :initarg :imu_angular_velocity_y
    :type cl:float
    :initform 0.0)
   (imu_angular_velocity_z
    :reader imu_angular_velocity_z
    :initarg :imu_angular_velocity_z
    :type cl:float
    :initform 0.0)
   (tool_pose_x
    :reader tool_pose_x
    :initarg :tool_pose_x
    :type cl:float
    :initform 0.0)
   (tool_pose_y
    :reader tool_pose_y
    :initarg :tool_pose_y
    :type cl:float
    :initform 0.0)
   (tool_pose_z
    :reader tool_pose_z
    :initarg :tool_pose_z
    :type cl:float
    :initform 0.0)
   (tool_pose_theta_x
    :reader tool_pose_theta_x
    :initarg :tool_pose_theta_x
    :type cl:float
    :initform 0.0)
   (tool_pose_theta_y
    :reader tool_pose_theta_y
    :initarg :tool_pose_theta_y
    :type cl:float
    :initform 0.0)
   (tool_pose_theta_z
    :reader tool_pose_theta_z
    :initarg :tool_pose_theta_z
    :type cl:float
    :initform 0.0)
   (tool_twist_linear_x
    :reader tool_twist_linear_x
    :initarg :tool_twist_linear_x
    :type cl:float
    :initform 0.0)
   (tool_twist_linear_y
    :reader tool_twist_linear_y
    :initarg :tool_twist_linear_y
    :type cl:float
    :initform 0.0)
   (tool_twist_linear_z
    :reader tool_twist_linear_z
    :initarg :tool_twist_linear_z
    :type cl:float
    :initform 0.0)
   (tool_twist_angular_x
    :reader tool_twist_angular_x
    :initarg :tool_twist_angular_x
    :type cl:float
    :initform 0.0)
   (tool_twist_angular_y
    :reader tool_twist_angular_y
    :initarg :tool_twist_angular_y
    :type cl:float
    :initform 0.0)
   (tool_twist_angular_z
    :reader tool_twist_angular_z
    :initarg :tool_twist_angular_z
    :type cl:float
    :initform 0.0)
   (tool_external_wrench_force_x
    :reader tool_external_wrench_force_x
    :initarg :tool_external_wrench_force_x
    :type cl:float
    :initform 0.0)
   (tool_external_wrench_force_y
    :reader tool_external_wrench_force_y
    :initarg :tool_external_wrench_force_y
    :type cl:float
    :initform 0.0)
   (tool_external_wrench_force_z
    :reader tool_external_wrench_force_z
    :initarg :tool_external_wrench_force_z
    :type cl:float
    :initform 0.0)
   (tool_external_wrench_torque_x
    :reader tool_external_wrench_torque_x
    :initarg :tool_external_wrench_torque_x
    :type cl:float
    :initform 0.0)
   (tool_external_wrench_torque_y
    :reader tool_external_wrench_torque_y
    :initarg :tool_external_wrench_torque_y
    :type cl:float
    :initform 0.0)
   (tool_external_wrench_torque_z
    :reader tool_external_wrench_torque_z
    :initarg :tool_external_wrench_torque_z
    :type cl:float
    :initform 0.0)
   (fault_bank_a
    :reader fault_bank_a
    :initarg :fault_bank_a
    :type cl:integer
    :initform 0)
   (fault_bank_b
    :reader fault_bank_b
    :initarg :fault_bank_b
    :type cl:integer
    :initform 0)
   (warning_bank_a
    :reader warning_bank_a
    :initarg :warning_bank_a
    :type cl:integer
    :initform 0)
   (warning_bank_b
    :reader warning_bank_b
    :initarg :warning_bank_b
    :type cl:integer
    :initform 0)
   (commanded_tool_pose_x
    :reader commanded_tool_pose_x
    :initarg :commanded_tool_pose_x
    :type cl:float
    :initform 0.0)
   (commanded_tool_pose_y
    :reader commanded_tool_pose_y
    :initarg :commanded_tool_pose_y
    :type cl:float
    :initform 0.0)
   (commanded_tool_pose_z
    :reader commanded_tool_pose_z
    :initarg :commanded_tool_pose_z
    :type cl:float
    :initform 0.0)
   (commanded_tool_pose_theta_x
    :reader commanded_tool_pose_theta_x
    :initarg :commanded_tool_pose_theta_x
    :type cl:float
    :initform 0.0)
   (commanded_tool_pose_theta_y
    :reader commanded_tool_pose_theta_y
    :initarg :commanded_tool_pose_theta_y
    :type cl:float
    :initform 0.0)
   (commanded_tool_pose_theta_z
    :reader commanded_tool_pose_theta_z
    :initarg :commanded_tool_pose_theta_z
    :type cl:float
    :initform 0.0))
)

(cl:defclass BaseFeedback (<BaseFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BaseFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BaseFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<BaseFeedback> is deprecated: use kortex_driver-msg:BaseFeedback instead.")))

(cl:ensure-generic-function 'active_state_connection_identifier-val :lambda-list '(m))
(cl:defmethod active_state_connection_identifier-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:active_state_connection_identifier-val is deprecated.  Use kortex_driver-msg:active_state_connection_identifier instead.")
  (active_state_connection_identifier m))

(cl:ensure-generic-function 'active_state-val :lambda-list '(m))
(cl:defmethod active_state-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:active_state-val is deprecated.  Use kortex_driver-msg:active_state instead.")
  (active_state m))

(cl:ensure-generic-function 'arm_voltage-val :lambda-list '(m))
(cl:defmethod arm_voltage-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:arm_voltage-val is deprecated.  Use kortex_driver-msg:arm_voltage instead.")
  (arm_voltage m))

(cl:ensure-generic-function 'arm_current-val :lambda-list '(m))
(cl:defmethod arm_current-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:arm_current-val is deprecated.  Use kortex_driver-msg:arm_current instead.")
  (arm_current m))

(cl:ensure-generic-function 'temperature_cpu-val :lambda-list '(m))
(cl:defmethod temperature_cpu-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:temperature_cpu-val is deprecated.  Use kortex_driver-msg:temperature_cpu instead.")
  (temperature_cpu m))

(cl:ensure-generic-function 'temperature_ambient-val :lambda-list '(m))
(cl:defmethod temperature_ambient-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:temperature_ambient-val is deprecated.  Use kortex_driver-msg:temperature_ambient instead.")
  (temperature_ambient m))

(cl:ensure-generic-function 'imu_acceleration_x-val :lambda-list '(m))
(cl:defmethod imu_acceleration_x-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:imu_acceleration_x-val is deprecated.  Use kortex_driver-msg:imu_acceleration_x instead.")
  (imu_acceleration_x m))

(cl:ensure-generic-function 'imu_acceleration_y-val :lambda-list '(m))
(cl:defmethod imu_acceleration_y-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:imu_acceleration_y-val is deprecated.  Use kortex_driver-msg:imu_acceleration_y instead.")
  (imu_acceleration_y m))

(cl:ensure-generic-function 'imu_acceleration_z-val :lambda-list '(m))
(cl:defmethod imu_acceleration_z-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:imu_acceleration_z-val is deprecated.  Use kortex_driver-msg:imu_acceleration_z instead.")
  (imu_acceleration_z m))

(cl:ensure-generic-function 'imu_angular_velocity_x-val :lambda-list '(m))
(cl:defmethod imu_angular_velocity_x-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:imu_angular_velocity_x-val is deprecated.  Use kortex_driver-msg:imu_angular_velocity_x instead.")
  (imu_angular_velocity_x m))

(cl:ensure-generic-function 'imu_angular_velocity_y-val :lambda-list '(m))
(cl:defmethod imu_angular_velocity_y-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:imu_angular_velocity_y-val is deprecated.  Use kortex_driver-msg:imu_angular_velocity_y instead.")
  (imu_angular_velocity_y m))

(cl:ensure-generic-function 'imu_angular_velocity_z-val :lambda-list '(m))
(cl:defmethod imu_angular_velocity_z-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:imu_angular_velocity_z-val is deprecated.  Use kortex_driver-msg:imu_angular_velocity_z instead.")
  (imu_angular_velocity_z m))

(cl:ensure-generic-function 'tool_pose_x-val :lambda-list '(m))
(cl:defmethod tool_pose_x-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:tool_pose_x-val is deprecated.  Use kortex_driver-msg:tool_pose_x instead.")
  (tool_pose_x m))

(cl:ensure-generic-function 'tool_pose_y-val :lambda-list '(m))
(cl:defmethod tool_pose_y-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:tool_pose_y-val is deprecated.  Use kortex_driver-msg:tool_pose_y instead.")
  (tool_pose_y m))

(cl:ensure-generic-function 'tool_pose_z-val :lambda-list '(m))
(cl:defmethod tool_pose_z-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:tool_pose_z-val is deprecated.  Use kortex_driver-msg:tool_pose_z instead.")
  (tool_pose_z m))

(cl:ensure-generic-function 'tool_pose_theta_x-val :lambda-list '(m))
(cl:defmethod tool_pose_theta_x-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:tool_pose_theta_x-val is deprecated.  Use kortex_driver-msg:tool_pose_theta_x instead.")
  (tool_pose_theta_x m))

(cl:ensure-generic-function 'tool_pose_theta_y-val :lambda-list '(m))
(cl:defmethod tool_pose_theta_y-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:tool_pose_theta_y-val is deprecated.  Use kortex_driver-msg:tool_pose_theta_y instead.")
  (tool_pose_theta_y m))

(cl:ensure-generic-function 'tool_pose_theta_z-val :lambda-list '(m))
(cl:defmethod tool_pose_theta_z-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:tool_pose_theta_z-val is deprecated.  Use kortex_driver-msg:tool_pose_theta_z instead.")
  (tool_pose_theta_z m))

(cl:ensure-generic-function 'tool_twist_linear_x-val :lambda-list '(m))
(cl:defmethod tool_twist_linear_x-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:tool_twist_linear_x-val is deprecated.  Use kortex_driver-msg:tool_twist_linear_x instead.")
  (tool_twist_linear_x m))

(cl:ensure-generic-function 'tool_twist_linear_y-val :lambda-list '(m))
(cl:defmethod tool_twist_linear_y-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:tool_twist_linear_y-val is deprecated.  Use kortex_driver-msg:tool_twist_linear_y instead.")
  (tool_twist_linear_y m))

(cl:ensure-generic-function 'tool_twist_linear_z-val :lambda-list '(m))
(cl:defmethod tool_twist_linear_z-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:tool_twist_linear_z-val is deprecated.  Use kortex_driver-msg:tool_twist_linear_z instead.")
  (tool_twist_linear_z m))

(cl:ensure-generic-function 'tool_twist_angular_x-val :lambda-list '(m))
(cl:defmethod tool_twist_angular_x-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:tool_twist_angular_x-val is deprecated.  Use kortex_driver-msg:tool_twist_angular_x instead.")
  (tool_twist_angular_x m))

(cl:ensure-generic-function 'tool_twist_angular_y-val :lambda-list '(m))
(cl:defmethod tool_twist_angular_y-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:tool_twist_angular_y-val is deprecated.  Use kortex_driver-msg:tool_twist_angular_y instead.")
  (tool_twist_angular_y m))

(cl:ensure-generic-function 'tool_twist_angular_z-val :lambda-list '(m))
(cl:defmethod tool_twist_angular_z-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:tool_twist_angular_z-val is deprecated.  Use kortex_driver-msg:tool_twist_angular_z instead.")
  (tool_twist_angular_z m))

(cl:ensure-generic-function 'tool_external_wrench_force_x-val :lambda-list '(m))
(cl:defmethod tool_external_wrench_force_x-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:tool_external_wrench_force_x-val is deprecated.  Use kortex_driver-msg:tool_external_wrench_force_x instead.")
  (tool_external_wrench_force_x m))

(cl:ensure-generic-function 'tool_external_wrench_force_y-val :lambda-list '(m))
(cl:defmethod tool_external_wrench_force_y-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:tool_external_wrench_force_y-val is deprecated.  Use kortex_driver-msg:tool_external_wrench_force_y instead.")
  (tool_external_wrench_force_y m))

(cl:ensure-generic-function 'tool_external_wrench_force_z-val :lambda-list '(m))
(cl:defmethod tool_external_wrench_force_z-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:tool_external_wrench_force_z-val is deprecated.  Use kortex_driver-msg:tool_external_wrench_force_z instead.")
  (tool_external_wrench_force_z m))

(cl:ensure-generic-function 'tool_external_wrench_torque_x-val :lambda-list '(m))
(cl:defmethod tool_external_wrench_torque_x-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:tool_external_wrench_torque_x-val is deprecated.  Use kortex_driver-msg:tool_external_wrench_torque_x instead.")
  (tool_external_wrench_torque_x m))

(cl:ensure-generic-function 'tool_external_wrench_torque_y-val :lambda-list '(m))
(cl:defmethod tool_external_wrench_torque_y-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:tool_external_wrench_torque_y-val is deprecated.  Use kortex_driver-msg:tool_external_wrench_torque_y instead.")
  (tool_external_wrench_torque_y m))

(cl:ensure-generic-function 'tool_external_wrench_torque_z-val :lambda-list '(m))
(cl:defmethod tool_external_wrench_torque_z-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:tool_external_wrench_torque_z-val is deprecated.  Use kortex_driver-msg:tool_external_wrench_torque_z instead.")
  (tool_external_wrench_torque_z m))

(cl:ensure-generic-function 'fault_bank_a-val :lambda-list '(m))
(cl:defmethod fault_bank_a-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:fault_bank_a-val is deprecated.  Use kortex_driver-msg:fault_bank_a instead.")
  (fault_bank_a m))

(cl:ensure-generic-function 'fault_bank_b-val :lambda-list '(m))
(cl:defmethod fault_bank_b-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:fault_bank_b-val is deprecated.  Use kortex_driver-msg:fault_bank_b instead.")
  (fault_bank_b m))

(cl:ensure-generic-function 'warning_bank_a-val :lambda-list '(m))
(cl:defmethod warning_bank_a-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:warning_bank_a-val is deprecated.  Use kortex_driver-msg:warning_bank_a instead.")
  (warning_bank_a m))

(cl:ensure-generic-function 'warning_bank_b-val :lambda-list '(m))
(cl:defmethod warning_bank_b-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:warning_bank_b-val is deprecated.  Use kortex_driver-msg:warning_bank_b instead.")
  (warning_bank_b m))

(cl:ensure-generic-function 'commanded_tool_pose_x-val :lambda-list '(m))
(cl:defmethod commanded_tool_pose_x-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:commanded_tool_pose_x-val is deprecated.  Use kortex_driver-msg:commanded_tool_pose_x instead.")
  (commanded_tool_pose_x m))

(cl:ensure-generic-function 'commanded_tool_pose_y-val :lambda-list '(m))
(cl:defmethod commanded_tool_pose_y-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:commanded_tool_pose_y-val is deprecated.  Use kortex_driver-msg:commanded_tool_pose_y instead.")
  (commanded_tool_pose_y m))

(cl:ensure-generic-function 'commanded_tool_pose_z-val :lambda-list '(m))
(cl:defmethod commanded_tool_pose_z-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:commanded_tool_pose_z-val is deprecated.  Use kortex_driver-msg:commanded_tool_pose_z instead.")
  (commanded_tool_pose_z m))

(cl:ensure-generic-function 'commanded_tool_pose_theta_x-val :lambda-list '(m))
(cl:defmethod commanded_tool_pose_theta_x-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:commanded_tool_pose_theta_x-val is deprecated.  Use kortex_driver-msg:commanded_tool_pose_theta_x instead.")
  (commanded_tool_pose_theta_x m))

(cl:ensure-generic-function 'commanded_tool_pose_theta_y-val :lambda-list '(m))
(cl:defmethod commanded_tool_pose_theta_y-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:commanded_tool_pose_theta_y-val is deprecated.  Use kortex_driver-msg:commanded_tool_pose_theta_y instead.")
  (commanded_tool_pose_theta_y m))

(cl:ensure-generic-function 'commanded_tool_pose_theta_z-val :lambda-list '(m))
(cl:defmethod commanded_tool_pose_theta_z-val ((m <BaseFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:commanded_tool_pose_theta_z-val is deprecated.  Use kortex_driver-msg:commanded_tool_pose_theta_z instead.")
  (commanded_tool_pose_theta_z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BaseFeedback>) ostream)
  "Serializes a message object of type '<BaseFeedback>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'active_state_connection_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'active_state_connection_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'active_state_connection_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'active_state_connection_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'active_state)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'active_state)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'active_state)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'active_state)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'arm_voltage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'arm_current))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'temperature_cpu))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'temperature_ambient))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'imu_acceleration_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'imu_acceleration_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'imu_acceleration_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'imu_angular_velocity_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'imu_angular_velocity_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'imu_angular_velocity_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tool_pose_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tool_pose_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tool_pose_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tool_pose_theta_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tool_pose_theta_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tool_pose_theta_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tool_twist_linear_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tool_twist_linear_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tool_twist_linear_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tool_twist_angular_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tool_twist_angular_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tool_twist_angular_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tool_external_wrench_force_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tool_external_wrench_force_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tool_external_wrench_force_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tool_external_wrench_torque_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tool_external_wrench_torque_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tool_external_wrench_torque_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fault_bank_a)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'fault_bank_a)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'fault_bank_a)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'fault_bank_a)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fault_bank_b)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'fault_bank_b)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'fault_bank_b)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'fault_bank_b)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'warning_bank_a)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'warning_bank_a)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'warning_bank_a)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'warning_bank_a)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'warning_bank_b)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'warning_bank_b)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'warning_bank_b)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'warning_bank_b)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'commanded_tool_pose_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'commanded_tool_pose_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'commanded_tool_pose_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'commanded_tool_pose_theta_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'commanded_tool_pose_theta_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'commanded_tool_pose_theta_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BaseFeedback>) istream)
  "Deserializes a message object of type '<BaseFeedback>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'active_state_connection_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'active_state_connection_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'active_state_connection_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'active_state_connection_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'active_state)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'active_state)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'active_state)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'active_state)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'arm_voltage) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'arm_current) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'temperature_cpu) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'temperature_ambient) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'imu_acceleration_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'imu_acceleration_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'imu_acceleration_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'imu_angular_velocity_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'imu_angular_velocity_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'imu_angular_velocity_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tool_pose_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tool_pose_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tool_pose_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tool_pose_theta_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tool_pose_theta_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tool_pose_theta_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tool_twist_linear_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tool_twist_linear_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tool_twist_linear_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tool_twist_angular_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tool_twist_angular_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tool_twist_angular_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tool_external_wrench_force_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tool_external_wrench_force_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tool_external_wrench_force_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tool_external_wrench_torque_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tool_external_wrench_torque_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tool_external_wrench_torque_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fault_bank_a)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'fault_bank_a)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'fault_bank_a)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'fault_bank_a)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fault_bank_b)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'fault_bank_b)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'fault_bank_b)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'fault_bank_b)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'warning_bank_a)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'warning_bank_a)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'warning_bank_a)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'warning_bank_a)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'warning_bank_b)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'warning_bank_b)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'warning_bank_b)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'warning_bank_b)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'commanded_tool_pose_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'commanded_tool_pose_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'commanded_tool_pose_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'commanded_tool_pose_theta_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'commanded_tool_pose_theta_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'commanded_tool_pose_theta_z) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BaseFeedback>)))
  "Returns string type for a message object of type '<BaseFeedback>"
  "kortex_driver/BaseFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BaseFeedback)))
  "Returns string type for a message object of type 'BaseFeedback"
  "kortex_driver/BaseFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BaseFeedback>)))
  "Returns md5sum for a message object of type '<BaseFeedback>"
  "10860d64294ad841b25c45c5195aa64d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BaseFeedback)))
  "Returns md5sum for a message object of type 'BaseFeedback"
  "10860d64294ad841b25c45c5195aa64d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BaseFeedback>)))
  "Returns full string definition for message of type '<BaseFeedback>"
  (cl:format cl:nil "~%uint32 active_state_connection_identifier~%uint32 active_state~%float32 arm_voltage~%float32 arm_current~%float32 temperature_cpu~%float32 temperature_ambient~%float32 imu_acceleration_x~%float32 imu_acceleration_y~%float32 imu_acceleration_z~%float32 imu_angular_velocity_x~%float32 imu_angular_velocity_y~%float32 imu_angular_velocity_z~%float32 tool_pose_x~%float32 tool_pose_y~%float32 tool_pose_z~%float32 tool_pose_theta_x~%float32 tool_pose_theta_y~%float32 tool_pose_theta_z~%float32 tool_twist_linear_x~%float32 tool_twist_linear_y~%float32 tool_twist_linear_z~%float32 tool_twist_angular_x~%float32 tool_twist_angular_y~%float32 tool_twist_angular_z~%float32 tool_external_wrench_force_x~%float32 tool_external_wrench_force_y~%float32 tool_external_wrench_force_z~%float32 tool_external_wrench_torque_x~%float32 tool_external_wrench_torque_y~%float32 tool_external_wrench_torque_z~%uint32 fault_bank_a~%uint32 fault_bank_b~%uint32 warning_bank_a~%uint32 warning_bank_b~%float32 commanded_tool_pose_x~%float32 commanded_tool_pose_y~%float32 commanded_tool_pose_z~%float32 commanded_tool_pose_theta_x~%float32 commanded_tool_pose_theta_y~%float32 commanded_tool_pose_theta_z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BaseFeedback)))
  "Returns full string definition for message of type 'BaseFeedback"
  (cl:format cl:nil "~%uint32 active_state_connection_identifier~%uint32 active_state~%float32 arm_voltage~%float32 arm_current~%float32 temperature_cpu~%float32 temperature_ambient~%float32 imu_acceleration_x~%float32 imu_acceleration_y~%float32 imu_acceleration_z~%float32 imu_angular_velocity_x~%float32 imu_angular_velocity_y~%float32 imu_angular_velocity_z~%float32 tool_pose_x~%float32 tool_pose_y~%float32 tool_pose_z~%float32 tool_pose_theta_x~%float32 tool_pose_theta_y~%float32 tool_pose_theta_z~%float32 tool_twist_linear_x~%float32 tool_twist_linear_y~%float32 tool_twist_linear_z~%float32 tool_twist_angular_x~%float32 tool_twist_angular_y~%float32 tool_twist_angular_z~%float32 tool_external_wrench_force_x~%float32 tool_external_wrench_force_y~%float32 tool_external_wrench_force_z~%float32 tool_external_wrench_torque_x~%float32 tool_external_wrench_torque_y~%float32 tool_external_wrench_torque_z~%uint32 fault_bank_a~%uint32 fault_bank_b~%uint32 warning_bank_a~%uint32 warning_bank_b~%float32 commanded_tool_pose_x~%float32 commanded_tool_pose_y~%float32 commanded_tool_pose_z~%float32 commanded_tool_pose_theta_x~%float32 commanded_tool_pose_theta_y~%float32 commanded_tool_pose_theta_z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BaseFeedback>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BaseFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'BaseFeedback
    (cl:cons ':active_state_connection_identifier (active_state_connection_identifier msg))
    (cl:cons ':active_state (active_state msg))
    (cl:cons ':arm_voltage (arm_voltage msg))
    (cl:cons ':arm_current (arm_current msg))
    (cl:cons ':temperature_cpu (temperature_cpu msg))
    (cl:cons ':temperature_ambient (temperature_ambient msg))
    (cl:cons ':imu_acceleration_x (imu_acceleration_x msg))
    (cl:cons ':imu_acceleration_y (imu_acceleration_y msg))
    (cl:cons ':imu_acceleration_z (imu_acceleration_z msg))
    (cl:cons ':imu_angular_velocity_x (imu_angular_velocity_x msg))
    (cl:cons ':imu_angular_velocity_y (imu_angular_velocity_y msg))
    (cl:cons ':imu_angular_velocity_z (imu_angular_velocity_z msg))
    (cl:cons ':tool_pose_x (tool_pose_x msg))
    (cl:cons ':tool_pose_y (tool_pose_y msg))
    (cl:cons ':tool_pose_z (tool_pose_z msg))
    (cl:cons ':tool_pose_theta_x (tool_pose_theta_x msg))
    (cl:cons ':tool_pose_theta_y (tool_pose_theta_y msg))
    (cl:cons ':tool_pose_theta_z (tool_pose_theta_z msg))
    (cl:cons ':tool_twist_linear_x (tool_twist_linear_x msg))
    (cl:cons ':tool_twist_linear_y (tool_twist_linear_y msg))
    (cl:cons ':tool_twist_linear_z (tool_twist_linear_z msg))
    (cl:cons ':tool_twist_angular_x (tool_twist_angular_x msg))
    (cl:cons ':tool_twist_angular_y (tool_twist_angular_y msg))
    (cl:cons ':tool_twist_angular_z (tool_twist_angular_z msg))
    (cl:cons ':tool_external_wrench_force_x (tool_external_wrench_force_x msg))
    (cl:cons ':tool_external_wrench_force_y (tool_external_wrench_force_y msg))
    (cl:cons ':tool_external_wrench_force_z (tool_external_wrench_force_z msg))
    (cl:cons ':tool_external_wrench_torque_x (tool_external_wrench_torque_x msg))
    (cl:cons ':tool_external_wrench_torque_y (tool_external_wrench_torque_y msg))
    (cl:cons ':tool_external_wrench_torque_z (tool_external_wrench_torque_z msg))
    (cl:cons ':fault_bank_a (fault_bank_a msg))
    (cl:cons ':fault_bank_b (fault_bank_b msg))
    (cl:cons ':warning_bank_a (warning_bank_a msg))
    (cl:cons ':warning_bank_b (warning_bank_b msg))
    (cl:cons ':commanded_tool_pose_x (commanded_tool_pose_x msg))
    (cl:cons ':commanded_tool_pose_y (commanded_tool_pose_y msg))
    (cl:cons ':commanded_tool_pose_z (commanded_tool_pose_z msg))
    (cl:cons ':commanded_tool_pose_theta_x (commanded_tool_pose_theta_x msg))
    (cl:cons ':commanded_tool_pose_theta_y (commanded_tool_pose_theta_y msg))
    (cl:cons ':commanded_tool_pose_theta_z (commanded_tool_pose_theta_z msg))
))
