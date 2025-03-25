; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude BaseCyclic_CustomData.msg.html

(cl:defclass <BaseCyclic_CustomData> (roslisp-msg-protocol:ros-message)
  ((frame_id
    :reader frame_id
    :initarg :frame_id
    :type cl:integer
    :initform 0)
   (custom_data_0
    :reader custom_data_0
    :initarg :custom_data_0
    :type cl:integer
    :initform 0)
   (custom_data_1
    :reader custom_data_1
    :initarg :custom_data_1
    :type cl:integer
    :initform 0)
   (custom_data_2
    :reader custom_data_2
    :initarg :custom_data_2
    :type cl:integer
    :initform 0)
   (custom_data_3
    :reader custom_data_3
    :initarg :custom_data_3
    :type cl:integer
    :initform 0)
   (custom_data_4
    :reader custom_data_4
    :initarg :custom_data_4
    :type cl:integer
    :initform 0)
   (custom_data_5
    :reader custom_data_5
    :initarg :custom_data_5
    :type cl:integer
    :initform 0)
   (custom_data_6
    :reader custom_data_6
    :initarg :custom_data_6
    :type cl:integer
    :initform 0)
   (custom_data_7
    :reader custom_data_7
    :initarg :custom_data_7
    :type cl:integer
    :initform 0)
   (actuators_custom_data
    :reader actuators_custom_data
    :initarg :actuators_custom_data
    :type (cl:vector kortex_driver-msg:ActuatorCustomData)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:ActuatorCustomData :initial-element (cl:make-instance 'kortex_driver-msg:ActuatorCustomData)))
   (interconnect_custom_data
    :reader interconnect_custom_data
    :initarg :interconnect_custom_data
    :type kortex_driver-msg:InterconnectCyclic_CustomData
    :initform (cl:make-instance 'kortex_driver-msg:InterconnectCyclic_CustomData)))
)

(cl:defclass BaseCyclic_CustomData (<BaseCyclic_CustomData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BaseCyclic_CustomData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BaseCyclic_CustomData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<BaseCyclic_CustomData> is deprecated: use kortex_driver-msg:BaseCyclic_CustomData instead.")))

(cl:ensure-generic-function 'frame_id-val :lambda-list '(m))
(cl:defmethod frame_id-val ((m <BaseCyclic_CustomData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:frame_id-val is deprecated.  Use kortex_driver-msg:frame_id instead.")
  (frame_id m))

(cl:ensure-generic-function 'custom_data_0-val :lambda-list '(m))
(cl:defmethod custom_data_0-val ((m <BaseCyclic_CustomData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:custom_data_0-val is deprecated.  Use kortex_driver-msg:custom_data_0 instead.")
  (custom_data_0 m))

(cl:ensure-generic-function 'custom_data_1-val :lambda-list '(m))
(cl:defmethod custom_data_1-val ((m <BaseCyclic_CustomData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:custom_data_1-val is deprecated.  Use kortex_driver-msg:custom_data_1 instead.")
  (custom_data_1 m))

(cl:ensure-generic-function 'custom_data_2-val :lambda-list '(m))
(cl:defmethod custom_data_2-val ((m <BaseCyclic_CustomData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:custom_data_2-val is deprecated.  Use kortex_driver-msg:custom_data_2 instead.")
  (custom_data_2 m))

(cl:ensure-generic-function 'custom_data_3-val :lambda-list '(m))
(cl:defmethod custom_data_3-val ((m <BaseCyclic_CustomData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:custom_data_3-val is deprecated.  Use kortex_driver-msg:custom_data_3 instead.")
  (custom_data_3 m))

(cl:ensure-generic-function 'custom_data_4-val :lambda-list '(m))
(cl:defmethod custom_data_4-val ((m <BaseCyclic_CustomData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:custom_data_4-val is deprecated.  Use kortex_driver-msg:custom_data_4 instead.")
  (custom_data_4 m))

(cl:ensure-generic-function 'custom_data_5-val :lambda-list '(m))
(cl:defmethod custom_data_5-val ((m <BaseCyclic_CustomData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:custom_data_5-val is deprecated.  Use kortex_driver-msg:custom_data_5 instead.")
  (custom_data_5 m))

(cl:ensure-generic-function 'custom_data_6-val :lambda-list '(m))
(cl:defmethod custom_data_6-val ((m <BaseCyclic_CustomData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:custom_data_6-val is deprecated.  Use kortex_driver-msg:custom_data_6 instead.")
  (custom_data_6 m))

(cl:ensure-generic-function 'custom_data_7-val :lambda-list '(m))
(cl:defmethod custom_data_7-val ((m <BaseCyclic_CustomData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:custom_data_7-val is deprecated.  Use kortex_driver-msg:custom_data_7 instead.")
  (custom_data_7 m))

(cl:ensure-generic-function 'actuators_custom_data-val :lambda-list '(m))
(cl:defmethod actuators_custom_data-val ((m <BaseCyclic_CustomData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:actuators_custom_data-val is deprecated.  Use kortex_driver-msg:actuators_custom_data instead.")
  (actuators_custom_data m))

(cl:ensure-generic-function 'interconnect_custom_data-val :lambda-list '(m))
(cl:defmethod interconnect_custom_data-val ((m <BaseCyclic_CustomData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:interconnect_custom_data-val is deprecated.  Use kortex_driver-msg:interconnect_custom_data instead.")
  (interconnect_custom_data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BaseCyclic_CustomData>) ostream)
  "Serializes a message object of type '<BaseCyclic_CustomData>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'frame_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'frame_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'frame_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'frame_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_3)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_3)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_3)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_3)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_4)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_4)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_4)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_4)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_5)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_5)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_5)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_5)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_6)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_6)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_6)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_6)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_7)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_7)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_7)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_7)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'actuators_custom_data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'actuators_custom_data))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'interconnect_custom_data) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BaseCyclic_CustomData>) istream)
  "Deserializes a message object of type '<BaseCyclic_CustomData>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'frame_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'frame_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'frame_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'frame_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_0)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_0)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_0)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_0)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_3)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_3)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_3)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_3)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_4)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_4)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_4)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_4)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_5)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_5)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_5)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_5)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_6)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_6)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_6)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_6)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_7)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_7)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_7)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_7)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'actuators_custom_data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'actuators_custom_data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:ActuatorCustomData))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'interconnect_custom_data) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BaseCyclic_CustomData>)))
  "Returns string type for a message object of type '<BaseCyclic_CustomData>"
  "kortex_driver/BaseCyclic_CustomData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BaseCyclic_CustomData)))
  "Returns string type for a message object of type 'BaseCyclic_CustomData"
  "kortex_driver/BaseCyclic_CustomData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BaseCyclic_CustomData>)))
  "Returns md5sum for a message object of type '<BaseCyclic_CustomData>"
  "de86e2c250eb6e05b62cbbd55c2a087b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BaseCyclic_CustomData)))
  "Returns md5sum for a message object of type 'BaseCyclic_CustomData"
  "de86e2c250eb6e05b62cbbd55c2a087b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BaseCyclic_CustomData>)))
  "Returns full string definition for message of type '<BaseCyclic_CustomData>"
  (cl:format cl:nil "~%uint32 frame_id~%uint32 custom_data_0~%uint32 custom_data_1~%uint32 custom_data_2~%uint32 custom_data_3~%uint32 custom_data_4~%uint32 custom_data_5~%uint32 custom_data_6~%uint32 custom_data_7~%ActuatorCustomData[] actuators_custom_data~%InterconnectCyclic_CustomData interconnect_custom_data~%================================================================================~%MSG: kortex_driver/ActuatorCustomData~%~%uint32 command_id~%uint32 custom_data_0~%uint32 custom_data_1~%uint32 custom_data_2~%uint32 custom_data_3~%uint32 custom_data_4~%uint32 custom_data_5~%uint32 custom_data_6~%uint32 custom_data_7~%uint32 custom_data_8~%uint32 custom_data_9~%uint32 custom_data_10~%uint32 custom_data_11~%uint32 custom_data_12~%uint32 custom_data_13~%uint32 custom_data_14~%uint32 custom_data_15~%================================================================================~%MSG: kortex_driver/InterconnectCyclic_CustomData~%~%InterconnectCyclic_MessageId custom_data_id~%uint32 custom_data_0~%uint32 custom_data_1~%uint32 custom_data_2~%uint32 custom_data_3~%uint32 custom_data_4~%uint32 custom_data_5~%uint32 custom_data_6~%uint32 custom_data_7~%uint32 custom_data_8~%uint32 custom_data_9~%uint32 custom_data_10~%uint32 custom_data_11~%uint32 custom_data_12~%uint32 custom_data_13~%uint32 custom_data_14~%uint32 custom_data_15~%InterconnectCyclic_CustomData_tool_customData oneof_tool_customData~%================================================================================~%MSG: kortex_driver/InterconnectCyclic_MessageId~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/InterconnectCyclic_CustomData_tool_customData~%~%GripperCyclic_CustomData[] gripper_custom_data~%================================================================================~%MSG: kortex_driver/GripperCyclic_CustomData~%~%GripperCyclic_MessageId custom_data_id~%CustomDataUnit gripper_custom_data~%CustomDataUnit[] motor_custom_data~%================================================================================~%MSG: kortex_driver/GripperCyclic_MessageId~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/CustomDataUnit~%~%uint32 custom_data_0~%uint32 custom_data_1~%uint32 custom_data_2~%uint32 custom_data_3~%uint32 custom_data_4~%uint32 custom_data_5~%uint32 custom_data_6~%uint32 custom_data_7~%uint32 custom_data_8~%uint32 custom_data_9~%uint32 custom_data_10~%uint32 custom_data_11~%uint32 custom_data_12~%uint32 custom_data_13~%uint32 custom_data_14~%uint32 custom_data_15~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BaseCyclic_CustomData)))
  "Returns full string definition for message of type 'BaseCyclic_CustomData"
  (cl:format cl:nil "~%uint32 frame_id~%uint32 custom_data_0~%uint32 custom_data_1~%uint32 custom_data_2~%uint32 custom_data_3~%uint32 custom_data_4~%uint32 custom_data_5~%uint32 custom_data_6~%uint32 custom_data_7~%ActuatorCustomData[] actuators_custom_data~%InterconnectCyclic_CustomData interconnect_custom_data~%================================================================================~%MSG: kortex_driver/ActuatorCustomData~%~%uint32 command_id~%uint32 custom_data_0~%uint32 custom_data_1~%uint32 custom_data_2~%uint32 custom_data_3~%uint32 custom_data_4~%uint32 custom_data_5~%uint32 custom_data_6~%uint32 custom_data_7~%uint32 custom_data_8~%uint32 custom_data_9~%uint32 custom_data_10~%uint32 custom_data_11~%uint32 custom_data_12~%uint32 custom_data_13~%uint32 custom_data_14~%uint32 custom_data_15~%================================================================================~%MSG: kortex_driver/InterconnectCyclic_CustomData~%~%InterconnectCyclic_MessageId custom_data_id~%uint32 custom_data_0~%uint32 custom_data_1~%uint32 custom_data_2~%uint32 custom_data_3~%uint32 custom_data_4~%uint32 custom_data_5~%uint32 custom_data_6~%uint32 custom_data_7~%uint32 custom_data_8~%uint32 custom_data_9~%uint32 custom_data_10~%uint32 custom_data_11~%uint32 custom_data_12~%uint32 custom_data_13~%uint32 custom_data_14~%uint32 custom_data_15~%InterconnectCyclic_CustomData_tool_customData oneof_tool_customData~%================================================================================~%MSG: kortex_driver/InterconnectCyclic_MessageId~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/InterconnectCyclic_CustomData_tool_customData~%~%GripperCyclic_CustomData[] gripper_custom_data~%================================================================================~%MSG: kortex_driver/GripperCyclic_CustomData~%~%GripperCyclic_MessageId custom_data_id~%CustomDataUnit gripper_custom_data~%CustomDataUnit[] motor_custom_data~%================================================================================~%MSG: kortex_driver/GripperCyclic_MessageId~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/CustomDataUnit~%~%uint32 custom_data_0~%uint32 custom_data_1~%uint32 custom_data_2~%uint32 custom_data_3~%uint32 custom_data_4~%uint32 custom_data_5~%uint32 custom_data_6~%uint32 custom_data_7~%uint32 custom_data_8~%uint32 custom_data_9~%uint32 custom_data_10~%uint32 custom_data_11~%uint32 custom_data_12~%uint32 custom_data_13~%uint32 custom_data_14~%uint32 custom_data_15~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BaseCyclic_CustomData>))
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
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'actuators_custom_data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'interconnect_custom_data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BaseCyclic_CustomData>))
  "Converts a ROS message object to a list"
  (cl:list 'BaseCyclic_CustomData
    (cl:cons ':frame_id (frame_id msg))
    (cl:cons ':custom_data_0 (custom_data_0 msg))
    (cl:cons ':custom_data_1 (custom_data_1 msg))
    (cl:cons ':custom_data_2 (custom_data_2 msg))
    (cl:cons ':custom_data_3 (custom_data_3 msg))
    (cl:cons ':custom_data_4 (custom_data_4 msg))
    (cl:cons ':custom_data_5 (custom_data_5 msg))
    (cl:cons ':custom_data_6 (custom_data_6 msg))
    (cl:cons ':custom_data_7 (custom_data_7 msg))
    (cl:cons ':actuators_custom_data (actuators_custom_data msg))
    (cl:cons ':interconnect_custom_data (interconnect_custom_data msg))
))
