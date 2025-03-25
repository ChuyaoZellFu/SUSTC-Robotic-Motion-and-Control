; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ProtectionZoneList.msg.html

(cl:defclass <ProtectionZoneList> (roslisp-msg-protocol:ros-message)
  ((protection_zones
    :reader protection_zones
    :initarg :protection_zones
    :type (cl:vector kortex_driver-msg:ProtectionZone)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:ProtectionZone :initial-element (cl:make-instance 'kortex_driver-msg:ProtectionZone))))
)

(cl:defclass ProtectionZoneList (<ProtectionZoneList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ProtectionZoneList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ProtectionZoneList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ProtectionZoneList> is deprecated: use kortex_driver-msg:ProtectionZoneList instead.")))

(cl:ensure-generic-function 'protection_zones-val :lambda-list '(m))
(cl:defmethod protection_zones-val ((m <ProtectionZoneList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:protection_zones-val is deprecated.  Use kortex_driver-msg:protection_zones instead.")
  (protection_zones m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ProtectionZoneList>) ostream)
  "Serializes a message object of type '<ProtectionZoneList>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'protection_zones))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'protection_zones))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ProtectionZoneList>) istream)
  "Deserializes a message object of type '<ProtectionZoneList>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'protection_zones) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'protection_zones)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:ProtectionZone))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ProtectionZoneList>)))
  "Returns string type for a message object of type '<ProtectionZoneList>"
  "kortex_driver/ProtectionZoneList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ProtectionZoneList)))
  "Returns string type for a message object of type 'ProtectionZoneList"
  "kortex_driver/ProtectionZoneList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ProtectionZoneList>)))
  "Returns md5sum for a message object of type '<ProtectionZoneList>"
  "a08108637290aeb7551436f88ea3c88f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ProtectionZoneList)))
  "Returns md5sum for a message object of type 'ProtectionZoneList"
  "a08108637290aeb7551436f88ea3c88f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ProtectionZoneList>)))
  "Returns full string definition for message of type '<ProtectionZoneList>"
  (cl:format cl:nil "~%ProtectionZone[] protection_zones~%================================================================================~%MSG: kortex_driver/ProtectionZone~%~%ProtectionZoneHandle handle~%string name~%string application_data~%bool is_enabled~%ZoneShape shape~%CartesianLimitation[] limitations~%CartesianLimitation[] envelope_limitations~%================================================================================~%MSG: kortex_driver/ProtectionZoneHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/ZoneShape~%~%uint32 shape_type~%Point origin~%Base_RotationMatrix orientation~%float32[] dimensions~%float32 envelope_thickness~%================================================================================~%MSG: kortex_driver/Point~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: kortex_driver/Base_RotationMatrix~%~%Base_RotationMatrixRow row1~%Base_RotationMatrixRow row2~%Base_RotationMatrixRow row3~%================================================================================~%MSG: kortex_driver/Base_RotationMatrixRow~%~%float32 column1~%float32 column2~%float32 column3~%================================================================================~%MSG: kortex_driver/CartesianLimitation~%~%uint32 type~%float32 translation~%float32 orientation~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ProtectionZoneList)))
  "Returns full string definition for message of type 'ProtectionZoneList"
  (cl:format cl:nil "~%ProtectionZone[] protection_zones~%================================================================================~%MSG: kortex_driver/ProtectionZone~%~%ProtectionZoneHandle handle~%string name~%string application_data~%bool is_enabled~%ZoneShape shape~%CartesianLimitation[] limitations~%CartesianLimitation[] envelope_limitations~%================================================================================~%MSG: kortex_driver/ProtectionZoneHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/ZoneShape~%~%uint32 shape_type~%Point origin~%Base_RotationMatrix orientation~%float32[] dimensions~%float32 envelope_thickness~%================================================================================~%MSG: kortex_driver/Point~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: kortex_driver/Base_RotationMatrix~%~%Base_RotationMatrixRow row1~%Base_RotationMatrixRow row2~%Base_RotationMatrixRow row3~%================================================================================~%MSG: kortex_driver/Base_RotationMatrixRow~%~%float32 column1~%float32 column2~%float32 column3~%================================================================================~%MSG: kortex_driver/CartesianLimitation~%~%uint32 type~%float32 translation~%float32 orientation~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ProtectionZoneList>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'protection_zones) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ProtectionZoneList>))
  "Converts a ROS message object to a list"
  (cl:list 'ProtectionZoneList
    (cl:cons ':protection_zones (protection_zones msg))
))
