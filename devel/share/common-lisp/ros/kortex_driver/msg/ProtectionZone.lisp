; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ProtectionZone.msg.html

(cl:defclass <ProtectionZone> (roslisp-msg-protocol:ros-message)
  ((handle
    :reader handle
    :initarg :handle
    :type kortex_driver-msg:ProtectionZoneHandle
    :initform (cl:make-instance 'kortex_driver-msg:ProtectionZoneHandle))
   (name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (application_data
    :reader application_data
    :initarg :application_data
    :type cl:string
    :initform "")
   (is_enabled
    :reader is_enabled
    :initarg :is_enabled
    :type cl:boolean
    :initform cl:nil)
   (shape
    :reader shape
    :initarg :shape
    :type kortex_driver-msg:ZoneShape
    :initform (cl:make-instance 'kortex_driver-msg:ZoneShape))
   (limitations
    :reader limitations
    :initarg :limitations
    :type (cl:vector kortex_driver-msg:CartesianLimitation)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:CartesianLimitation :initial-element (cl:make-instance 'kortex_driver-msg:CartesianLimitation)))
   (envelope_limitations
    :reader envelope_limitations
    :initarg :envelope_limitations
    :type (cl:vector kortex_driver-msg:CartesianLimitation)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:CartesianLimitation :initial-element (cl:make-instance 'kortex_driver-msg:CartesianLimitation))))
)

(cl:defclass ProtectionZone (<ProtectionZone>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ProtectionZone>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ProtectionZone)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ProtectionZone> is deprecated: use kortex_driver-msg:ProtectionZone instead.")))

(cl:ensure-generic-function 'handle-val :lambda-list '(m))
(cl:defmethod handle-val ((m <ProtectionZone>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:handle-val is deprecated.  Use kortex_driver-msg:handle instead.")
  (handle m))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <ProtectionZone>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:name-val is deprecated.  Use kortex_driver-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'application_data-val :lambda-list '(m))
(cl:defmethod application_data-val ((m <ProtectionZone>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:application_data-val is deprecated.  Use kortex_driver-msg:application_data instead.")
  (application_data m))

(cl:ensure-generic-function 'is_enabled-val :lambda-list '(m))
(cl:defmethod is_enabled-val ((m <ProtectionZone>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:is_enabled-val is deprecated.  Use kortex_driver-msg:is_enabled instead.")
  (is_enabled m))

(cl:ensure-generic-function 'shape-val :lambda-list '(m))
(cl:defmethod shape-val ((m <ProtectionZone>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:shape-val is deprecated.  Use kortex_driver-msg:shape instead.")
  (shape m))

(cl:ensure-generic-function 'limitations-val :lambda-list '(m))
(cl:defmethod limitations-val ((m <ProtectionZone>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:limitations-val is deprecated.  Use kortex_driver-msg:limitations instead.")
  (limitations m))

(cl:ensure-generic-function 'envelope_limitations-val :lambda-list '(m))
(cl:defmethod envelope_limitations-val ((m <ProtectionZone>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:envelope_limitations-val is deprecated.  Use kortex_driver-msg:envelope_limitations instead.")
  (envelope_limitations m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ProtectionZone>) ostream)
  "Serializes a message object of type '<ProtectionZone>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'handle) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'application_data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'application_data))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_enabled) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'shape) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'limitations))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'limitations))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'envelope_limitations))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'envelope_limitations))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ProtectionZone>) istream)
  "Deserializes a message object of type '<ProtectionZone>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'handle) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'application_data) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'application_data) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'is_enabled) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'shape) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'limitations) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'limitations)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:CartesianLimitation))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'envelope_limitations) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'envelope_limitations)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:CartesianLimitation))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ProtectionZone>)))
  "Returns string type for a message object of type '<ProtectionZone>"
  "kortex_driver/ProtectionZone")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ProtectionZone)))
  "Returns string type for a message object of type 'ProtectionZone"
  "kortex_driver/ProtectionZone")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ProtectionZone>)))
  "Returns md5sum for a message object of type '<ProtectionZone>"
  "9648b3590ddbe012236cdc588d3f6ef8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ProtectionZone)))
  "Returns md5sum for a message object of type 'ProtectionZone"
  "9648b3590ddbe012236cdc588d3f6ef8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ProtectionZone>)))
  "Returns full string definition for message of type '<ProtectionZone>"
  (cl:format cl:nil "~%ProtectionZoneHandle handle~%string name~%string application_data~%bool is_enabled~%ZoneShape shape~%CartesianLimitation[] limitations~%CartesianLimitation[] envelope_limitations~%================================================================================~%MSG: kortex_driver/ProtectionZoneHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/ZoneShape~%~%uint32 shape_type~%Point origin~%Base_RotationMatrix orientation~%float32[] dimensions~%float32 envelope_thickness~%================================================================================~%MSG: kortex_driver/Point~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: kortex_driver/Base_RotationMatrix~%~%Base_RotationMatrixRow row1~%Base_RotationMatrixRow row2~%Base_RotationMatrixRow row3~%================================================================================~%MSG: kortex_driver/Base_RotationMatrixRow~%~%float32 column1~%float32 column2~%float32 column3~%================================================================================~%MSG: kortex_driver/CartesianLimitation~%~%uint32 type~%float32 translation~%float32 orientation~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ProtectionZone)))
  "Returns full string definition for message of type 'ProtectionZone"
  (cl:format cl:nil "~%ProtectionZoneHandle handle~%string name~%string application_data~%bool is_enabled~%ZoneShape shape~%CartesianLimitation[] limitations~%CartesianLimitation[] envelope_limitations~%================================================================================~%MSG: kortex_driver/ProtectionZoneHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/ZoneShape~%~%uint32 shape_type~%Point origin~%Base_RotationMatrix orientation~%float32[] dimensions~%float32 envelope_thickness~%================================================================================~%MSG: kortex_driver/Point~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: kortex_driver/Base_RotationMatrix~%~%Base_RotationMatrixRow row1~%Base_RotationMatrixRow row2~%Base_RotationMatrixRow row3~%================================================================================~%MSG: kortex_driver/Base_RotationMatrixRow~%~%float32 column1~%float32 column2~%float32 column3~%================================================================================~%MSG: kortex_driver/CartesianLimitation~%~%uint32 type~%float32 translation~%float32 orientation~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ProtectionZone>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'handle))
     4 (cl:length (cl:slot-value msg 'name))
     4 (cl:length (cl:slot-value msg 'application_data))
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'shape))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'limitations) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'envelope_limitations) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ProtectionZone>))
  "Converts a ROS message object to a list"
  (cl:list 'ProtectionZone
    (cl:cons ':handle (handle msg))
    (cl:cons ':name (name msg))
    (cl:cons ':application_data (application_data msg))
    (cl:cons ':is_enabled (is_enabled msg))
    (cl:cons ':shape (shape msg))
    (cl:cons ':limitations (limitations msg))
    (cl:cons ':envelope_limitations (envelope_limitations msg))
))
