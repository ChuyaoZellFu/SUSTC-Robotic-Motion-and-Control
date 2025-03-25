; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ZoneShape.msg.html

(cl:defclass <ZoneShape> (roslisp-msg-protocol:ros-message)
  ((shape_type
    :reader shape_type
    :initarg :shape_type
    :type cl:integer
    :initform 0)
   (origin
    :reader origin
    :initarg :origin
    :type kortex_driver-msg:Point
    :initform (cl:make-instance 'kortex_driver-msg:Point))
   (orientation
    :reader orientation
    :initarg :orientation
    :type kortex_driver-msg:Base_RotationMatrix
    :initform (cl:make-instance 'kortex_driver-msg:Base_RotationMatrix))
   (dimensions
    :reader dimensions
    :initarg :dimensions
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (envelope_thickness
    :reader envelope_thickness
    :initarg :envelope_thickness
    :type cl:float
    :initform 0.0))
)

(cl:defclass ZoneShape (<ZoneShape>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ZoneShape>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ZoneShape)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ZoneShape> is deprecated: use kortex_driver-msg:ZoneShape instead.")))

(cl:ensure-generic-function 'shape_type-val :lambda-list '(m))
(cl:defmethod shape_type-val ((m <ZoneShape>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:shape_type-val is deprecated.  Use kortex_driver-msg:shape_type instead.")
  (shape_type m))

(cl:ensure-generic-function 'origin-val :lambda-list '(m))
(cl:defmethod origin-val ((m <ZoneShape>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:origin-val is deprecated.  Use kortex_driver-msg:origin instead.")
  (origin m))

(cl:ensure-generic-function 'orientation-val :lambda-list '(m))
(cl:defmethod orientation-val ((m <ZoneShape>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:orientation-val is deprecated.  Use kortex_driver-msg:orientation instead.")
  (orientation m))

(cl:ensure-generic-function 'dimensions-val :lambda-list '(m))
(cl:defmethod dimensions-val ((m <ZoneShape>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:dimensions-val is deprecated.  Use kortex_driver-msg:dimensions instead.")
  (dimensions m))

(cl:ensure-generic-function 'envelope_thickness-val :lambda-list '(m))
(cl:defmethod envelope_thickness-val ((m <ZoneShape>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:envelope_thickness-val is deprecated.  Use kortex_driver-msg:envelope_thickness instead.")
  (envelope_thickness m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ZoneShape>) ostream)
  "Serializes a message object of type '<ZoneShape>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'shape_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'shape_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'shape_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'shape_type)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'origin) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'orientation) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'dimensions))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'dimensions))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'envelope_thickness))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ZoneShape>) istream)
  "Deserializes a message object of type '<ZoneShape>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'shape_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'shape_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'shape_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'shape_type)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'origin) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'orientation) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'dimensions) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'dimensions)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'envelope_thickness) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ZoneShape>)))
  "Returns string type for a message object of type '<ZoneShape>"
  "kortex_driver/ZoneShape")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ZoneShape)))
  "Returns string type for a message object of type 'ZoneShape"
  "kortex_driver/ZoneShape")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ZoneShape>)))
  "Returns md5sum for a message object of type '<ZoneShape>"
  "8de31189535cb06cb8caf5fc49d6c1b4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ZoneShape)))
  "Returns md5sum for a message object of type 'ZoneShape"
  "8de31189535cb06cb8caf5fc49d6c1b4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ZoneShape>)))
  "Returns full string definition for message of type '<ZoneShape>"
  (cl:format cl:nil "~%uint32 shape_type~%Point origin~%Base_RotationMatrix orientation~%float32[] dimensions~%float32 envelope_thickness~%================================================================================~%MSG: kortex_driver/Point~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: kortex_driver/Base_RotationMatrix~%~%Base_RotationMatrixRow row1~%Base_RotationMatrixRow row2~%Base_RotationMatrixRow row3~%================================================================================~%MSG: kortex_driver/Base_RotationMatrixRow~%~%float32 column1~%float32 column2~%float32 column3~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ZoneShape)))
  "Returns full string definition for message of type 'ZoneShape"
  (cl:format cl:nil "~%uint32 shape_type~%Point origin~%Base_RotationMatrix orientation~%float32[] dimensions~%float32 envelope_thickness~%================================================================================~%MSG: kortex_driver/Point~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: kortex_driver/Base_RotationMatrix~%~%Base_RotationMatrixRow row1~%Base_RotationMatrixRow row2~%Base_RotationMatrixRow row3~%================================================================================~%MSG: kortex_driver/Base_RotationMatrixRow~%~%float32 column1~%float32 column2~%float32 column3~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ZoneShape>))
  (cl:+ 0
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'origin))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'orientation))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'dimensions) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ZoneShape>))
  "Converts a ROS message object to a list"
  (cl:list 'ZoneShape
    (cl:cons ':shape_type (shape_type msg))
    (cl:cons ':origin (origin msg))
    (cl:cons ':orientation (orientation msg))
    (cl:cons ':dimensions (dimensions msg))
    (cl:cons ':envelope_thickness (envelope_thickness msg))
))
