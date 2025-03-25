; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude Wrench.msg.html

(cl:defclass <Wrench> (roslisp-msg-protocol:ros-message)
  ((force_x
    :reader force_x
    :initarg :force_x
    :type cl:float
    :initform 0.0)
   (force_y
    :reader force_y
    :initarg :force_y
    :type cl:float
    :initform 0.0)
   (force_z
    :reader force_z
    :initarg :force_z
    :type cl:float
    :initform 0.0)
   (torque_x
    :reader torque_x
    :initarg :torque_x
    :type cl:float
    :initform 0.0)
   (torque_y
    :reader torque_y
    :initarg :torque_y
    :type cl:float
    :initform 0.0)
   (torque_z
    :reader torque_z
    :initarg :torque_z
    :type cl:float
    :initform 0.0))
)

(cl:defclass Wrench (<Wrench>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Wrench>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Wrench)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<Wrench> is deprecated: use kortex_driver-msg:Wrench instead.")))

(cl:ensure-generic-function 'force_x-val :lambda-list '(m))
(cl:defmethod force_x-val ((m <Wrench>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:force_x-val is deprecated.  Use kortex_driver-msg:force_x instead.")
  (force_x m))

(cl:ensure-generic-function 'force_y-val :lambda-list '(m))
(cl:defmethod force_y-val ((m <Wrench>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:force_y-val is deprecated.  Use kortex_driver-msg:force_y instead.")
  (force_y m))

(cl:ensure-generic-function 'force_z-val :lambda-list '(m))
(cl:defmethod force_z-val ((m <Wrench>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:force_z-val is deprecated.  Use kortex_driver-msg:force_z instead.")
  (force_z m))

(cl:ensure-generic-function 'torque_x-val :lambda-list '(m))
(cl:defmethod torque_x-val ((m <Wrench>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:torque_x-val is deprecated.  Use kortex_driver-msg:torque_x instead.")
  (torque_x m))

(cl:ensure-generic-function 'torque_y-val :lambda-list '(m))
(cl:defmethod torque_y-val ((m <Wrench>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:torque_y-val is deprecated.  Use kortex_driver-msg:torque_y instead.")
  (torque_y m))

(cl:ensure-generic-function 'torque_z-val :lambda-list '(m))
(cl:defmethod torque_z-val ((m <Wrench>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:torque_z-val is deprecated.  Use kortex_driver-msg:torque_z instead.")
  (torque_z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Wrench>) ostream)
  "Serializes a message object of type '<Wrench>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'force_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'force_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'force_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'torque_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'torque_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'torque_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Wrench>) istream)
  "Deserializes a message object of type '<Wrench>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'force_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'force_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'force_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'torque_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'torque_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'torque_z) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Wrench>)))
  "Returns string type for a message object of type '<Wrench>"
  "kortex_driver/Wrench")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Wrench)))
  "Returns string type for a message object of type 'Wrench"
  "kortex_driver/Wrench")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Wrench>)))
  "Returns md5sum for a message object of type '<Wrench>"
  "b01974557c40b776cdb7003057779989")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Wrench)))
  "Returns md5sum for a message object of type 'Wrench"
  "b01974557c40b776cdb7003057779989")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Wrench>)))
  "Returns full string definition for message of type '<Wrench>"
  (cl:format cl:nil "~%float32 force_x~%float32 force_y~%float32 force_z~%float32 torque_x~%float32 torque_y~%float32 torque_z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Wrench)))
  "Returns full string definition for message of type 'Wrench"
  (cl:format cl:nil "~%float32 force_x~%float32 force_y~%float32 force_z~%float32 torque_x~%float32 torque_y~%float32 torque_z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Wrench>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Wrench>))
  "Converts a ROS message object to a list"
  (cl:list 'Wrench
    (cl:cons ':force_x (force_x msg))
    (cl:cons ':force_y (force_y msg))
    (cl:cons ':force_z (force_z msg))
    (cl:cons ':torque_x (torque_x msg))
    (cl:cons ':torque_y (torque_y msg))
    (cl:cons ':torque_z (torque_z msg))
))
