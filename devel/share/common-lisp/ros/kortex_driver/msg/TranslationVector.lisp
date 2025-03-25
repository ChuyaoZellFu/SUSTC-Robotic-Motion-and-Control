; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude TranslationVector.msg.html

(cl:defclass <TranslationVector> (roslisp-msg-protocol:ros-message)
  ((t_x
    :reader t_x
    :initarg :t_x
    :type cl:float
    :initform 0.0)
   (t_y
    :reader t_y
    :initarg :t_y
    :type cl:float
    :initform 0.0)
   (t_z
    :reader t_z
    :initarg :t_z
    :type cl:float
    :initform 0.0))
)

(cl:defclass TranslationVector (<TranslationVector>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TranslationVector>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TranslationVector)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<TranslationVector> is deprecated: use kortex_driver-msg:TranslationVector instead.")))

(cl:ensure-generic-function 't_x-val :lambda-list '(m))
(cl:defmethod t_x-val ((m <TranslationVector>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:t_x-val is deprecated.  Use kortex_driver-msg:t_x instead.")
  (t_x m))

(cl:ensure-generic-function 't_y-val :lambda-list '(m))
(cl:defmethod t_y-val ((m <TranslationVector>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:t_y-val is deprecated.  Use kortex_driver-msg:t_y instead.")
  (t_y m))

(cl:ensure-generic-function 't_z-val :lambda-list '(m))
(cl:defmethod t_z-val ((m <TranslationVector>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:t_z-val is deprecated.  Use kortex_driver-msg:t_z instead.")
  (t_z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TranslationVector>) ostream)
  "Serializes a message object of type '<TranslationVector>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 't_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 't_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 't_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TranslationVector>) istream)
  "Deserializes a message object of type '<TranslationVector>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 't_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 't_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 't_z) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TranslationVector>)))
  "Returns string type for a message object of type '<TranslationVector>"
  "kortex_driver/TranslationVector")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TranslationVector)))
  "Returns string type for a message object of type 'TranslationVector"
  "kortex_driver/TranslationVector")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TranslationVector>)))
  "Returns md5sum for a message object of type '<TranslationVector>"
  "1c59f12cff6754b509028be4dd466647")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TranslationVector)))
  "Returns md5sum for a message object of type 'TranslationVector"
  "1c59f12cff6754b509028be4dd466647")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TranslationVector>)))
  "Returns full string definition for message of type '<TranslationVector>"
  (cl:format cl:nil "~%float32 t_x~%float32 t_y~%float32 t_z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TranslationVector)))
  "Returns full string definition for message of type 'TranslationVector"
  (cl:format cl:nil "~%float32 t_x~%float32 t_y~%float32 t_z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TranslationVector>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TranslationVector>))
  "Converts a ROS message object to a list"
  (cl:list 'TranslationVector
    (cl:cons ':t_x (t_x msg))
    (cl:cons ':t_y (t_y msg))
    (cl:cons ':t_z (t_z msg))
))
