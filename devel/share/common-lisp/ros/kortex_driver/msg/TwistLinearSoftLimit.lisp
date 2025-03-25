; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude TwistLinearSoftLimit.msg.html

(cl:defclass <TwistLinearSoftLimit> (roslisp-msg-protocol:ros-message)
  ((control_mode
    :reader control_mode
    :initarg :control_mode
    :type cl:integer
    :initform 0)
   (twist_linear_soft_limit
    :reader twist_linear_soft_limit
    :initarg :twist_linear_soft_limit
    :type cl:float
    :initform 0.0))
)

(cl:defclass TwistLinearSoftLimit (<TwistLinearSoftLimit>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TwistLinearSoftLimit>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TwistLinearSoftLimit)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<TwistLinearSoftLimit> is deprecated: use kortex_driver-msg:TwistLinearSoftLimit instead.")))

(cl:ensure-generic-function 'control_mode-val :lambda-list '(m))
(cl:defmethod control_mode-val ((m <TwistLinearSoftLimit>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:control_mode-val is deprecated.  Use kortex_driver-msg:control_mode instead.")
  (control_mode m))

(cl:ensure-generic-function 'twist_linear_soft_limit-val :lambda-list '(m))
(cl:defmethod twist_linear_soft_limit-val ((m <TwistLinearSoftLimit>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:twist_linear_soft_limit-val is deprecated.  Use kortex_driver-msg:twist_linear_soft_limit instead.")
  (twist_linear_soft_limit m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TwistLinearSoftLimit>) ostream)
  "Serializes a message object of type '<TwistLinearSoftLimit>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'control_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'control_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'control_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'control_mode)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'twist_linear_soft_limit))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TwistLinearSoftLimit>) istream)
  "Deserializes a message object of type '<TwistLinearSoftLimit>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'control_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'control_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'control_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'control_mode)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'twist_linear_soft_limit) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TwistLinearSoftLimit>)))
  "Returns string type for a message object of type '<TwistLinearSoftLimit>"
  "kortex_driver/TwistLinearSoftLimit")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TwistLinearSoftLimit)))
  "Returns string type for a message object of type 'TwistLinearSoftLimit"
  "kortex_driver/TwistLinearSoftLimit")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TwistLinearSoftLimit>)))
  "Returns md5sum for a message object of type '<TwistLinearSoftLimit>"
  "56b2076ba48a21a3287444da3b0c4e3c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TwistLinearSoftLimit)))
  "Returns md5sum for a message object of type 'TwistLinearSoftLimit"
  "56b2076ba48a21a3287444da3b0c4e3c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TwistLinearSoftLimit>)))
  "Returns full string definition for message of type '<TwistLinearSoftLimit>"
  (cl:format cl:nil "~%uint32 control_mode~%float32 twist_linear_soft_limit~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TwistLinearSoftLimit)))
  "Returns full string definition for message of type 'TwistLinearSoftLimit"
  (cl:format cl:nil "~%uint32 control_mode~%float32 twist_linear_soft_limit~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TwistLinearSoftLimit>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TwistLinearSoftLimit>))
  "Converts a ROS message object to a list"
  (cl:list 'TwistLinearSoftLimit
    (cl:cons ':control_mode (control_mode msg))
    (cl:cons ':twist_linear_soft_limit (twist_linear_soft_limit msg))
))
