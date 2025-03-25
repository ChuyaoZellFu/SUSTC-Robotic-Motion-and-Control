; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude TwistAngularSoftLimit.msg.html

(cl:defclass <TwistAngularSoftLimit> (roslisp-msg-protocol:ros-message)
  ((control_mode
    :reader control_mode
    :initarg :control_mode
    :type cl:integer
    :initform 0)
   (twist_angular_soft_limit
    :reader twist_angular_soft_limit
    :initarg :twist_angular_soft_limit
    :type cl:float
    :initform 0.0))
)

(cl:defclass TwistAngularSoftLimit (<TwistAngularSoftLimit>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TwistAngularSoftLimit>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TwistAngularSoftLimit)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<TwistAngularSoftLimit> is deprecated: use kortex_driver-msg:TwistAngularSoftLimit instead.")))

(cl:ensure-generic-function 'control_mode-val :lambda-list '(m))
(cl:defmethod control_mode-val ((m <TwistAngularSoftLimit>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:control_mode-val is deprecated.  Use kortex_driver-msg:control_mode instead.")
  (control_mode m))

(cl:ensure-generic-function 'twist_angular_soft_limit-val :lambda-list '(m))
(cl:defmethod twist_angular_soft_limit-val ((m <TwistAngularSoftLimit>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:twist_angular_soft_limit-val is deprecated.  Use kortex_driver-msg:twist_angular_soft_limit instead.")
  (twist_angular_soft_limit m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TwistAngularSoftLimit>) ostream)
  "Serializes a message object of type '<TwistAngularSoftLimit>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'control_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'control_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'control_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'control_mode)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'twist_angular_soft_limit))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TwistAngularSoftLimit>) istream)
  "Deserializes a message object of type '<TwistAngularSoftLimit>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'control_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'control_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'control_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'control_mode)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'twist_angular_soft_limit) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TwistAngularSoftLimit>)))
  "Returns string type for a message object of type '<TwistAngularSoftLimit>"
  "kortex_driver/TwistAngularSoftLimit")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TwistAngularSoftLimit)))
  "Returns string type for a message object of type 'TwistAngularSoftLimit"
  "kortex_driver/TwistAngularSoftLimit")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TwistAngularSoftLimit>)))
  "Returns md5sum for a message object of type '<TwistAngularSoftLimit>"
  "ec20d4ed4983e99d6172917731a2eee5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TwistAngularSoftLimit)))
  "Returns md5sum for a message object of type 'TwistAngularSoftLimit"
  "ec20d4ed4983e99d6172917731a2eee5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TwistAngularSoftLimit>)))
  "Returns full string definition for message of type '<TwistAngularSoftLimit>"
  (cl:format cl:nil "~%uint32 control_mode~%float32 twist_angular_soft_limit~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TwistAngularSoftLimit)))
  "Returns full string definition for message of type 'TwistAngularSoftLimit"
  (cl:format cl:nil "~%uint32 control_mode~%float32 twist_angular_soft_limit~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TwistAngularSoftLimit>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TwistAngularSoftLimit>))
  "Converts a ROS message object to a list"
  (cl:list 'TwistAngularSoftLimit
    (cl:cons ':control_mode (control_mode msg))
    (cl:cons ':twist_angular_soft_limit (twist_angular_soft_limit msg))
))
