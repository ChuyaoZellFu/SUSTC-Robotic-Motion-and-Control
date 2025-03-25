; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude EncoderDerivativeParameters.msg.html

(cl:defclass <EncoderDerivativeParameters> (roslisp-msg-protocol:ros-message)
  ((max_window_width
    :reader max_window_width
    :initarg :max_window_width
    :type cl:integer
    :initform 0)
   (min_angle
    :reader min_angle
    :initarg :min_angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass EncoderDerivativeParameters (<EncoderDerivativeParameters>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EncoderDerivativeParameters>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EncoderDerivativeParameters)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<EncoderDerivativeParameters> is deprecated: use kortex_driver-msg:EncoderDerivativeParameters instead.")))

(cl:ensure-generic-function 'max_window_width-val :lambda-list '(m))
(cl:defmethod max_window_width-val ((m <EncoderDerivativeParameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:max_window_width-val is deprecated.  Use kortex_driver-msg:max_window_width instead.")
  (max_window_width m))

(cl:ensure-generic-function 'min_angle-val :lambda-list '(m))
(cl:defmethod min_angle-val ((m <EncoderDerivativeParameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:min_angle-val is deprecated.  Use kortex_driver-msg:min_angle instead.")
  (min_angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EncoderDerivativeParameters>) ostream)
  "Serializes a message object of type '<EncoderDerivativeParameters>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'max_window_width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'max_window_width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'max_window_width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'max_window_width)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'min_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EncoderDerivativeParameters>) istream)
  "Deserializes a message object of type '<EncoderDerivativeParameters>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'max_window_width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'max_window_width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'max_window_width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'max_window_width)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'min_angle) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EncoderDerivativeParameters>)))
  "Returns string type for a message object of type '<EncoderDerivativeParameters>"
  "kortex_driver/EncoderDerivativeParameters")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EncoderDerivativeParameters)))
  "Returns string type for a message object of type 'EncoderDerivativeParameters"
  "kortex_driver/EncoderDerivativeParameters")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EncoderDerivativeParameters>)))
  "Returns md5sum for a message object of type '<EncoderDerivativeParameters>"
  "4e0a5e3a584720a2550104b5e1faa434")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EncoderDerivativeParameters)))
  "Returns md5sum for a message object of type 'EncoderDerivativeParameters"
  "4e0a5e3a584720a2550104b5e1faa434")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EncoderDerivativeParameters>)))
  "Returns full string definition for message of type '<EncoderDerivativeParameters>"
  (cl:format cl:nil "~%uint32 max_window_width~%float32 min_angle~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EncoderDerivativeParameters)))
  "Returns full string definition for message of type 'EncoderDerivativeParameters"
  (cl:format cl:nil "~%uint32 max_window_width~%float32 min_angle~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EncoderDerivativeParameters>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EncoderDerivativeParameters>))
  "Converts a ROS message object to a list"
  (cl:list 'EncoderDerivativeParameters
    (cl:cons ':max_window_width (max_window_width msg))
    (cl:cons ':min_angle (min_angle msg))
))
