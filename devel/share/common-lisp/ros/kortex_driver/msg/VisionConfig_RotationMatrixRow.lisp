; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude VisionConfig_RotationMatrixRow.msg.html

(cl:defclass <VisionConfig_RotationMatrixRow> (roslisp-msg-protocol:ros-message)
  ((column1
    :reader column1
    :initarg :column1
    :type cl:float
    :initform 0.0)
   (column2
    :reader column2
    :initarg :column2
    :type cl:float
    :initform 0.0)
   (column3
    :reader column3
    :initarg :column3
    :type cl:float
    :initform 0.0))
)

(cl:defclass VisionConfig_RotationMatrixRow (<VisionConfig_RotationMatrixRow>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VisionConfig_RotationMatrixRow>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VisionConfig_RotationMatrixRow)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<VisionConfig_RotationMatrixRow> is deprecated: use kortex_driver-msg:VisionConfig_RotationMatrixRow instead.")))

(cl:ensure-generic-function 'column1-val :lambda-list '(m))
(cl:defmethod column1-val ((m <VisionConfig_RotationMatrixRow>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:column1-val is deprecated.  Use kortex_driver-msg:column1 instead.")
  (column1 m))

(cl:ensure-generic-function 'column2-val :lambda-list '(m))
(cl:defmethod column2-val ((m <VisionConfig_RotationMatrixRow>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:column2-val is deprecated.  Use kortex_driver-msg:column2 instead.")
  (column2 m))

(cl:ensure-generic-function 'column3-val :lambda-list '(m))
(cl:defmethod column3-val ((m <VisionConfig_RotationMatrixRow>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:column3-val is deprecated.  Use kortex_driver-msg:column3 instead.")
  (column3 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VisionConfig_RotationMatrixRow>) ostream)
  "Serializes a message object of type '<VisionConfig_RotationMatrixRow>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'column1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'column2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'column3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VisionConfig_RotationMatrixRow>) istream)
  "Deserializes a message object of type '<VisionConfig_RotationMatrixRow>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'column1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'column2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'column3) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VisionConfig_RotationMatrixRow>)))
  "Returns string type for a message object of type '<VisionConfig_RotationMatrixRow>"
  "kortex_driver/VisionConfig_RotationMatrixRow")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VisionConfig_RotationMatrixRow)))
  "Returns string type for a message object of type 'VisionConfig_RotationMatrixRow"
  "kortex_driver/VisionConfig_RotationMatrixRow")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VisionConfig_RotationMatrixRow>)))
  "Returns md5sum for a message object of type '<VisionConfig_RotationMatrixRow>"
  "7749fbbc183a47c4c30d1e4e586c8b66")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VisionConfig_RotationMatrixRow)))
  "Returns md5sum for a message object of type 'VisionConfig_RotationMatrixRow"
  "7749fbbc183a47c4c30d1e4e586c8b66")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VisionConfig_RotationMatrixRow>)))
  "Returns full string definition for message of type '<VisionConfig_RotationMatrixRow>"
  (cl:format cl:nil "~%float32 column1~%float32 column2~%float32 column3~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VisionConfig_RotationMatrixRow)))
  "Returns full string definition for message of type 'VisionConfig_RotationMatrixRow"
  (cl:format cl:nil "~%float32 column1~%float32 column2~%float32 column3~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VisionConfig_RotationMatrixRow>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VisionConfig_RotationMatrixRow>))
  "Converts a ROS message object to a list"
  (cl:list 'VisionConfig_RotationMatrixRow
    (cl:cons ':column1 (column1 msg))
    (cl:cons ':column2 (column2 msg))
    (cl:cons ':column3 (column3 msg))
))
