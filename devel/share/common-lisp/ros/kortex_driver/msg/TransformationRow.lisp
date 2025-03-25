; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude TransformationRow.msg.html

(cl:defclass <TransformationRow> (roslisp-msg-protocol:ros-message)
  ((c0
    :reader c0
    :initarg :c0
    :type cl:float
    :initform 0.0)
   (c1
    :reader c1
    :initarg :c1
    :type cl:float
    :initform 0.0)
   (c2
    :reader c2
    :initarg :c2
    :type cl:float
    :initform 0.0)
   (c3
    :reader c3
    :initarg :c3
    :type cl:float
    :initform 0.0))
)

(cl:defclass TransformationRow (<TransformationRow>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TransformationRow>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TransformationRow)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<TransformationRow> is deprecated: use kortex_driver-msg:TransformationRow instead.")))

(cl:ensure-generic-function 'c0-val :lambda-list '(m))
(cl:defmethod c0-val ((m <TransformationRow>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:c0-val is deprecated.  Use kortex_driver-msg:c0 instead.")
  (c0 m))

(cl:ensure-generic-function 'c1-val :lambda-list '(m))
(cl:defmethod c1-val ((m <TransformationRow>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:c1-val is deprecated.  Use kortex_driver-msg:c1 instead.")
  (c1 m))

(cl:ensure-generic-function 'c2-val :lambda-list '(m))
(cl:defmethod c2-val ((m <TransformationRow>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:c2-val is deprecated.  Use kortex_driver-msg:c2 instead.")
  (c2 m))

(cl:ensure-generic-function 'c3-val :lambda-list '(m))
(cl:defmethod c3-val ((m <TransformationRow>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:c3-val is deprecated.  Use kortex_driver-msg:c3 instead.")
  (c3 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TransformationRow>) ostream)
  "Serializes a message object of type '<TransformationRow>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'c0))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'c1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'c2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'c3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TransformationRow>) istream)
  "Deserializes a message object of type '<TransformationRow>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'c0) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'c1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'c2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'c3) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TransformationRow>)))
  "Returns string type for a message object of type '<TransformationRow>"
  "kortex_driver/TransformationRow")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TransformationRow)))
  "Returns string type for a message object of type 'TransformationRow"
  "kortex_driver/TransformationRow")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TransformationRow>)))
  "Returns md5sum for a message object of type '<TransformationRow>"
  "0b9e4dcf087a9f1dd44881ce6cb761b8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TransformationRow)))
  "Returns md5sum for a message object of type 'TransformationRow"
  "0b9e4dcf087a9f1dd44881ce6cb761b8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TransformationRow>)))
  "Returns full string definition for message of type '<TransformationRow>"
  (cl:format cl:nil "~%float32 c0~%float32 c1~%float32 c2~%float32 c3~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TransformationRow)))
  "Returns full string definition for message of type 'TransformationRow"
  (cl:format cl:nil "~%float32 c0~%float32 c1~%float32 c2~%float32 c3~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TransformationRow>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TransformationRow>))
  "Converts a ROS message object to a list"
  (cl:list 'TransformationRow
    (cl:cons ':c0 (c0 msg))
    (cl:cons ':c1 (c1 msg))
    (cl:cons ':c2 (c2 msg))
    (cl:cons ':c3 (c3 msg))
))
