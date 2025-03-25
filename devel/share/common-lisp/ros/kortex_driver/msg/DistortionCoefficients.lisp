; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude DistortionCoefficients.msg.html

(cl:defclass <DistortionCoefficients> (roslisp-msg-protocol:ros-message)
  ((k1
    :reader k1
    :initarg :k1
    :type cl:float
    :initform 0.0)
   (k2
    :reader k2
    :initarg :k2
    :type cl:float
    :initform 0.0)
   (k3
    :reader k3
    :initarg :k3
    :type cl:float
    :initform 0.0)
   (p1
    :reader p1
    :initarg :p1
    :type cl:float
    :initform 0.0)
   (p2
    :reader p2
    :initarg :p2
    :type cl:float
    :initform 0.0))
)

(cl:defclass DistortionCoefficients (<DistortionCoefficients>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DistortionCoefficients>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DistortionCoefficients)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<DistortionCoefficients> is deprecated: use kortex_driver-msg:DistortionCoefficients instead.")))

(cl:ensure-generic-function 'k1-val :lambda-list '(m))
(cl:defmethod k1-val ((m <DistortionCoefficients>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:k1-val is deprecated.  Use kortex_driver-msg:k1 instead.")
  (k1 m))

(cl:ensure-generic-function 'k2-val :lambda-list '(m))
(cl:defmethod k2-val ((m <DistortionCoefficients>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:k2-val is deprecated.  Use kortex_driver-msg:k2 instead.")
  (k2 m))

(cl:ensure-generic-function 'k3-val :lambda-list '(m))
(cl:defmethod k3-val ((m <DistortionCoefficients>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:k3-val is deprecated.  Use kortex_driver-msg:k3 instead.")
  (k3 m))

(cl:ensure-generic-function 'p1-val :lambda-list '(m))
(cl:defmethod p1-val ((m <DistortionCoefficients>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:p1-val is deprecated.  Use kortex_driver-msg:p1 instead.")
  (p1 m))

(cl:ensure-generic-function 'p2-val :lambda-list '(m))
(cl:defmethod p2-val ((m <DistortionCoefficients>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:p2-val is deprecated.  Use kortex_driver-msg:p2 instead.")
  (p2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DistortionCoefficients>) ostream)
  "Serializes a message object of type '<DistortionCoefficients>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'k1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'k2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'k3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'p1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'p2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DistortionCoefficients>) istream)
  "Deserializes a message object of type '<DistortionCoefficients>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'k1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'k2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'k3) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'p1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'p2) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DistortionCoefficients>)))
  "Returns string type for a message object of type '<DistortionCoefficients>"
  "kortex_driver/DistortionCoefficients")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DistortionCoefficients)))
  "Returns string type for a message object of type 'DistortionCoefficients"
  "kortex_driver/DistortionCoefficients")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DistortionCoefficients>)))
  "Returns md5sum for a message object of type '<DistortionCoefficients>"
  "3d597195629c570d527b87022489fa04")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DistortionCoefficients)))
  "Returns md5sum for a message object of type 'DistortionCoefficients"
  "3d597195629c570d527b87022489fa04")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DistortionCoefficients>)))
  "Returns full string definition for message of type '<DistortionCoefficients>"
  (cl:format cl:nil "~%float32 k1~%float32 k2~%float32 k3~%float32 p1~%float32 p2~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DistortionCoefficients)))
  "Returns full string definition for message of type 'DistortionCoefficients"
  (cl:format cl:nil "~%float32 k1~%float32 k2~%float32 k3~%float32 p1~%float32 p2~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DistortionCoefficients>))
  (cl:+ 0
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DistortionCoefficients>))
  "Converts a ROS message object to a list"
  (cl:list 'DistortionCoefficients
    (cl:cons ':k1 (k1 msg))
    (cl:cons ':k2 (k2 msg))
    (cl:cons ':k3 (k3 msg))
    (cl:cons ':p1 (p1 msg))
    (cl:cons ':p2 (p2 msg))
))
